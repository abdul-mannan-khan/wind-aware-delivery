"""Real city geometry from OpenStreetMap, with the same interface as city.City.

Building footprints and heights (tag `height`, else `building:levels` x 3.2 m + 2 m, else a default) are
rasterised at 1 m into a square area. The street network is not taken from OSM: streets are the open space
between buildings, and their axis, width and "avenue" flag are estimated from the raster (structure tensor of
the building mask and distance transform), so the wind model can channel the flow along real, arbitrarily
oriented streets.

  python3 osm_city.py fetch london   ../data/cities/london.npz     # download (needs internet) and rasterise
  python3 osm_city.py show  ../data/cities/london.npz              # picture of the height map, depot, customers

In an experiment config:  city: {osm: ../data/cities/london.npz}
"""
import json
import sys
import urllib.parse
import urllib.request

import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter, maximum_filter, sobel

AREAS = {   # centre (lat, lon) of a 640 m x 640 m area
    "london": (51.5133, -0.0835, "City of London around Bank and Leadenhall Street"),
    "manhattan": (40.7549, -73.9840, "Midtown Manhattan around Bryant Park"),
}
SIZE = 640.0
DEFAULT_H = 12.0
TARGETS = [(18.0, 18.0), (618.0, 442.0), (274.0, 618.0), (618.0, 106.0), (362.0, 362.0)]  # depot + 4 customers


# ---------------------------------------------------------------------------------- fetch
def _height(tags):
    for k in ("height", "building:height"):
        if k in tags:
            try:
                return float(str(tags[k]).split()[0].replace("m", "").replace(",", "."))
            except ValueError:
                pass
    if "building:levels" in tags:
        try:
            return float(tags["building:levels"]) * 3.2 + 2.0
        except ValueError:
            pass
    return None


def fetch(name, out):
    lat0, lon0, desc = AREAS[name]
    m_lat = 111320.0
    m_lon = 111320.0 * np.cos(np.radians(lat0))
    half = SIZE / 2 + 40.0
    bbox = (lat0 - half / m_lat, lon0 - half / m_lon, lat0 + half / m_lat, lon0 + half / m_lon)
    q = f"""[out:json][timeout:120];
(way["building"]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]});
 relation["building"]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}););
out geom;"""
    req = urllib.request.Request("https://overpass-api.de/api/interpreter",
                                 data=urllib.parse.urlencode({"data": q}).encode(),
                                 headers={"User-Agent": "wind-aware-delivery research"})
    data = json.load(urllib.request.urlopen(req, timeout=180))
    polys = []
    for el in data["elements"]:
        tags = el.get("tags", {})
        h = _height(tags)
        rings = []
        if el["type"] == "way" and "geometry" in el:
            rings = [el["geometry"]]
        elif el["type"] == "relation":
            rings = [mb["geometry"] for mb in el.get("members", []) if mb.get("role") == "outer" and "geometry" in mb]
        for g in rings:
            xy = np.array([((p["lon"] - lon0) * m_lon + SIZE / 2, (p["lat"] - lat0) * m_lat + SIZE / 2) for p in g])
            polys.append((xy, h))
    H = rasterise(polys)
    known = sum(h is not None for _, h in polys)
    np.savez_compressed(out, H=H.astype(np.float32), lat0=lat0, lon0=lon0, size=SIZE, name=name, desc=desc,
                        n_buildings=len(polys), n_height_known=known,
                        source="OpenStreetMap contributors (ODbL), Overpass API")
    print(f"{out}: {len(polys)} building polygons, {known} with height or levels; max height {H.max():.0f} m")


def rasterise(polys, res=1.0):
    from matplotlib.path import Path
    n = int(SIZE / res)
    c = (np.arange(n) + 0.5) * res
    X, Y = np.meshgrid(c, c, indexing="ij")
    pts = np.column_stack([X.ravel(), Y.ravel()])
    H = np.zeros(n * n)
    for xy, h in polys:
        lo, hi = xy.min(0), xy.max(0)
        if hi[0] < 0 or hi[1] < 0 or lo[0] > SIZE or lo[1] > SIZE:
            continue
        sel = (pts[:, 0] >= lo[0]) & (pts[:, 0] <= hi[0]) & (pts[:, 1] >= lo[1]) & (pts[:, 1] <= hi[1])
        if not sel.any():
            continue
        inside = Path(xy).contains_points(pts[sel])
        idx = np.flatnonzero(sel)[inside]
        H[idx] = np.maximum(H[idx], h if h is not None else DEFAULT_H)
    return H.reshape(n, n)


# ---------------------------------------------------------------------------------- city
class OSMCity:
    """Raster city with the interface used by WindField, Planner and mission.py."""

    def __init__(self, osm, canyon_coherence=0.35, avenue_width=28.0):
        d = np.load(osm, allow_pickle=False)
        self.H1 = d["H"].astype(float)               # 1 m raster, H1[ix, iy]
        self.size = float(d["size"])
        self.name = str(d["name"])
        self.res1 = self.size / self.H1.shape[0]
        self.coh, self.aw = canyon_coherence, avenue_width
        self.buildings = np.zeros((0, 5))            # not used: clearance comes from the raster
        self._clear = {}
        self.depot, self.customers = self._pick_points()

    # rasters ---------------------------------------------------------------------------
    def height_map(self, res):
        k = max(int(round(res / self.res1)), 1)
        n = self.H1.shape[0] // k
        H = self.H1[:n * k, :n * k].reshape(n, k, n, k).max(axis=(1, 3))
        xs = (np.arange(n) + 0.5) * res
        return xs, H

    def street_axis(self, res):
        """Per cell: canyon flag, unit street axis (n,n,2), street width (m) and avenue flag."""
        xs, H = self.height_map(res)
        solid = (H > 2.0).astype(float)
        sig = 8.0 / res
        gx, gy = sobel(gaussian_filter(solid, 1.0), 0), sobel(gaussian_filter(solid, 1.0), 1)
        Jxx, Jyy, Jxy = (gaussian_filter(a, sig) for a in (gx * gx, gy * gy, gx * gy))
        tr = Jxx + Jyy + 1e-12
        lam_d = np.sqrt((Jxx - Jyy) ** 2 + 4 * Jxy ** 2)
        coherence = lam_d / tr
        ang_n = 0.5 * np.arctan2(2 * Jxy, Jxx - Jyy)     # direction across the street (gradient)
        axis = np.stack([-np.sin(ang_n), np.cos(ang_n)], axis=-1)
        free = solid < 0.5
        dist = distance_transform_edt(free) * res
        width = 2.0 * maximum_filter(dist, size=int(round(30.0 / res)) | 1)
        canyon = free & (coherence > self.coh)
        return xs, canyon, axis, width, canyon & (width > self.aw)

    def clearance(self, p):
        """Horizontal distance (m) from p to the nearest building taller than p_z - 1 m."""
        zk = int(np.clip((p[2] - 1.0) // 2.0, 0, 60))
        if zk not in self._clear:
            occ = self.H1 > 2.0 * zk + 1.0
            self._clear[zk] = distance_transform_edt(~occ).astype(np.float32) * self.res1 if occ.any() \
                else np.full(self.H1.shape, 99.0, np.float32)
        i = int(np.clip(p[0] / self.res1, 0, self.H1.shape[0] - 1))
        j = int(np.clip(p[1] / self.res1, 0, self.H1.shape[1] - 1))
        return float(min(self._clear[zk][i, j], 99.0))

    def inside_building(self, p, margin=0.0):
        return self.clearance(p) <= margin and p[2] <= self.H1[int(np.clip(p[0], 0, self.size - 1)),
                                                               int(np.clip(p[1], 0, self.size - 1))]

    def _pick_points(self):
        """Depot and customers: the open street cell (>= 7 m from walls) nearest to the procedural positions."""
        free = self.H1 <= 2.0
        dist = distance_transform_edt(free) * self.res1
        ok = np.argwhere(dist >= 7.0)
        out = []
        for tx, ty in TARGETS:
            c = ok[np.argmin(np.hypot(ok[:, 0] - tx / self.res1, ok[:, 1] - ty / self.res1))]
            out.append(np.array([(c[0] + 0.5) * self.res1, (c[1] + 0.5) * self.res1]))
        return out[0], out[1:]


def show(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    c = OSMCity(path)
    xs, H = c.height_map(2.0)
    _, canyon, axis, width, aven = c.street_axis(4.0)
    fig, ax = plt.subplots(1, 2, figsize=(11, 5.2))
    im = ax[0].imshow(np.ma.masked_equal(H, 0).T, origin="lower", extent=[0, c.size] * 2, cmap="viridis")
    plt.colorbar(im, ax=ax[0], label="building height (m)")
    ax[0].plot(*c.depot, "rs", ms=8)
    for k, cu in enumerate(c.customers):
        ax[0].plot(*cu, "r^", ms=8)
        ax[0].text(cu[0] + 8, cu[1] + 8, f"C{k}", color="r")
    ax[0].set_title(f"{c.name}: buildings, depot (square), customers")
    xs4 = (np.arange(canyon.shape[0]) + 0.5) * 4.0
    X, Y = np.meshgrid(xs4, xs4, indexing="ij")
    s = (slice(None, None, 3), slice(None, None, 3))
    m = canyon[s]
    ax[1].imshow((H > 2).T, origin="lower", extent=[0, c.size] * 2, cmap="Greys", vmax=2)
    ax[1].quiver(X[s][m], Y[s][m], axis[..., 0][s][m], axis[..., 1][s][m], np.where(aven[s][m], 1, 0),
                 cmap="coolwarm", headwidth=1, headlength=0, pivot="mid", scale=40)
    ax[1].set_title("estimated street axes (red: avenues)")
    for a in ax:
        a.set_xlim(0, c.size)
        a.set_ylim(0, c.size)
        a.set_aspect("equal")
    fig.tight_layout()
    out = path.replace(".npz", ".png")
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    if sys.argv[1] == "fetch":
        fetch(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "show":
        show(sys.argv[2])
