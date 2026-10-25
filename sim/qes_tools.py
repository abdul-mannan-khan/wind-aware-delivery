"""Run QES-Winds on the cities of this study and bring its fields onto the simulator's wind grid.

  python3 qes_tools.py inputs  CITY OUTDIR [--dirs 0,45,...]   # buildings shapefile + one XML per wind direction
  python3 qes_tools.py convert CITY OUTDIR                      # QES NetCDF -> OUTDIR/qes_thXXX.npz (simulator grid)
CITY is 'procedural' or a path to an OSM city .npz (osm_city.py). Wind direction theta is where the air goes TO
(0 = +x/east); QES uses the meteorological direction the wind comes FROM, clockwise from north: 270 - theta.
The inflow profile is the analytical model's power law, U(z) = U_run (z / z_ref)^alpha, given to QES as a
measured profile (boundaryLayerFlag 4) at one site, so both models start from the same approach flow.
Building footprints are taken from the same 1 m raster as the simulator (procedural: exact rectangles).
QES places the buildings relative to the lower-left corner of the shapefile; 'convert' finds that offset by matching
QES's building cells to the raster, and checks it.
"""
import argparse
import glob
import json
import os
import subprocess

import numpy as np

U_RUN = 10.0                       # m/s at z_ref; fields are scaled linearly to other speeds
DIRS = (0, 45, 90, 135, 180, 225, 270, 315)
DXY = 2.0


def load_city(spec):
    if spec == "procedural":
        from city import City
        return City()
    from osm_city import OSMCity
    return OSMCity(spec)


def footprints(city):
    """List of (polygon xy (k,2), height) in simulator coordinates."""
    if hasattr(city, "H1"):
        from rasterio import features, transform
        H = np.round(city.H1.T * 2) / 2                          # (y, x) image, heights to 0.5 m
        H[H <= 2.0] = 0
        tr = transform.from_origin(0, city.size, city.res1, city.res1)
        img = H[::-1].astype(np.float32)                         # row 0 = top (y = size)
        out = []
        for geom, val in features.shapes(img, mask=img > 0, transform=tr):
            for ring in geom["coordinates"][:1]:                 # outer ring
                out.append((np.array(ring), float(val)))
        return out
    rect = lambda x0, y0, x1, y1: np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]])
    out = [(rect(x0, y0, x1, y1), float(h)) for x0, y0, x1, y1, h in city.buildings]
    # QES places the domain at the lower-left corner of the footprints: anchor it at the city's corners with
    # two 2 m x 2 m, 1 m tall blocks so that the QES grid covers the whole city (no extrapolated strips)
    out += [(rect(0, 0, 2, 2), 1.0), (rect(city.size - 2, city.size - 2, city.size, city.size), 1.0)]
    return out


def write_shp(polys, path):
    import shapefile                                             # pyshp
    w = shapefile.Writer(path, shapeType=shapefile.POLYGON)
    w.field("H", "N", decimal=2)
    for xy, h in polys:
        area2 = np.sum(xy[:-1, 0] * xy[1:, 1] - xy[1:, 0] * xy[:-1, 1])
        ring = xy if area2 < 0 else xy[::-1]                     # ESRI outer rings are clockwise
        w.poly([ring.tolist()])
        w.record(h)
    w.close()


def xml(name, shp, n, nz, dz, theta, z_ref, alpha):
    zs = [2, 5, 10, 15, 20, 30, 40, 50, 60, 80, 100, 150, 200, 300, 400, 600]
    met = (270.0 - theta) % 360.0
    prof = "".join(f"<height>{z}</height>" for z in zs) + "".join(
        f"<speed>{U_RUN * (z / z_ref) ** alpha:.4f}</speed>" for z in zs) + "".join(
        f"<direction>{met:.1f}</direction>" for _ in zs)
    return f"""<QESWindsParameters>
  <simulationParameters>
    <halo_x>0.0</halo_x><halo_y>0.0</halo_y>
    <domain>{n} {n} {nz}</domain>
    <cellSize>{DXY} {DXY} {dz}</cellSize>
    <verticalStretching>0</verticalStretching>
    <totalTimeIncrements>1</totalTimeIncrements>
    <logLawFlag>0</logLawFlag>
    <maxIterations>500</maxIterations>
    <tolerance>1e-9</tolerance>
    <meshTypeFlag>0</meshTypeFlag>
    <domainRotation>0</domainRotation>
    <UTMx>0</UTMx><UTMy>0</UTMy><UTMZone>1</UTMZone><UTMZoneLetter>17</UTMZoneLetter>
    <readCoefficientsFlag>0</readCoefficientsFlag>
  </simulationParameters>
  <metParams>
    <z0_domain_flag>0</z0_domain_flag>
    <sensor>
      <site_coord_flag>1</site_coord_flag>
      <site_xcoord>1.0</site_xcoord><site_ycoord>1.0</site_ycoord>
      <timeSeries>
        <timeStamp>2026-01-01T00:00:00</timeStamp>
        <boundaryLayerFlag>4</boundaryLayerFlag>
        <siteZ0>0.5</siteZ0>
        <reciprocal>0.0</reciprocal>
        {prof}
      </timeSeries>
    </sensor>
  </metParams>
  <buildingsParams>
    <wallRoughness>0.01</wallRoughness>
    <rooftopFlag>1</rooftopFlag>
    <upwindCavityFlag>2</upwindCavityFlag>
    <streetCanyonFlag>1</streetCanyonFlag>
    <streetIntersectionFlag>1</streetIntersectionFlag>
    <wakeFlag>2</wakeFlag>
    <highRiseFlag>0</highRiseFlag>
    <sidewallFlag>1</sidewallFlag>
    <SHPFile>{shp}.shp</SHPFile>
    <SHPBuildingLayer>{os.path.basename(shp)}</SHPBuildingLayer>
    <SHPHeightField>H</SHPHeightField>
    <heightFactor>1.0</heightFactor>
  </buildingsParams>
  <fileOptions>
    <outputFlag>1</outputFlag>
    <outputFields>u</outputFields><outputFields>v</outputFields><outputFields>w</outputFields>
    <outputFields>icell</outputFields>
  </fileOptions>
</QESWindsParameters>
"""


def inputs(spec, out, dirs):
    from wind import WindField
    os.makedirs(out, exist_ok=True)
    city = load_city(spec)
    polys = footprints(city)
    shp = os.path.join(out, "buildings")
    write_shp(polys, shp)
    xy = np.concatenate([p for p, _ in polys])
    hmax = max(h for _, h in polys)
    size = city.size
    n = int(np.ceil(size / DXY))
    dz = 2.0 if hmax < 150 else 3.0
    nz = int(np.ceil(max(200.0, 1.4 * hmax) / dz))
    z_ref, alpha = 60.0, 0.30                                    # analytical model defaults (wind.py)
    meta = dict(city=spec, n=n, nz=nz, dz=dz, dxy=DXY, U_run=U_RUN, z_ref=z_ref, alpha=alpha,
                shp_min=[float(xy[:, 0].min()), float(xy[:, 1].min())], n_polygons=len(polys), h_max=hmax)
    json.dump(meta, open(os.path.join(out, "meta.json"), "w"), indent=1)
    for th in dirs:
        with open(os.path.join(out, f"th{th:03d}.xml"), "w") as f:
            f.write(xml(f"th{th:03d}", os.path.abspath(shp), n, nz, dz, th, z_ref, alpha))
    print(f"{out}: {len(polys)} footprints, domain {n}x{n}x{nz} ({DXY} m, dz {dz} m), {len(dirs)} directions")


def convert(spec, out):
    import netCDF4
    from scipy.interpolate import RegularGridInterpolator
    from scipy.signal import fftconvolve
    from wind import WindField
    city = load_city(spec)
    meta = json.load(open(os.path.join(out, "meta.json")))
    ref = WindField(city, U_RUN, 0.0)                            # target grid and building heights
    xs, zs, Hb = ref.xs, ref.zs, ref.Hb
    report = {}
    for f in sorted(glob.glob(os.path.join(out, "th*_windsOut.nc"))):
        th = int(os.path.basename(f)[2:5])
        nc = netCDF4.Dataset(f)
        x, y, z = (np.array(nc[k][:]) for k in ("x", "y", "z"))
        u, v, w = (np.array(nc[k][0]) for k in ("u", "v", "w"))            # (z, y, x)
        ic = np.array(nc["icell"][0])
        nc.close()
        # offset of the QES grid: match its building cells near the ground to the simulator raster
        _, Hq = city.height_map(DXY)
        bq = (ic[np.argmin(np.abs(z - 3.0))] == 0).T.astype(float)        # (x, y), 0 = solid in QES
        bs = (Hq > 3.0).astype(float)
        c = fftconvolve(bs, bq[::-1, ::-1], mode="full")
        i, j = np.unravel_index(np.argmax(c), c.shape)
        ox, oy = i - (bq.shape[0] - 1), j - (bq.shape[1] - 1)              # sim index = QES index + offset
        sx, sy = ox * DXY, oy * DXY
        ii = np.arange(bs.shape[0])[:, None] - ox
        jj = np.arange(bs.shape[1])[None, :] - oy
        inside = (ii >= 0) & (ii < bq.shape[0]) & (jj >= 0) & (jj < bq.shape[1])
        bqs = np.where(inside, bq[np.clip(ii, 0, bq.shape[0] - 1), np.clip(jj, 0, bq.shape[1] - 1)], 0.0)
        iou = float((bs * bqs).sum() / max(((bs + bqs) > 0).sum(), 1))
        U = np.zeros((len(xs), len(xs), len(zs), 3))
        X, Y, Z = np.meshgrid(xs, xs, np.maximum(zs, z[0]), indexing="ij")
        pts = np.stack([Z - 0, Y - sy, X - sx], axis=-1)
        for k, comp in enumerate((u, v, w)):
            comp = np.where(ic == 0, 0.0, comp)
            it = RegularGridInterpolator((z, y, x), comp, bounds_error=False, fill_value=None)
            U[..., k] = it(pts)
        for kz, zz in enumerate(zs):
            U[Hb > max(zz, 1.0), kz, :] = 0.0
        np.savez_compressed(os.path.join(out, f"qes_th{th:03d}.npz"), U=U.astype(np.float32), U_run=U_RUN,
                            shift=[sx, sy], iou=iou, source=os.path.basename(f))
        report[th] = dict(shift_m=[sx, sy], building_iou=round(iou, 3))
        print(f"theta {th}: offset ({sx:.0f}, {sy:.0f}) m, building overlap IoU {iou:.3f}")
    json.dump(report, open(os.path.join(out, "convert_report.json"), "w"), indent=1)


def run(out, solver=2):
    for x in sorted(glob.glob(os.path.join(out, "th*.xml"))):
        base = x[:-4]
        if os.path.exists(base + "_windsOut.nc"):
            continue
        with open(base + ".log", "w") as log:
            subprocess.run(["qesWinds", "-q", x, "-s", str(solver), "-o", base], stdout=log, stderr=subprocess.STDOUT,
                           check=True)
        print("done", base, flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["inputs", "run", "convert"])
    ap.add_argument("city")
    ap.add_argument("out")
    ap.add_argument("--dirs", default=",".join(map(str, DIRS)))
    ap.add_argument("--solver", type=int, default=2, help="QES solver: 1 CPU, 2 GPU dynamic parallelism, 3, 4")
    a = ap.parse_args()
    if a.cmd == "inputs":
        inputs(a.city, a.out, [int(t) for t in a.dirs.split(",")])
    elif a.cmd == "run":
        run(a.out, a.solver)
    else:
        convert(a.city, a.out)
