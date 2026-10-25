"""Procedural city: a regular grid of building blocks separated by streets and wider avenues.

Avenues (every third street) are the "wind corridors": they are wider and channel the wind
when the wind blows roughly along them.
"""
import numpy as np


class City:
    def __init__(self, n_blocks=7, block=60.0, street=20.0, avenue=36.0, avenue_every=3,
                 h_min=10.0, h_max=85.0, seed=1):
        rng = np.random.default_rng(seed)
        self.block, self.street, self.avenue = block, street, avenue
        # street widths between blocks along one axis (index k = gap before block k)
        widths = [avenue if k % avenue_every == 0 else street for k in range(n_blocks + 1)]
        edges = [0.0]
        self.gaps = []          # (start, end, is_avenue) of each street along an axis
        x = 0.0
        for k in range(n_blocks):
            self.gaps.append((x, x + widths[k], widths[k] == avenue))
            x += widths[k]
            edges.append(x)
            x += block
            edges.append(x)
        self.gaps.append((x, x + widths[-1], widths[-1] == avenue))
        x += widths[-1]
        self.size = x
        self.block_lo = np.array([edges[2 * k + 1] for k in range(n_blocks)])
        # each block is split into 2x2 buildings with a 4 m alley; heights drawn per building
        self.buildings = []     # (x0, y0, x1, y1, h)
        half = (block - 4.0) / 2
        for i in range(n_blocks):
            for j in range(n_blocks):
                x0, y0 = self.block_lo[i], self.block_lo[j]
                # taller towers near the centre of the city (central business district)
                c = self.size / 2
                r = np.hypot(x0 + block / 2 - c, y0 + block / 2 - c) / (self.size / 2)
                for a in range(2):
                    for b in range(2):
                        bx, by = x0 + a * (half + 4.0), y0 + b * (half + 4.0)
                        h = h_min + (h_max - h_min) * np.clip(1.0 - r, 0, 1) ** 1.5 * rng.uniform(0.6, 1.0) \
                            + rng.uniform(0, 10)
                        self.buildings.append((bx, by, bx + half, by + half, float(h)))
        self.buildings = np.array(self.buildings)

    # -- rasters -------------------------------------------------------------------------
    def height_map(self, res):
        n = int(np.ceil(self.size / res))
        xs = (np.arange(n) + 0.5) * res
        H = np.zeros((n, n))
        for x0, y0, x1, y1, h in self.buildings:
            ix = (xs >= x0) & (xs < x1)
            iy = (xs >= y0) & (xs < y1)
            H[np.ix_(ix, iy)] = np.maximum(H[np.ix_(ix, iy)], h)
        return xs, H

    def street_class(self, res):
        """Per-cell label: 0 building/alley block, 1 street along x, 2 street along y, 3 intersection.
        Also returns a per-cell street width (m) and an avenue flag."""
        n = int(np.ceil(self.size / res))
        xs = (np.arange(n) + 0.5) * res

        def in_gap(v):
            idx = np.full(v.shape, -1)
            for k, (a, b, _) in enumerate(self.gaps):
                idx[(v >= a) & (v < b)] = k
            return idx

        gx, gy = in_gap(xs), in_gap(xs)
        GX, GY = np.meshgrid(gx, gy, indexing="ij")
        cls = np.zeros((n, n), dtype=int)
        cls[(GY >= 0) & (GX < 0)] = 1      # y inside a gap -> street running along x
        cls[(GX >= 0) & (GY < 0)] = 2      # street running along y
        cls[(GX >= 0) & (GY >= 0)] = 3
        width = np.zeros((n, n))
        aven = np.zeros((n, n), dtype=bool)
        for k, (a, b, av) in enumerate(self.gaps):
            m = (GY == k) & (cls == 1)
            width[m], aven[m] = b - a, av
            m = (GX == k) & (cls == 2)
            width[m], aven[m] = b - a, av
            m = ((GX == k) | (GY == k)) & (cls == 3)
            width[m] = np.maximum(width[m], b - a)
            aven[m] |= av
        return xs, cls, width, aven

    def inside_building(self, p, margin=0.0):
        x, y, z = p
        b = self.buildings
        m = (x >= b[:, 0] - margin) & (x <= b[:, 2] + margin) & (y >= b[:, 1] - margin) & \
            (y <= b[:, 3] + margin) & (z <= b[:, 4] + margin)
        return bool(m.any())
