"""Grid A* route planning at a fixed cruise altitude (shortest vs. wind-aware energy cost),
path shortcutting, corner rounding and a curvature/acceleration-limited speed profile.

With `speeds` (a tuple of ground speeds) the planner also chooses the cruise speed: the cost of a cell is the
lowest energy per metre over the speeds that respect the tilt limit there, and each straight segment of the final
path is flown at the speed that minimises its energy in the local wind (e.g. faster with a tailwind)."""
import heapq
import numpy as np
from scipy.ndimage import binary_dilation, uniform_filter1d

from quadrotor import Params, trim_power

DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


class Planner:
    def __init__(self, city, wind, z_cruise=30.0, v_cruise=8.0, res=4.0, clear_h=8.0,
                 clear_v=8.0, mass=None, lam_turb=0.0, P=None, tilt_limit=30.0, speeds=None):
        self.city, self.wind, self.z, self.v, self.res = city, wind, z_cruise, v_cruise, res
        self.speeds = tuple(float(v) for v in speeds) if speeds else None
        self.P = P or Params()
        self.mass = mass if mass is not None else self.P.m_frame + self.P.m_payload
        xs, H = city.height_map(res)
        self.xs = xs
        occ = H > z_cruise - clear_v
        r = int(np.ceil(clear_h / res))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        self.occ = binary_dilation(occ, structure=(xx ** 2 + yy ** 2) <= r * r)
        # straight shortcuts and corner rounding are checked against occ_tight: keep at least one cell of margin
        # (with clear_h < 2 res the half-radius rounded down to zero cells, so routes could graze walls)
        rt = max(r // 2, 1)
        self.occ_tight = binary_dilation(occ, structure=(xx ** 2 + yy ** 2) <= rt * rt)
        self.lam = lam_turb
        self.tilt_limit = tilt_limit
        self._cost_tables()

    def set_mass(self, mass):
        self.mass = mass
        self._cost_tables()

    def _cost_tables(self):
        """Energy per metre (J/m) for each cell and each of the 8 headings, from the mean wind. With several
        speeds, Ek[k] is the table at speed k and Epm the cell-wise minimum over the speeds."""
        U, TI = self.wind.layer(self.z)
        n = len(self.xs)
        vs = self.speeds or (self.v,)
        self.Ek = np.zeros((len(vs), n, n, 8))
        # quantise wind to 0.25 m/s to share computations
        key = np.round(U[:, :, :2] / 0.25).astype(int).reshape(-1, 2)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        # power depends on (velocity, wind vector): tabulate on the quantised winds, then look up
        for kv, v in enumerate(vs):
            for d, (dx, dy) in enumerate(DIRS):
                t = np.array([dx, dy, 0.0]) / np.hypot(dx, dy)
                PT = np.array([trim_power(v * t, np.array([k[0] * 0.25, k[1] * 0.25, 0.0]), self.mass, self.P,
                                          return_tilt=True) for k in uniq])
                Pq = np.where(PT[:, 1] > self.tilt_limit, np.inf, PT[:, 0])   # infeasible: needs too much tilt
                self.Ek[kv, :, :, d] = (Pq[inv] / v).reshape(n, n)
        self.Epm = self.Ek.min(axis=0)
        gust = TI * np.linalg.norm(U, axis=2)               # turbulence std (m/s)
        self.turb = gust
        fin = self.Epm[~self.occ]
        fin = fin[np.isfinite(fin)]
        self.Epm_min = fin.min() if len(fin) else 0.0

    def cell(self, p):
        return (int(np.clip(p[0] / self.res, 0, len(self.xs) - 1)), int(np.clip(p[1] / self.res, 0, len(self.xs) - 1)))

    def edge_cost(self, i, j, d, mode):
        L = self.res * (np.sqrt(2) if d % 2 else 1.0)
        if mode == "shortest":
            return L
        return L * (self.Epm[i, j, d] + self.lam * self.turb[i, j])

    def astar(self, start, goal, mode):
        n = len(self.xs)
        s, g = self.cell(start), self.cell(goal)
        h_scale = 1.0 if mode == "shortest" else self.Epm_min
        hfun = lambda c: h_scale * self.res * np.hypot(c[0] - g[0], c[1] - g[1])
        openq = [(hfun(s), 0.0, s)]
        best = {s: 0.0}
        parent = {s: None}
        occ = self.occ.copy()
        occ[s] = occ[g] = False
        while openq:
            f, gc, c = heapq.heappop(openq)
            if c == g:
                break
            if gc > best.get(c, np.inf):
                continue
            for d, (dx, dy) in enumerate(DIRS):
                nb = (c[0] + dx, c[1] + dy)
                if not (0 <= nb[0] < n and 0 <= nb[1] < n) or occ[nb]:
                    continue
                if d % 2 and (occ[c[0] + dx, c[1]] or occ[c[0], c[1] + dy]):
                    continue
                ng = gc + self.edge_cost(c[0], c[1], d, mode)
                if ng < best.get(nb, np.inf):
                    best[nb] = ng
                    parent[nb] = c
                    heapq.heappush(openq, (ng + hfun(nb), ng, nb))
        if g not in parent:
            raise RuntimeError(f"no path at z={self.z}")
        path, c = [], g
        while c is not None:
            path.append(c)
            c = parent[c]
        pts = (np.array(path[::-1], float) + 0.5) * self.res
        pts[0], pts[-1] = start[:2], goal[:2]
        return pts

    # -- cost of an arbitrary polyline (used for shortcutting and for reporting) -------------
    def _seg_costs(self, a, b, step=1.0):
        """Energy of the straight segment a->b at each tabulated speed (array over speeds), its length and
        whether it is collision-free. The energy per metre for an arbitrary heading is interpolated between the
        two neighbouring tabulated headings."""
        L = np.linalg.norm(b - a)
        if L < 1e-9:
            return np.zeros(len(self.Ek)), 0.0, True
        k = max(int(L / step), 1)
        pts = a + (b - a) * (np.arange(k) + 0.5)[:, None] / k
        idx = np.clip((pts / self.res).astype(int), 0, len(self.xs) - 1)
        free = not self.occ_tight[idx[:, 0], idx[:, 1]].any()
        ang = np.arctan2(b[1] - a[1], b[0] - a[0]) % (2 * np.pi)
        fd = ang / (np.pi / 4)
        d0 = int(fd) % 8
        d1, w1 = (d0 + 1) % 8, fd - int(fd)
        e = self.Ek[:, idx[:, 0], idx[:, 1], d0]
        if w1 > 1e-9:
            e = (1 - w1) * e + w1 * self.Ek[:, idx[:, 0], idx[:, 1], d1]
        e = e + self.lam * self.turb[idx[:, 0], idx[:, 1]]
        return e.sum(axis=1) * L / k, L, free

    def line_cost(self, a, b, mode, step=1.0):
        """Cost of a straight segment a->b (flown at its best speed) and whether it is collision-free."""
        c, L, free = self._seg_costs(a, b, step)
        if mode == "shortest" or not free or L < 1e-9:
            return L, free
        return float(c.min()), free

    def seg_speeds(self, pts):
        """Ground speed for each segment of the path: the tabulated speed with the lowest segment energy."""
        vs = self.speeds or (self.v,)
        out = []
        for k in range(len(pts) - 1):
            c = self._seg_costs(pts[k], pts[k + 1])[0]
            out.append(vs[int(np.argmin(c))] if np.isfinite(c).any() else min(vs))
        return np.array(out)

    def path_energy_at(self, pts, speeds):
        """Predicted energy of the path flown at the given per-segment ground speeds (used to score a plan made
        by another planner, e.g. a still-air plan, in this planner's wind)."""
        vs = list(self.speeds or (self.v,))
        lam, self.lam = self.lam, 0.0
        e = 0.0
        for k in range(len(pts) - 1):
            v = float(speeds[k])
            kv = int(np.argmin([abs(v - x) for x in vs]))
            e += self._seg_costs(pts[k], pts[k + 1])[0][kv]
        self.lam = lam
        return e

    def shortcut(self, pts, mode):
        seg = np.array([self.line_cost(pts[k], pts[k + 1], mode)[0] for k in range(len(pts) - 1)])
        cum = np.concatenate([[0], np.cumsum(seg)])
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j_best = i + 1
            # farthest point reachable by a straight, free and not-more-expensive segment
            for j in range(len(pts) - 1, i + 1, -1):
                c_line, free = self.line_cost(pts[i], pts[j], mode)
                tol = 1.0 if self.speeds is None else 1.02    # a long segment has one speed: allow 2 %
                if free and (mode == "shortest" or c_line <= tol * (cum[j] - cum[i])):
                    j_best = j
                    break
            out.append(pts[j_best])
            i = j_best
        return np.array(out)

    def plan(self, start, goal, mode):
        pts = self.astar(start, goal, mode)
        pts = self.shortcut(pts, mode)
        return pts

    def path_energy(self, pts, mode="energy"):
        lam, self.lam = self.lam, 0.0
        e = sum(self.line_cost(pts[k], pts[k + 1], "energy")[0] for k in range(len(pts) - 1))
        self.lam = lam
        return e


# --------------------------------------------------------------------------------------------
def time_parametrise(poly3, v_max, a_max=2.0, a_lat=2.5, ds=0.5, round_m=10.0, occ_check=None):
    """Turn a 3-D polyline into a time-indexed reference (t, p, v, a) with rounded corners.
    v_max is one speed or one speed per polyline segment.
    occ_check(p_xy) -> bool returns True if a smoothed point is in collision."""
    seg = np.diff(poly3, axis=0)
    L = np.linalg.norm(seg, axis=1)
    s_nodes = np.concatenate([[0], np.cumsum(L)])
    s = np.arange(0, s_nodes[-1] + 1e-9, ds)
    if s[-1] < s_nodes[-1]:
        s = np.append(s, s_nodes[-1])
    v_seg = np.broadcast_to(np.asarray(v_max, float), (len(L),))
    vt = v_seg[np.clip(np.searchsorted(s_nodes, s, side="right") - 1, 0, len(L) - 1)]
    P = np.column_stack([np.interp(s, s_nodes, poly3[:, k]) for k in range(3)])
    w = int(round_m / ds) | 1
    while w > 1:
        Ps = P.copy()
        for k in range(3):
            Ps[:, k] = uniform_filter1d(P[:, k], size=w, mode="nearest")
        Ps[0], Ps[-1] = P[0], P[-1]
        if occ_check is None or not any(occ_check(q) for q in Ps[::4]):
            P = Ps
            break
        w = (w // 2) | 1
    d = np.diff(P, axis=0)
    dl = np.linalg.norm(d, axis=1)
    keep = np.concatenate([[True], dl > 1e-6])
    P = P[keep]
    vt = vt[keep]
    d = np.diff(P, axis=0)
    dl = np.linalg.norm(d, axis=1)
    s = np.concatenate([[0], np.cumsum(dl)])
    T = np.gradient(P, s, axis=0)
    T /= np.linalg.norm(T, axis=1, keepdims=True) + 1e-12
    kappa = np.linalg.norm(np.gradient(T, s, axis=0), axis=1)
    vlim = np.minimum(vt, np.sqrt(a_lat / np.maximum(kappa, 1e-6)))
    vlim[0] = vlim[-1] = 0.0
    v = vlim.copy()
    for i in range(1, len(v)):
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_max * dl[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * dl[i]))
    vm = 0.5 * (v[1:] + v[:-1])
    dt = dl / np.maximum(vm, 0.05)
    t = np.concatenate([[0], np.cumsum(dt)])
    Vv = T * v[:, None]
    A = np.gradient(Vv, t, axis=0)
    for k in range(3):
        A[:, k] = uniform_filter1d(A[:, k], size=9, mode="nearest")
    return t, P, Vv, A


class Reference:
    """Concatenation of time-parametrised segments and hover holds, queried by time."""

    def __init__(self):
        self.t, self.P, self.V, self.A = [np.zeros(0)], [np.zeros((0, 3))], [np.zeros((0, 3))], [np.zeros((0, 3))]
        self.events = []   # (time, name)
        self.T_end = 0.0

    def add(self, seg):
        t, P, V, A = seg
        self.t.append(t + self.T_end)
        self.P.append(P), self.V.append(V), self.A.append(A)
        self.T_end += t[-1] + 1e-3

    def hold(self, p, duration, event=None):
        if event:
            self.events.append((self.T_end + 0.5 * duration, event))
        t = np.array([0.0, duration])
        self.add((t, np.array([p, p], float), np.zeros((2, 3)), np.zeros((2, 3))))

    def finalize(self):
        self.t = np.concatenate(self.t)
        self.P, self.V, self.A = np.concatenate(self.P), np.concatenate(self.V), np.concatenate(self.A)

    def __call__(self, t):
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        a = np.clip((t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9), 0, 1)
        f = lambda X: (1 - a) * X[i] + a * X[i + 1]
        return f(self.P), f(self.V), f(self.A)
