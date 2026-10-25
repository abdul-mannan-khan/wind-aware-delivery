"""Compare the analytical urban wind field (wind.py) with QES-Winds on the same city and approach flow.

  python3 qes_compare.py OUT_CSV_DIR procedural:../results/qes/procedural london:../data/cities/london.npz:../results/qes/london ...

Per city, wind direction and height (outside buildings):
  flow metrics of the horizontal wind speed, as used in urban-flow model evaluation: FAC2 (share of points within a
  factor of two), hit rate q (|difference| <= 25 % of QES or <= 0.05 U_ref), fractional bias FB, normalised mean
  square error NMSE, correlation R, mean absolute direction difference where QES speed > 0.1 U_ref;
  planner metrics at the cruise layers: agreement of the steady-tilt feasibility of every cell and heading (best
  speed, loaded vehicle, 35 deg) at U_ref = 12 and 16 m/s, and correlation of the energy-per-metre costs.
Writes qes_flow_metrics.csv, qes_planner_metrics.csv and qes_maps.npz (speed maps at 20 m for figures).
"""
import os
import sys

import numpy as np
import pandas as pd

from city import City
from mission import SPEEDS_FINE, TILT_LIMIT
from planner import Planner
from qes_wind import QESWindField
from wind import WindField

U_REF = 10.0


def flow_metrics(a, q, U):
    """a: analytical, q: QES speeds (same points)."""
    m = np.isfinite(a) & np.isfinite(q)
    a, q = a[m], q[m]
    r = a / np.maximum(q, 1e-6)
    fac2 = np.mean((r >= 0.5) & (r <= 2.0))
    hit = np.mean((np.abs(a - q) <= 0.25 * np.abs(q)) | (np.abs(a - q) <= 0.05 * U))
    fb = 2 * (q.mean() - a.mean()) / (q.mean() + a.mean())
    nmse = np.mean((a - q) ** 2) / (a.mean() * q.mean())
    R = np.corrcoef(a, q)[0, 1]
    return dict(FAC2=fac2, hit_rate=hit, FB=fb, NMSE=nmse, R=R, mean_analytic=a.mean(), mean_qes=q.mean())


def load_spec(spec):
    parts = spec.split(":")
    if parts[0] == "procedural":
        return parts[0], City(), parts[1]
    from osm_city import OSMCity
    return parts[0], OSMCity(parts[1]), parts[2]


def one(args):
    """Metrics for one city and wind direction."""
    spec, th = args
    name, city, qdir = load_spec(spec)
    flow, plan, maps = [], [], {}
    A = WindField(city, U_REF, th)
    Q = QESWindField(city, U_REF, th, qdir)
    for z in (10.0, 20.0, 35.0, 50.0):
        k = int(round(z / A.dz))
        free = A.Hb < z
        sa = np.hypot(A.U[:, :, k, 0], A.U[:, :, k, 1])[free]
        sq = np.hypot(Q.U[:, :, k, 0], Q.U[:, :, k, 1])[free]
        row = dict(city=name, theta=th, z=z, **flow_metrics(sa, sq, U_REF))
        da = np.arctan2(A.U[:, :, k, 1], A.U[:, :, k, 0])[free]
        dq = np.arctan2(Q.U[:, :, k, 1], Q.U[:, :, k, 0])[free]
        sel = sq > 0.1 * U_REF
        row["dir_err_deg"] = float(np.degrees(np.abs(np.angle(np.exp(1j * (da - dq)))))[sel].mean())
        flow.append(row)
        if z == 20.0:
            maps[f"{name}_th{th:03d}_analytic"] = np.where(free, np.hypot(A.U[:, :, k, 0], A.U[:, :, k, 1]), np.nan)
            maps[f"{name}_th{th:03d}_qes"] = np.where(free, np.hypot(Q.U[:, :, k, 0], Q.U[:, :, k, 1]), np.nan)
    for U in (12.0, 16.0):
        A = WindField(city, U, th)
        Q = QESWindField(city, U, th, qdir)
        for z in (20.0, 35.0, 50.0):
            pa = Planner(city, A, z_cruise=z, v_cruise=max(SPEEDS_FINE), mass=2.2, tilt_limit=TILT_LIMIT,
                         speeds=SPEEDS_FINE, clear_h=4.0)
            pq = Planner(city, Q, z_cruise=z, v_cruise=max(SPEEDS_FINE), mass=2.2, tilt_limit=TILT_LIMIT,
                         speeds=SPEEDS_FINE, clear_h=4.0)
            free = ~pa.occ
            fa, fq = np.isfinite(pa.Epm)[free], np.isfinite(pq.Epm)[free]
            both = fa & fq
            ea, eq = pa.Epm[free][both], pq.Epm[free][both]
            plan.append(dict(city=name, theta=th, U_ref=U, z=z, feasible_analytic=fa.mean(),
                             feasible_qes=fq.mean(), agreement=np.mean(fa == fq),
                             analytic_only=np.mean(fa & ~fq), qes_only=np.mean(fq & ~fa),
                             cost_R=np.corrcoef(ea, eq)[0, 1] if both.sum() > 10 else np.nan,
                             cost_ratio=float(np.median(ea / eq)) if both.sum() > 10 else np.nan))
    print(name, th, flush=True)
    return flow, plan, maps


def main(out, specs, workers=48):
    from multiprocessing import Pool
    os.makedirs(out, exist_ok=True)
    jobs = []
    for spec in specs:
        qdir = spec.split(":")[-1]
        jobs += [(spec, int(f[6:9])) for f in sorted(os.listdir(qdir)) if f.startswith("qes_th") and f.endswith(".npz")]
    flow, plan, maps = [], [], {}
    with Pool(workers) as pool:
        for f, p, m in pool.imap_unordered(one, jobs):
            flow += f
            plan += p
            maps.update(m)
    pd.DataFrame(flow).to_csv(os.path.join(out, "qes_flow_metrics.csv"), index=False)
    pd.DataFrame(plan).to_csv(os.path.join(out, "qes_planner_metrics.csv"), index=False)
    np.savez_compressed(os.path.join(out, "qes_maps.npz"), **maps)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
