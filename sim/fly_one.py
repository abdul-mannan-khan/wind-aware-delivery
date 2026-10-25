"""Fly ONE delivery mission and plot it (the best way to see what the simulator does).

Usage:
    python fly_one.py                                   # 12 m/s wind toward 180 deg, customer 0, wind-aware, PID+DOB
    python fly_one.py --U 16 --theta 0 --planner shortest --controller pid --customer 2
    python fly_one.py --config ../configs/heavy_parcel.yaml --U 12 --theta 90

Prints the mission result and saves ../results/single/<name>.png: route over the building map with the mean wind
at cruise height, and time series of tracking error, wind speed and battery energy.
"""
import argparse
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import mission
from city import City
from run_experiments import apply_config, load_config, make_city, make_params
from wind import WindField


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--U", type=float, default=12.0, help="reference wind speed at 60 m (m/s)")
    ap.add_argument("--theta", type=float, default=180.0, help="direction the wind blows TOWARD (deg, 0 = east)")
    ap.add_argument("--customer", type=int, default=0)
    ap.add_argument("--planner", default="wind", choices=["shortest", "energy0", "wind", "cutoff", "egate", "rienecker"])
    ap.add_argument("--controller", default="dob", choices=["pid", "dob"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--config", help="optional YAML file with vehicle/planner/wind/gust/city changes")
    a = ap.parse_args()

    cfg = load_config(a.config) if a.config else {}
    apply_config(cfg)
    P = make_params()
    city = make_city(cfg.get("city"))
    wind = WindField(city, a.U, a.theta, **(cfg.get("wind") or {}))
    t0 = time.time()
    r = mission.run_mission(a.U, a.theta, a.customer, a.planner, a.controller, a.seed, keep_log=True,
                            city=city, wind=wind, P=P)
    print(f"simulated in {time.time() - t0:.0f} s")
    for k, v in r.items():
        if k not in ("log", "pred"):
            print(f"  {k:15s} {v:.3f}" if isinstance(v, float) else f"  {k:15s} {v}")
    if r.get("rejected"):
        print("The wind-aware planner REFUSED this mission (no route keeps the steady tilt below the limit).")
        return

    cols, L = r["log"]
    c = {n: i for i, n in enumerate(cols)}
    pred = r["pred"]
    fig = plt.figure(figsize=(12, 5.6))
    ax = fig.add_axes([0.04, 0.08, 0.44, 0.86])
    xs, H = city.height_map(4.0)
    ax.imshow(np.ma.masked_equal(H, 0).T, origin="lower", extent=[0, city.size] * 2, cmap="Greys", vmin=0, vmax=90)
    Uz, _ = wind.layer(pred["z_out"])
    s = 6
    X, Y = np.meshgrid(wind.xs[::s], wind.xs[::s], indexing="ij")
    ax.quiver(X, Y, Uz[::s, ::s, 0], Uz[::s, ::s, 1], color="#1f6fb2", alpha=0.6, scale=300, width=0.002)
    ax.plot(L[:, c["xd"]], L[:, c["yd"]], color="#999999", lw=1, ls="--", label="planned path")
    ax.plot(L[:, c["x"]], L[:, c["y"]], color="#eb6834", lw=1.8, label="flown path")
    ax.plot(*mission.DEPOT, "ks", ms=8, label="depot")
    ax.plot(*mission.CUSTOMERS[a.customer], "k*", ms=13, label="customer")
    ax.set_title(f"{a.planner} + {a.controller}, U = {a.U:g} m/s toward {a.theta:g}°  "
                 f"(cruise {pred['z_out']:.0f} m out / {pred['z_back']:.0f} m back; arrows = wind at {pred['z_out']:.0f} m)",
                 fontsize=8)
    ax.legend(fontsize=7, loc="upper left")
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    t = L[:, c["t"]]
    err = np.linalg.norm(L[:, [c["x"], c["y"], c["z"]]] - L[:, [c["xd"], c["yd"], c["zd"]]], axis=1)
    wsp = np.linalg.norm(L[:, [c["wx"], c["wy"], c["wz"]]], axis=1)
    for k, (y, lab) in enumerate([(err, "tracking error (m)"), (wsp, "wind at drone (m/s)"),
                                  (L[:, c["E"]] / 3600, "energy used (Wh)")]):
        a2 = fig.add_axes([0.56, 0.08 + (2 - k) * 0.31, 0.42, 0.25])
        a2.plot(t, y, color="#333333", lw=1.2)
        a2.set_ylabel(lab, fontsize=8)
        a2.grid(alpha=0.3)
        if k == 2:
            a2.axhline(P.battery_Wh, color="r", ls=":", lw=1)
            a2.set_xlabel("time (s)")
    os.makedirs("../results/single", exist_ok=True)
    name = f"U{a.U:g}_th{a.theta:g}_c{a.customer}_{a.planner}_{a.controller}_s{a.seed}"
    if cfg.get("name"):
        name = f"{cfg['name']}_{name}"
    out = f"../results/single/{name}.png"
    fig.savefig(out, dpi=130)
    print("saved", out)


if __name__ == "__main__":
    main()
