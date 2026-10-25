"""Top-down demo video of two logged missions (12 m/s wind toward east, customer C1):
shortest path at 35 m (loses track on the headwind leg) beside the wind-aware plan (completes).
Uses results/example_logs.pkl from make_logs.py; writes results/demo_wind_corridors.mp4."""
import pickle

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter

from analyze import C_SHORT, C_WIND, INK, RES
from city import City
from mission import CUSTOMERS, DEPOT
from wind import WindField

plt.rcParams["savefig.bbox"] = None  # tight bbox changes the frame size between frames
STEP = 10  # log rows per frame (1 s of mission time)
CASES = [((12.0, 0.0, 0, "shortest", "dob"), "Shortest path @ 35 m, 8 m/s", C_SHORT),
         ((12.0, 0.0, 0, "wind", "dob"), "Wind-aware plan", C_WIND)]


def main():
    logs = pickle.load(open(f"{RES}/example_logs.pkl", "rb"))
    city = City()
    wind = WindField(city, 12.0, 0.0)
    xs, H = city.height_map(2.0)
    fig, axs = plt.subplots(1, 2, figsize=(10.8, 5.6))
    fig.subplots_adjust(left=0.04, right=0.99, top=0.86, bottom=0.03, wspace=0.06)
    fig.suptitle("Quadrotor delivery through urban wind corridors - wind 12 m/s blowing toward east (20x speed)",
                 fontsize=12)
    fig.text(0.5, 0.905, "background: mean wind speed at cruise height (darker = stronger); grey: buildings above the "
             "cruise layer; dashed: planned route; square: depot; star: customer", ha="center", fontsize=8.5)
    arts = []
    for ax, (key, title, col) in zip(axs, CASES):
        r = logs[key]
        cols, L = r["log"]
        c = {n: i for i, n in enumerate(cols)}
        U, _ = wind.layer(r["z_out"])
        s = np.hypot(U[..., 0], U[..., 1])
        ax.imshow(s.T, origin="lower", extent=[0, city.size] * 2, cmap="Blues", vmin=0, vmax=16)
        ax.imshow(np.ma.masked_equal(H, 0).T > r["z_out"] - 8, origin="lower", extent=[0, city.size] * 2,
                  cmap="Greys", alpha=0.55, vmin=0, vmax=1.4)
        ax.plot(L[:, c["xd"]], L[:, c["yd"]], color="white", lw=1.2, ls="--")
        ax.plot(*DEPOT, "s", ms=9, color=INK, mec="white")
        cx, cy = CUSTOMERS[r["customer"]]
        ax.plot(cx, cy, "*", ms=14, color="gold", mec=INK)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f"{title}  (cruise {r['z_out']:.0f} m)", fontsize=10.5, color=col, fontweight="bold")
        trail, = ax.plot([], [], color=col, lw=2.2)
        dot, = ax.plot([], [], "o", ms=8, color=col, mec="white")
        txt = ax.text(8, city.size - 10, "", va="top", fontsize=9.5, color=INK,
                      bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"))
        status = ax.text(city.size / 2, 30, "", ha="center", fontsize=13, fontweight="bold")
        arts.append((r, L, c, trail, dot, txt, status))
    n = max(len(a[1]) for a in arts) // STEP + 25
    writer = FFMpegWriter(fps=20, bitrate=3000)
    with writer.saving(fig, f"{RES}/demo_wind_corridors.mp4", dpi=110):
        for f in range(n):
            for r, L, c, trail, dot, txt, status in arts:
                k = min(f * STEP, len(L) - 1)
                trail.set_data(L[:k + 1, c["x"]], L[:k + 1, c["y"]])
                dot.set_data([L[k, c["x"]]], [L[k, c["y"]]])
                err = np.linalg.norm(L[k, [c["x"], c["y"], c["z"]]] - L[k, [c["xd"], c["yd"], c["zd"]]])
                wspd = np.hypot(L[k, c["wx"]], L[k, c["wy"]])
                txt.set_text(f"t = {L[k, c['t']]:5.0f} s   altitude {L[k, c['z']]:4.0f} m\n"
                             f"wind at drone {wspd:4.1f} m/s\npower {L[k, c['P']]:4.0f} W   energy "
                             f"{L[k, c['E']] / 3600:4.1f} Wh\ntracking error {err:5.1f} m")
                if k == len(L) - 1:
                    if r["success"]:
                        status.set_text("DELIVERED AND RETURNED")
                        status.set_color("#1baf7a")
                    else:
                        status.set_text("LOST TRACK - MISSION FAILED")
                        status.set_color("#b3261e")
                    status.set_bbox(dict(facecolor="white", alpha=0.9, edgecolor="none"))
            writer.grab_frame()
    print("saved", f"{RES}/demo_wind_corridors.mp4", n, "frames")


if __name__ == "__main__":
    main()
