"""Summarise one or more campaign files: table + two figures.

Usage:
    python summarise.py ../results/experiments/quick/campaign.csv
    python summarise.py ../results/experiments/baseline/campaign.csv ../results/experiments/heavy_parcel/campaign.csv
    python summarise.py A.csv B.csv --x theta          # put another factor on the horizontal axis

For every experiment (= folder name of the CSV), planner, controller and value of the x factor it reports
  missions, delivered (%), refused before take-off (%), failed in flight (%),
  mean energy of a delivered mission (Wh) and energy per parcel delivered (Wh, failed flights included),
  mean RMS tracking error (m).
Output goes next to the first CSV (or to --out): summary.csv, success_rate.png, energy_per_parcel.png.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PNAME = {"shortest": "Shortest path", "energy0": "Energy (still air)", "wind": "Wind-aware",
         "cutoff": "Wind cut-off", "egate": "Energy gate", "rienecker": "Min-energy route (fixed z, v)"}
COLOR = {"shortest": "#2a78d6", "energy0": "#1baf7a", "wind": "#eb6834",
         "cutoff": "#7d5ba6", "egate": "#8c6d1f", "rienecker": "#d94f8a"}
DASH = {"pid": "-", "dob": "--"}
MARK = ["o", "s", "^", "D", "v", "P"]


def as_bool(s):
    return s.astype(str).str.lower().isin(["true", "1"])


def load(path):
    d = pd.read_csv(path)
    for c in ("success", "rejected", "collided", "tracking_fail"):
        d[c] = as_bool(d[c]) if c in d else False
    d["experiment"] = os.path.basename(os.path.dirname(os.path.abspath(path)))
    return d


def summarise(d, x):
    rows = []
    for key, g in d.groupby(["experiment", "planner", "controller", x]):
        n, ok = len(g), g.success.sum()
        flown = g[~g.rejected]
        rows.append(dict(zip(["experiment", "planner", "controller", x], key),
                         missions=n,
                         delivered_pct=100 * ok / n,
                         refused_pct=100 * g.rejected.mean(),
                         failed_in_flight_pct=100 * ((~g.success) & (~g.rejected)).mean(),
                         energy_delivered_Wh=g.loc[g.success, "E_Wh"].mean() if ok else np.nan,
                         energy_per_parcel_Wh=flown.E_Wh.fillna(0).sum() / ok if ok else np.nan,
                         rms_err_m=flown.rms_err.mean() if len(flown) else np.nan))
    return pd.DataFrame(rows)


def plot(s, x, col, ylabel, path):
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    exps = list(dict.fromkeys(s.experiment))
    for (e, p, c), g in s.groupby(["experiment", "planner", "controller"], sort=False):
        g = g.sort_values(x)
        lab = f"{PNAME.get(p, p)} + {c.upper()}" + (f"  [{e}]" if len(exps) > 1 else "")
        ax.plot(g[x], g[col], DASH.get(c, "-"), marker=MARK[exps.index(e) % len(MARK)], color=COLOR.get(p, "k"),
                lw=1.8, ms=5, label=lab)
    ax.set_xlabel({"U_ref": "reference wind speed U_ref (m/s)", "theta": "wind direction (deg, toward)"}.get(x, x))
    ax.set_ylabel(ylabel)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--x", default="U_ref", help="factor on the horizontal axis (U_ref, theta, customer, seed)")
    ap.add_argument("--out", help="output folder (default: folder of the first CSV)")
    a = ap.parse_args()
    d = pd.concat([load(p) for p in a.csv], ignore_index=True)
    if "error" in d and d.error.notna().any():
        print(f"WARNING: {d.error.notna().sum()} missions raised an error, e.g. {d.error.dropna().iloc[0]}")
    s = summarise(d, a.x)
    out = a.out or os.path.dirname(os.path.abspath(a.csv[0]))
    os.makedirs(out, exist_ok=True)
    s.to_csv(os.path.join(out, "summary.csv"), index=False, float_format="%.2f")
    pd.set_option("display.width", 200)
    pd.set_option("display.max_rows", 500)
    print(s.to_string(index=False, float_format=lambda v: f"{v:.1f}"))
    plot(s, a.x, "delivered_pct", "parcels delivered (%)", os.path.join(out, "success_rate.png"))
    plot(s, a.x, "energy_per_parcel_Wh", "energy per parcel delivered (Wh)", os.path.join(out, "energy_per_parcel.png"))
    print(f"\nwrote {out}/summary.csv, success_rate.png, energy_per_parcel.png")


if __name__ == "__main__":
    main()
