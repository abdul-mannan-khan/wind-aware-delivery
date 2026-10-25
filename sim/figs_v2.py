"""Extra figures for the version 1 analysis (tilt envelope, failure modes,
outcome by wind direction, energy per delivered parcel)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np
import pandas as pd

from analyze import C_AUX, C_SHORT, C_WIND, COL, GRID, INK, INK2, PLANNERS, PNAME, RES, save
from quadrotor import Params, trim_power


def load():
    d = pd.read_csv(f"{RES}/campaign.csv")
    for c in ["rejected", "collided", "tracking_fail", "success"]:
        d[c] = d[c].fillna(False).astype(bool)
    mode = np.full(len(d), "completed", dtype=object)
    mode[(~d.success) & (d.max_err >= 10)] = "breach"
    mode[d.tracking_fail.values] = "lost"
    mode[d.collided.values] = "collision"
    mode[d.rejected.values] = "refused"
    d["mode"] = mode
    return d


def fig_tilt_envelope():
    P, m = Params(), 2.2
    U = np.linspace(0, 18, 181)
    fig, ax = plt.subplots(figsize=(4.4, 3.0))
    shades = {6: "#9cc3f0", 8: "#5f9ee6", 10: "#2a78d6", 12: "#174a8c"}
    for v, c in shades.items():
        t = [trim_power(np.array([v, 0, 0.0]), np.array([-u, 0, 0.0]), m, P, return_tilt=True)[1] for u in U]
        ax.plot(U, t, color=c, label=f"{v} m/s, 2.2 kg")
    for v in (6, 8):
        t = [trim_power(np.array([v, 0, 0.0]), np.array([-u, 0, 0.0]), 1.6, P, return_tilt=True)[1] for u in U]
        ax.plot(U, t, color=shades[v], ls="--", lw=1.6, label=f"{v} m/s, 1.6 kg (no parcel)")
    ax.axhline(35, color=C_WIND, lw=1.2, ls="--")
    ax.axhline(45, color=INK, lw=1.2, ls=":")
    ax.text(0.3, 36, "planner limit 35°", color=C_WIND, fontsize=7.5)
    ax.text(0.3, 46, "controller limit 45°", color=INK, fontsize=7.5)
    ax.set_xlabel("headwind along the leg (m/s)")
    ax.set_ylabel("steady tilt required (deg)")
    ax.set_ylim(0, 60)
    ax.legend(title="ground speed, mass", fontsize=6.5, title_fontsize=7, loc="lower right")
    save(fig, "tilt_envelope")


def fig_failure_modes(d):
    cats = [("completed", C_AUX, "completed"), ("refused", "#a6c8f0", "refused before take-off"),
            ("breach", "#f2b28f", "corridor breach (10-25 m)"), ("lost", C_WIND, "lost track (>25 m)"),
            ("collision", "#b3261e", "collision")]
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.7), sharey=True)
    for ax, pl in zip(axs, PLANNERS):
        g = d[d.planner == pl]
        tab = pd.crosstab(g.U_ref, g["mode"], normalize="index") * 100
        bottom = np.zeros(len(tab))
        for cat, col, lab in cats:
            v = tab[cat].values if cat in tab else np.zeros(len(tab))
            ax.bar(np.arange(len(tab)), v, 0.65, bottom=bottom, color=col, edgecolor="white", lw=1.5, label=lab)
            bottom += v
        ax.set_xticks(range(len(tab)))
        ax.set_xticklabels([f"{u:.0f}" for u in tab.index])
        ax.set_title(PNAME[pl])
        ax.set_xlabel("$U_{ref}$ (m/s)")
        ax.grid(axis="x")
    axs[0].set_ylabel("share of missions (%)")
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=5, fontsize=7, bbox_to_anchor=(0.5, -0.06))
    fig.subplots_adjust(bottom=0.27)
    save(fig, "failure_modes")


def fig_outcome_direction(d):
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.7), sharey=True)
    for ax, U in zip(axs, [12.0, 16.0]):
        for pl in PLANNERS:
            g = d[(d.planner == pl) & (d.U_ref == U)].groupby("theta").success.mean() * 100
            ax.plot(g.index, g.values, "o-", color=COL[pl], ms=4, label=PNAME[pl])
        ax.set_xticks(range(0, 360, 45))
        ax.set_xlabel("wind direction, blowing toward (deg)")
        ax.set_title(f"$U_{{ref}}$ = {U:.0f} m/s")
        ax.set_ylim(-5, 105)
    axs[0].set_ylabel("missions completed (%)")
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=3, fontsize=7, bbox_to_anchor=(0.5, -0.04))
    fig.subplots_adjust(bottom=0.3)
    save(fig, "outcome_by_direction")


def fig_energy_per_parcel(d):
    f = d[~d.rejected]
    fig, ax = plt.subplots(figsize=(4.4, 3.0))
    for pl in PLANNERS:
        g = f[f.planner == pl].groupby("U_ref").apply(lambda x: x.E_Wh.sum() / max(x.success.sum(), 1))
        ax.plot(g.index, g.values, "o-", color=COL[pl], ms=4, label=PNAME[pl])
    ax.set_yscale("log")
    ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    ax.set_yticks([10, 20, 50, 100])
    ax.set_yticklabels(["10", "20", "50", "100"])
    ax.set_xticks([4, 8, 12, 16])
    ax.set_xlabel("$U_{ref}$ (m/s)")
    ax.set_ylabel("energy per parcel delivered (Wh)")
    ax.legend(fontsize=7)
    save(fig, "energy_per_parcel")


if __name__ == "__main__":
    d = load()
    fig_tilt_envelope()
    fig_failure_modes(d)
    fig_outcome_direction(d)
    fig_energy_per_parcel(d)
    print(d.groupby(["planner", "U_ref"])["mode"].value_counts().unstack(fill_value=0))
