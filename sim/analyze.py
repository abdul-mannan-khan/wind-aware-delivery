"""Figures, LaTeX tables and number macros from the campaign results."""
import os
import pickle

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

from city import City
from mission import CUSTOMERS, DEPOT, build_reference
from quadrotor import Params
from wind import WindField

FIG = "../outputs/figures"
TAB = "../outputs/tables"
RES = "../results"
os.makedirs(FIG, exist_ok=True)
os.makedirs(TAB, exist_ok=True)

# colour roles (validated categorical slots 1-3 of the reference palette)
C_SHORT, C_WIND, C_AUX = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#dedcd6"
plt.rcParams.update({
    "font.family": "serif", "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "legend.frameon": False, "lines.linewidth": 2.0,
    "savefig.dpi": 200, "savefig.bbox": "tight"})
LS = {"pid": "-", "dob": "--"}
COL = {"shortest": C_SHORT, "energy0": C_AUX, "wind": C_WIND}
PNAME = {"shortest": "Shortest @35 m", "energy0": "Energy (no wind)", "wind": "Wind-aware"}
CNAME = {"pid": "PID", "dob": "PID+DOB"}
NAME = {(p, c): f"{PNAME[p]} + {CNAME[c]}" for p in PNAME for c in CNAME}
PLANNERS = ["shortest", "energy0", "wind"]
SPEEDS_U = [4.0, 8.0, 12.0, 16.0]
WORD = {4.0: "Four", 8.0: "Eight", 12.0: "Twelve", 16.0: "Sixteen"}


def save(fig, name):
    fig.savefig(f"{FIG}/{name}.pdf")
    fig.savefig(f"{FIG}/{name}.png")
    plt.close(fig)


# --------------------------------------------------------------------------------- scene
def fig_city(city):
    xs, H = city.height_map(2.0)
    fig, ax = plt.subplots(figsize=(4.6, 4.0))
    im = ax.imshow(np.ma.masked_equal(H, 0).T, origin="lower", extent=[0, city.size] * 2, cmap="Greys",
                   vmin=0, vmax=95)
    cb = fig.colorbar(im, ax=ax, shrink=0.85)
    cb.set_label("building height (m)")
    for a, b, av in city.gaps:
        if av:
            ax.axvspan(a, b, color=C_AUX, alpha=0.12, lw=0)
            ax.axhspan(a, b, color=C_AUX, alpha=0.12, lw=0)
    ax.plot(*DEPOT, "s", ms=8, color=INK, mec="white")
    ax.annotate("Depot", DEPOT, xytext=(8, 6), textcoords="offset points", color=INK)
    for i, c in enumerate(CUSTOMERS):
        ax.plot(*c, "o", ms=8, color=C_WIND, mec="white")
        ax.annotate(f"C{i + 1}", c, xytext=(-18 if c[0] > 500 else 6, 6), textcoords="offset points", color=INK)
    ax.set_xlabel("x / east (m)")
    ax.set_ylabel("y / north (m)")
    ax.grid(False)
    save(fig, "city_map")


def fig_wind_slices(city):
    w = WindField(city, 8.0, 30.0)
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.7), sharey=True)
    for ax, z in zip(axs, [20, 35, 50]):
        U, TI = w.layer(z)
        s = np.hypot(U[..., 0], U[..., 1])
        im = ax.imshow(s.T, origin="lower", extent=[0, city.size] * 2, cmap="Blues", vmin=0, vmax=10)
        q = 6
        X, Y = np.meshgrid(w.xs[::q], w.xs[::q], indexing="ij")
        ax.quiver(X, Y, U[::q, ::q, 0], U[::q, ::q, 1], scale=160, width=0.004, color=INK2)
        ax.set_title(f"z = {z} m")
        ax.set_xlabel("x (m)")
        ax.grid(False)
    axs[0].set_ylabel("y (m)")
    cb = fig.colorbar(im, ax=axs, shrink=0.9, pad=0.02)
    cb.set_label("mean wind speed (m/s)")
    save(fig, "wind_slices")
    # turbulence intensity at 20 m
    fig, ax = plt.subplots(figsize=(3.6, 3.1))
    U, TI = w.layer(20)
    im = ax.imshow((TI * np.hypot(U[..., 0], U[..., 1])).T, origin="lower", extent=[0, city.size] * 2,
                   cmap="Blues")
    cb = fig.colorbar(im, ax=ax, shrink=0.85)
    cb.set_label(r"gust std. $\sigma_u$ (m/s)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(False)
    save(fig, "turbulence_20m")


def fig_profiles(city):
    w = WindField(city, 8.0, 0.0)
    pts = {"Corridor (avenue, aligned)": (300.0, 274.0), "Street (cross-wind)": (186.0, 300.0),
           "Inside block alley / wake": (330.0, 150.0)}
    z = np.linspace(1, 95, 120)
    fig, ax = plt.subplots(figsize=(3.6, 3.1))
    for (name, (x, y)), c, ls in zip(pts.items(), [C_WIND, C_SHORT, C_AUX], ["-", "--", ":"]):
        s = [np.linalg.norm(w.mean(np.array([x, y, zz]))[0]) for zz in z]
        ax.plot(s, z, color=c, ls=ls, label=name)
    zz = np.maximum(z, 1)
    ax.plot(8.0 * (zz / 60) ** 0.3, z, color=INK2, lw=1, label="Power law (open)")
    ax.set_xlabel("mean wind speed (m/s)")
    ax.set_ylabel("height z (m)")
    ax.legend(loc="upper left", fontsize=7)
    save(fig, "wind_profiles")


def fig_routes(city):
    xs, H = city.height_map(2.0)
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 4.0), sharey=True)
    for ax, th in zip(axs, [180.0, 0.0]):
        w = WindField(city, 8.0, th)
        ax.imshow(np.ma.masked_equal(H, 0).T, origin="lower", extent=[0, city.size] * 2, cmap="Greys",
                  vmin=0, vmax=95)
        for mode in ("shortest", "energy0", "wind"):
            try:
                _, pred = build_reference(city, w, CUSTOMERS[0], mode, Params())
            except Exception:
                continue
            po, pb = pred["path_out"], pred["path_back"]
            ax.plot(po[:, 0], po[:, 1], color=COL[mode], ls="-",
                    label=f"{PNAME[mode]} out ({pred['z_out']:.0f} m, {pred['v_out']:.0f} m/s)")
            ax.plot(pb[:, 0], pb[:, 1], color=COL[mode], ls=":",
                    label=f"{PNAME[mode]} back ({pred['z_back']:.0f} m, {pred['v_back']:.0f} m/s)")
        ax.annotate("", xy=(560 + 50 * np.cos(np.radians(th)), 70), xytext=(560, 70),
                    arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.5))
        ax.text(560, 82, "wind 8 m/s", ha="center", color=INK, fontsize=7)
        ax.set_title(f"Wind toward {th:.0f}°")
        ax.set_xlabel("x (m)")
        ax.grid(False)
        ax.legend(loc="upper left", fontsize=5, bbox_to_anchor=(0, -0.13), ncol=1)
    axs[0].set_ylabel("y (m)")
    save(fig, "routes_example")


# ---------------------------------------------------------------------------- time series
def fig_timeseries(logs):
    fig, axs = plt.subplots(4, 1, figsize=(7.0, 6.4), sharex=True)
    for ctrl in ("pid", "dob"):
        cols, L = logs[(12.0, 180.0, 0, "shortest", ctrl)]["log"]
        c = {k: i for i, k in enumerate(cols)}
        t = L[:, c["t"]]
        err = np.linalg.norm(L[:, [c["x"], c["y"], c["z"]]] - L[:, [c["xd"], c["yd"], c["zd"]]], axis=1)
        col = C_SHORT if ctrl == "pid" else C_WIND
        lab = "PID" if ctrl == "pid" else "PID + DOB"
        axs[2].plot(t, err, color=col, lw=1.2, label=lab)
        if ctrl == "dob":
            axs[0].plot(t, L[:, c["z"]], color=INK, lw=1.2)
            axs[1].plot(t, np.linalg.norm(L[:, [c["wx"], c["wy"], c["wz"]]], axis=1), color=C_AUX, lw=0.8,
                        label="total (mean + gust)")
            axs[1].plot(t, np.linalg.norm(L[:, [c["umx"], c["umy"], c["umz"]]], axis=1), color=INK, lw=1.2,
                        label="mean")
        axs[3].plot(t, L[:, c["P"]], color=col, lw=0.8, label=lab)
    axs[0].set_ylabel("altitude (m)")
    axs[1].set_ylabel("wind (m/s)")
    axs[2].set_ylabel("position error (m)")
    axs[3].set_ylabel("power (W)")
    axs[3].set_xlabel("time (s)")
    axs[1].legend(loc="upper right", fontsize=7, ncol=2)
    axs[2].legend(loc="upper right", fontsize=7, ncol=2)
    save(fig, "timeseries_mission")
    # observer estimate vs true aerodynamic force
    cols, L = logs[(12.0, 180.0, 0, "shortest", "dob")]["log"]
    c = {k: i for i, k in enumerate(cols)}
    t = L[:, c["t"]]
    fig, axs = plt.subplots(2, 1, figsize=(7.0, 3.4), sharex=True)
    for ax, a, n in zip(axs, ["x", "y"], ["$x$", "$y$"]):
        ax.plot(t, L[:, c["fa" + a]], color=INK2, lw=0.8, label="true aerodynamic force")
        ax.plot(t, L[:, c["dh" + a]], color=C_WIND, lw=1.4, label=r"DOB estimate $\hat d$")
        ax.set_ylabel(f"force {n} (N)")
    axs[0].legend(loc="upper right", fontsize=7, ncol=2)
    axs[1].set_xlabel("time (s)")
    save(fig, "dob_estimate")


# ------------------------------------------------------------------------------- campaign
def ci95(x):
    x = np.asarray(x, float)
    return 1.96 * x.std(ddof=1) / np.sqrt(len(x)) if len(x) > 1 else 0.0




def outcome(df):
    o = np.where(df.rejected == True, "refused", np.where(df.collided == True, "collision",  # noqa: E712
                 np.where(df.success == True, "completed", "failed")))  # noqa: E712
    return pd.Series(o, index=df.index)


def paired(df, a, b, key=("U_ref", "theta", "customer", "seed", "controller")):
    """Join planner a and planner b on identical conditions; both must have completed."""
    key = list(key)
    A = df[(df.planner == a) & (df.success == True)].set_index(key)  # noqa: E712
    B = df[(df.planner == b) & (df.success == True)].set_index(key)  # noqa: E712
    j = A.join(B, lsuffix="_a", rsuffix="_b", how="inner").reset_index()
    j["save_pct"] = 100 * (j.E_Wh_a - j.E_Wh_b) / j.E_Wh_a
    j["dt_pct"] = 100 * (j.t_mission_b - j.t_mission_a) / j.t_mission_a
    return j


def campaign(df, macros):
    df = df.copy()
    df["rejected"] = df.rejected.fillna(False).astype(bool)
    df["collided"] = df.collided.fillna(False).astype(bool)
    df["outcome"] = outcome(df)
    ok = df[df.success == True]  # noqa: E712

    # ---- outcome rates per planner and wind speed (both controllers pooled)
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.6), sharey=True)
    cats = [("completed", C_AUX), ("refused", "#a6c8f0"), ("failed", C_WIND), ("collision", "#e34948")]
    for ax, pl in zip(axs, PLANNERS):
        g = df[df.planner == pl]
        tab = pd.crosstab(g.U_ref, g.outcome, normalize="index") * 100
        bottom = np.zeros(len(tab))
        for cat, col in cats:
            v = tab[cat].values if cat in tab else np.zeros(len(tab))
            ax.bar(np.arange(len(tab)), v, 0.65, bottom=bottom, color=col, edgecolor="white", lw=1.5,
                   label=cat)
            bottom += v
        ax.set_xticks(range(len(tab)))
        ax.set_xticklabels([f"{u:.0f}" for u in tab.index])
        ax.set_title(PNAME[pl])
        ax.set_xlabel("$U_{ref}$ (m/s)")
        ax.grid(axis="x")
    axs[0].set_ylabel("share of missions (%)")
    axs[2].legend(fontsize=7, loc="lower left")
    save(fig, "outcomes")

    # ---- energy and tracking error vs wind speed (completed missions)
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.9))
    for (pl, ct), g in ok.groupby(["planner", "controller"]):
        m = g.groupby("U_ref").E_Wh.agg(["mean", ci95])
        axs[0].errorbar(m.index, m["mean"], yerr=m["ci95"], color=COL[pl], ls=LS[ct], marker="o", ms=4,
                        capsize=3, label=NAME[(pl, ct)])
        r = g.groupby("U_ref").rms_err.agg(["mean", ci95])
        axs[1].errorbar(r.index, r["mean"], yerr=r["ci95"], color=COL[pl], ls=LS[ct], marker="o", ms=4,
                        capsize=3, label=NAME[(pl, ct)])
    for a in axs:
        a.set_xlabel("reference wind speed $U_{ref}$ (m/s)")
        a.set_xticks(SPEEDS_U)
    axs[0].set_ylabel("mission energy (Wh)")
    axs[1].set_ylabel("RMS position error (m)")
    axs[0].legend(fontsize=6)
    save(fig, "energy_error_vs_speed")

    # ---- paired savings
    j_ws = paired(df, "shortest", "wind")
    j_es = paired(df, "shortest", "energy0")
    j_we = paired(df, "energy0", "wind")
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.8), sharey=True)
    dirs = sorted(df.theta.unique())
    blues = ["#cfe0f6", "#a6c8f0", "#5b9be3", "#1c5aa8"]
    for ax, j, title in ((axs[0], j_ws, "Wind-aware vs. shortest @35 m"),
                         (axs[1], j_we, "Wind-aware vs. energy planner without wind")):
        width = 0.2
        for k, (U, col) in enumerate(zip(SPEEDS_U, blues)):
            g = j[j.U_ref == U].groupby("theta").save_pct.agg(["mean", ci95]).reindex(dirs)
            ax.bar(np.arange(len(dirs)) + (k - 1.5) * width, g["mean"], width - 0.02, yerr=g["ci95"], color=col,
                   capsize=1.5, label=f"{U:.0f} m/s", error_kw=dict(ecolor=INK2, lw=0.7))
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_xticks(range(len(dirs)))
        ax.set_xticklabels([f"{d:.0f}°" for d in dirs], fontsize=7)
        ax.set_xlabel(r"wind direction $\theta_w$")
        ax.set_title(title)
    axs[0].set_ylabel("energy saving (%)")
    axs[0].legend(fontsize=7, ncol=2, title="$U_{ref}$", title_fontsize=7)
    save(fig, "saving_by_direction")

    # ---- what the wind-aware planner chose
    wa = df[(df.planner == "wind") & (df.controller == "dob") & (df.rejected == False)]  # noqa: E712
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.6))
    for ax, var, vals, cols, name in (
            (axs[0], "z", [20.0, 35.0, 50.0], [C_SHORT, C_AUX, C_WIND], "cruise layer (m)"),
            (axs[1], "v", [6.0, 8.0, 10.0, 12.0], blues, "cruise speed (m/s)")):
        both = pd.concat([wa[["U_ref", f"{var}_out"]].rename(columns={f"{var}_out": "x"}),
                          wa[["U_ref", f"{var}_back"]].rename(columns={f"{var}_back": "x"})], ignore_index=True)
        tab = pd.crosstab(both.U_ref, both.x, normalize="index") * 100
        bottom = np.zeros(len(tab))
        for val, col in zip(vals, cols):
            v = tab[val].values if val in tab else np.zeros(len(tab))
            ax.bar(np.arange(len(tab)), v, 0.6, bottom=bottom, color=col, edgecolor="white", lw=1.5,
                   label=f"{val:.0f}")
            bottom += v
        ax.set_xticks(range(len(tab)))
        ax.set_xticklabels([f"{u:.0f}" for u in tab.index])
        ax.set_xlabel("$U_{ref}$ (m/s)")
        ax.set_ylabel("share of legs (%)")
        ax.legend(title=name, fontsize=7, title_fontsize=7, loc="upper left", bbox_to_anchor=(1.0, 1.0))
        ax.grid(axis="x")
    fig.tight_layout()
    save(fig, "planner_choices")

    # ---- predicted vs simulated energy
    fig, ax = plt.subplots(figsize=(3.4, 3.1))
    g0 = ok[ok.controller == "dob"]
    for pl in PLANNERS:
        g = g0[g0.planner == pl]
        ax.scatter(g.E_pred_Wh, g.E_Wh, s=8, color=COL[pl], alpha=0.6, edgecolors="none", label=PNAME[pl])
    lim = [g0.E_pred_Wh.min() * 0.95, g0.E_Wh.max() * 1.05]
    ax.plot(lim, lim, color=INK2, lw=0.8)
    ax.set_xlabel("planner-predicted energy (Wh)")
    ax.set_ylabel("simulated energy (Wh)")
    ax.legend(fontsize=7)
    save(fig, "pred_vs_sim")
    rp = stats.pearsonr(ok.E_pred_Wh, ok.E_Wh)[0]
    bias = 100 * ((ok.E_Wh - ok.E_pred_Wh) / ok.E_pred_Wh).mean()

    # ---- tables
    rows = []
    for U in SPEEDS_U:
        for pl in PLANNERS:
            for ct in ("pid", "dob"):
                g = df[(df.U_ref == U) & (df.planner == pl) & (df.controller == ct)]
                c = g[g.success == True]  # noqa: E712
                f = lambda x, d=2: f"{x:.{d}f}" if np.isfinite(x) else "--"
                rows.append([f"{U:.0f}", NAME[(pl, ct)], f"{100 * g.success.mean():.0f}",
                             f"{100 * g.rejected.mean():.0f}",
                             f"{100 * ((g.success == False) & (g.rejected == False)).mean():.0f}",  # noqa: E712
                             f(c.E_Wh.mean()), f(c.t_mission.mean(), 0), f(c.rms_err.mean()), f(c.max_err.mean()),
                             f(g.min_clear.min(), 1)])
    with open(f"{TAB}/summary.tex", "w") as fh:
        fh.write("\\begin{tabular}{llrrrrrrrr}\n\\toprule\n")
        fh.write("$U_{ref}$ & Configuration & Done & Refused & Failed & Energy & Time & RMS err. & Max err. & Min clr. \\\\\n")
        fh.write("(m/s) & & (\\%) & (\\%) & (\\%) & (Wh) & (s) & (m) & (m) & (m) \\\\\n\\midrule\n")
        for i, r in enumerate(rows):
            fh.write(" & ".join(r) + " \\\\\n")
            if i % 6 == 5 and i < len(rows) - 1:
                fh.write("\\midrule\n")
        fh.write("\\bottomrule\n\\end{tabular}\n")
    pd.DataFrame(rows, columns=["U_ref", "config", "done_pct", "refused_pct", "failed_pct", "E_Wh", "t_s",
                                "rms_m", "max_m", "min_clear_m"]).to_csv(f"{RES}/summary_table.csv", index=False)

    srows = []
    for name, j in (("Wind-aware vs. shortest", j_ws), ("Energy (no wind) vs. shortest", j_es),
                    ("Wind-aware vs. energy (no wind)", j_we)):
        for U in SPEEDS_U:
            g = j[j.U_ref == U]
            if len(g) < 2:
                srows.append([name, f"{U:.0f}", f"{len(g)}", "--", "--", "--", "--"])
                continue
            p = stats.wilcoxon(g.E_Wh_a, g.E_Wh_b).pvalue if (g.E_Wh_a != g.E_Wh_b).any() else 1.0
            srows.append([name, f"{U:.0f}", f"{len(g)}", f"{g.save_pct.mean():.1f} $\\pm$ {ci95(g.save_pct):.1f}",
                          f"{g.save_pct.max():.1f}", f"{g.dt_pct.mean():+.1f}", f"{p:.1e}"])
    with open(f"{TAB}/savings.tex", "w") as fh:
        fh.write("\\begin{tabular}{lrrrrrr}\n\\toprule\n")
        fh.write("Comparison & $U_{ref}$ & Pairs & Mean saving (\\%) & Max (\\%) & Time change (\\%) & Wilcoxon $p$ \\\\\n\\midrule\n")
        for i, r in enumerate(srows):
            fh.write(" & ".join(r) + " \\\\\n")
            if i % 4 == 3 and i < len(srows) - 1:
                fh.write("\\midrule\n")
        fh.write("\\bottomrule\n\\end{tabular}\n")
    pd.DataFrame(srows, columns=["comparison", "U_ref", "pairs", "mean_saving", "max_saving", "time_change",
                                 "wilcoxon_p"]).to_csv(f"{RES}/savings_table.csv", index=False)

    keyc = ["U_ref", "theta", "customer", "seed", "planner"]
    both = df[df.success == True]  # noqa: E712
    pc = both[both.controller == "pid"].set_index(keyc).join(both[both.controller == "dob"].set_index(keyc),
                                                             lsuffix="_p", rsuffix="_d", how="inner").reset_index()
    pc["rms_red"] = 100 * (pc.rms_err_p - pc.rms_err_d) / pc.rms_err_p
    pc["max_red"] = 100 * (pc.max_err_p - pc.max_err_d) / pc.max_err_p

    # ---- macros (numbers quoted in the text)
    macros["NMissions"] = f"{len(df)}"
    macros["NCollisions"] = f"{int(df.collided.sum())}"
    for pl, tag in (("shortest", "Short"), ("energy0", "EnergyZero"), ("wind", "Wind")):
        for U in SPEEDS_U:
            g = df[(df.planner == pl) & (df.U_ref == U)]
            macros[f"Done{tag}{WORD[U]}"] = f"{100 * g.success.mean():.0f}"
            macros[f"Fail{tag}{WORD[U]}"] = f"{100 * ((g.success == False) & (g.rejected == False)).mean():.0f}"  # noqa: E712
            macros[f"Refused{tag}{WORD[U]}"] = f"{100 * g.rejected.mean():.0f}"
    for name, j in (("WS", j_ws), ("ES", j_es), ("WE", j_we)):
        for U in SPEEDS_U:
            g = j[j.U_ref == U]
            macros[f"Save{name}{WORD[U]}"] = f"{g.save_pct.mean():.1f}" if len(g) else "--"
            macros[f"SaveMax{name}{WORD[U]}"] = f"{g.save_pct.max():.1f}" if len(g) else "--"
        macros[f"Save{name}All"] = f"{j.save_pct.mean():.1f}"
        macros[f"Time{name}All"] = f"{j.dt_pct.mean():+.1f}"
    for U in SPEEDS_U:
        c = pc[pc.U_ref == U]
        macros[f"RmsRed{WORD[U]}"] = f"{c.rms_red.mean():.0f}"
        macros[f"MaxRed{WORD[U]}"] = f"{c.max_red.mean():.0f}"
        for ct in ("pid", "dob"):
            g = ok[(ok.U_ref == U) & (ok.controller == ct)]
            macros[f"Rms{ct.capitalize()}{WORD[U]}"] = f"{g.rms_err.mean():.2f}"
            macros[f"Max{ct.capitalize()}{WORD[U]}"] = f"{g.max_err.mean():.2f}"
    macros["EnergyCtrlDiff"] = f"{100 * ((pc.E_Wh_d - pc.E_Wh_p) / pc.E_Wh_p).mean():+.1f}"
    macros["PredCorr"] = f"{rp:.3f}"
    macros["PredBias"] = f"{bias:.1f}"
    macros["EMax"] = f"{ok.E_Wh.max():.1f}"
    macros["BatteryUse"] = f"{100 * ok.E_Wh.max() / Params.battery_Wh:.0f}"
    macros["MinClear"] = f"{ok.min_clear.min():.1f}"
    fin = df[(df.success == False) & (df.rejected == False)]  # noqa: E712
    macros["NFailed"] = f"{len(fin)}"
    j_ws.to_csv(f"{RES}/paired_wind_vs_shortest.csv", index=False)
    j_we.to_csv(f"{RES}/paired_wind_vs_energy0.csv", index=False)
    pc.to_csv(f"{RES}/paired_controller.csv", index=False)


def fig_airsim():
    import glob
    logs = sorted(glob.glob(f"{RES}/airsim/log_U12_*.npy"))
    if not logs:
        return
    frames = sorted(glob.glob(f"{RES}/airsim/frames_*/*.png"))
    fig = plt.figure(figsize=(7.2, 2.9))
    ax = fig.add_axes([0.07, 0.17, 0.5 if frames else 0.9, 0.75])
    for f in logs:
        tag = os.path.basename(f)[4:-4]
        L = np.load(f)
        err = np.linalg.norm(L[:, 1:4] - L[:, 4:7], axis=1)
        pl = "wind" if tag.endswith("wind") else "shortest"
        th = tag.split("_")[1][2:]
        ax.plot(L[:, 0], err, color=COL[pl], ls="-" if th == "0" else ":", lw=1.2,
                label=f"{PNAME[pl]}, wind toward {th}°")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("tracking error in AirSim (m)")
    ax.legend(fontsize=6.5)
    if frames:
        img = plt.imread(frames[min(80, len(frames) - 1)])
        a2 = fig.add_axes([0.6, 0.12, 0.39, 0.82])
        a2.imshow(img)
        a2.axis("off")
        a2.set_title("AirSim chase camera (wind-aware, 12 m/s)", fontsize=7)
    save(fig, "airsim_check")


def write_macros(macros):
    with open(f"{TAB}/macros.tex", "w") as f:
        for k, v in macros.items():
            f.write(f"\\newcommand{{\\{k}}}{{{v}}}\n")


if __name__ == "__main__":
    city = City()
    macros = {}
    fig_city(city)
    fig_wind_slices(city)
    fig_profiles(city)
    fig_routes(city)
    if os.path.exists(f"{RES}/example_logs.pkl"):
        with open(f"{RES}/example_logs.pkl", "rb") as f:
            fig_timeseries(pickle.load(f))
    if os.path.exists(f"{RES}/campaign.csv"):
        df = pd.read_csv(f"{RES}/campaign.csv")
        campaign(df, macros)
    if os.path.exists(f"{RES}/airsim/airsim_summary.csv"):
        a = pd.read_csv(f"{RES}/airsim/airsim_summary.csv")
        a.to_csv(f"{RES}/airsim_table.csv", index=False)
        fig_airsim()
    write_macros(macros)
    print(macros)
