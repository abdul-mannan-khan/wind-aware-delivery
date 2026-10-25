"""Block diagram of the simulation software (Figure in Chapter 3)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
fig, ax = plt.subplots(figsize=(9.0, 3.8))
ax.set_xlim(0, 100)
ax.set_ylim(-4, 50)
ax.axis("off")


def box(x, y, w, h, title, sub, col):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2", fc=col + "22",
                                ec=col, lw=1.5))
    ax.text(x + w / 2, y + h * 0.66, title, ha="center", va="center", fontsize=8, weight="bold", color=INK,
            family="serif")
    ax.text(x + w / 2, y + h * 0.30, sub, ha="center", va="center", fontsize=6.3, color=INK2, family="serif")


def arrow(x0, y0, x1, y1, text=None, dy=1.5):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.2))
    if text:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + dy, text, ha="center", fontsize=6.5, color=INK2, family="serif")


box(1, 34, 17, 12, "City model", "196 buildings,\nstreets & avenues", BLUE)
box(27, 34, 17, 12, "Urban wind model", "profile, canopy, channelling,\nwakes (mean map)", BLUE)
box(53, 34, 17, 12, "Route planner", "A* on 4 m grid; layer,\nspeed & energy cost", ORANGE)
box(79, 34, 17, 12, "Trajectory", "corner rounding,\nspeed profile", ORANGE)
box(27, 8, 17, 12, "Gust model", "Dryden-type,\nTI-scheduled", BLUE)
box(53, 8, 17, 12, "6-DOF quadrotor", "drag, motors, payload\ndrop, power model", AQUA)
box(79, 8, 17, 12, "Controller", "geometric SE(3) + PID\n(+ DOB)", AQUA)
box(1, 8, 17, 12, "Metrics", "success, energy,\ntime, error", INK2)
arrow(18.8, 40, 26.2, 40)
arrow(44.8, 40, 52.2, 40, "mean wind", dy=1.2)
arrow(70.8, 40, 78.2, 40)
arrow(87.5, 33.2, 87.5, 20.8)
ax.text(88.5, 27, "p_d, v_d, a_d", fontsize=6.5, color=INK2, family="serif")
arrow(35.5, 33.2, 35.5, 20.8)
arrow(44.8, 14, 52.2, 14, "wind", dy=1.2)
arrow(78.2, 11.5, 70.8, 11.5)
ax.text(74.5, 8.6, "thrusts", ha="center", fontsize=6.5, color=INK2, family="serif")
ax.annotate("", xy=(78.2, 16.5), xytext=(70.8, 16.5), arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.2))
ax.text(74.5, 17.8, "state", ha="center", fontsize=6.5, color=INK2, family="serif")
ax.plot([61.5, 61.5, 9.5], [7.2, 1, 1], color=INK2, lw=1.2)
ax.annotate("", xy=(9.5, 7.2), xytext=(9.5, 0.9), arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.2))
ax.text(35.5, -2.6, "state, power, events (logged)", ha="center", fontsize=6.5, color=INK2, family="serif")
fig.savefig("../outputs/figures/sim_architecture.png", dpi=220, bbox_inches="tight")
fig.savefig("../outputs/figures/sim_architecture.pdf", bbox_inches="tight")
