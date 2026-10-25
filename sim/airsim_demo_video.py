"""Explained, slowed-down video of the AirSim replay (C1, 12 m/s wind toward east, wind-aware plan).

Combines the chase-camera frames (one every 2 s of mission time) with a live route map and charts drawn from
the 20 Hz AirSim flight log, phase captions, and title / explanation / result cards.
Writes results/airsim/demo_airsim_explained.mp4 (1920x1080, 30 fps)."""
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from city import City
from mission import CUSTOMERS, DEPOT

ROOT = Path(__file__).resolve().parents[1]
AS = ROOT / "results" / "airsim"
OUT = AS / "demo_airsim_explained.mp4"
W, H, FPS = 1920, 1080, 30
NAVY, NAVY2, WHITE, ORANGE, GREY = (20, 33, 61), (38, 58, 98), (255, 255, 255), (235, 104, 52), (90, 90, 90)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONTI = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Oblique.ttf"


def font(size, kind=""):
    return ImageFont.truetype({"": FONT, "b": FONTB, "i": FONTI}[kind], size)


# --------------------------------------------------------------------------- data
DEMO = AS / "demo"                                              # re-recorded inside the map (airsim_demo_record.py)
L = np.load(DEMO / "log.npy")
T, P, PD, WIND = L[:, 0], L[:, 1:4], L[:, 4:7], L[:, 10:13]
ERR = np.linalg.norm(P - PD, axis=1)
WSPD = np.linalg.norm(WIND[:, :2], axis=1)
T_END = T[-1]
FRAME_DT = 0.25                                                  # camera frame k recorded at t = k * FRAME_DT
FRAMES = sorted((DEMO / "frames").glob("*.png"))
CAM = {int(f.stem): f for f in FRAMES}
SKY = (-1.0, -1.0)                                               # whole route is now over the map's terrain

# phases: (t0, t1, playback speed, title, explanation)
PHASES = [
    (0.0, 3.0, 2.0, "Ready at the depot",
     "The drone hovers above the depot. The wind is 12 m/s, blowing toward the east. (AirSim's drone flies the "
     "plan but carries no physical parcel.)"),
    (3.0, 12.8, 2.5, "Climb to the 20 m cruise layer",
     "The wind-aware planner chose to fly low, between the buildings, where the wind is weaker."),
    (12.8, 124.1, 5.0, "Outbound leg: tailwind",
     "The route follows sheltered streets to customer C1 in the east. The wind pushes the drone from behind."),
    (124.1, 133.2, 2.5, "Descend to 6 m at the customer",
     "The drone slows down and descends above the delivery point."),
    (133.2, 141.1, 2.0, "Parcel drop point",
     "The plan holds an 8-second hover here, where the delivery drone releases its 0.6 kg parcel."),
    (141.1, 149.3, 2.5, "Climb back to 20 m", "The drone climbs back to its cruise layer for the return."),
    (149.3, 320.6, 5.0, "Return leg: headwind",
     "Flying into the wind is the hard part. Before take-off the planner checked that this leg is flyable "
     "for the delivery drone, which is lighter on the way back and so more limited by a headwind."),
    (320.6, 331.7, 2.5, "Descend at the depot", "The drone descends to land where it started."),
    (331.7, T_END + 0.01, 2.0, "Mission completed",
     "The drone is back at the depot after flying the whole route in the injected urban wind."),
]


def phase_at(t):
    for ph in PHASES:
        if ph[0] <= t < ph[1]:
            return ph
    return PHASES[-1]


def mission_times():
    """Mission time for every output frame, with the playback speed of each phase."""
    ts, t = [], 0.0
    while t <= T_END:
        ts.append(t)
        t += phase_at(t)[2] / FPS
    ts += [T_END] * int(1.5 * FPS)      # short hold on the final frame
    return ts


# --------------------------------------------------------------------------- map + chart (matplotlib)
city = City()
xs, Hmap = city.height_map(2.0)
fig = plt.figure(figsize=(6.9, 9.2), dpi=100)
axm = fig.add_axes([0.02, 0.33, 0.96, 0.62])
axc = fig.add_axes([0.11, 0.05, 0.86, 0.22])
fig.patch.set_facecolor("white")
axm.imshow(np.ma.masked_equal(Hmap, 0).T, origin="lower", extent=[0, city.size] * 2, cmap="Greys", vmin=-20,
           vmax=110)
for a, b, av in city.gaps:
    if av:
        axm.axvspan(a, b, color="#1baf7a", alpha=0.10, lw=0)
        axm.axhspan(a, b, color="#1baf7a", alpha=0.10, lw=0)
axm.plot(PD[:, 0], PD[:, 1], color="#2a78d6", lw=1.6, ls="--", label="planned route")
trail, = axm.plot([], [], color="#eb6834", lw=2.6, label="flown (AirSim)")
dot, = axm.plot([], [], "o", ms=11, color="#eb6834", mec="white", mew=2)
axm.plot(*DEPOT, "s", ms=12, color="#14213d", mec="white", mew=1.5)
axm.text(DEPOT[0] + 14, DEPOT[1] + 10, "Depot", fontsize=11, fontweight="bold", color="#14213d")
cx, cy = CUSTOMERS[0]
axm.plot(cx, cy, "*", ms=20, color="gold", mec="#14213d")
axm.text(cx - 18, cy + 18, "Customer C1", fontsize=11, fontweight="bold", color="#14213d", ha="right")
axm.annotate("", xy=(250, 590), xytext=(130, 590), arrowprops=dict(arrowstyle="-|>", lw=3, color="#14213d"))
axm.text(190, 602, "wind 12 m/s", ha="center", fontsize=11, fontweight="bold", color="#14213d")
axm.set_xlim(0, city.size)
axm.set_ylim(0, city.size)
axm.set_xticks([])
axm.set_yticks([])
axm.set_title("Route on the city map (top view)", fontsize=13, fontweight="bold", color="#14213d", loc="left")
axm.legend(loc="lower right", fontsize=10, framealpha=0.9)
l_err, = axc.plot([], [], color="#eb6834", lw=1.4, label="tracking error (m)")
l_wind, = axc.plot([], [], color="#2a78d6", lw=1.0, alpha=0.7, label="wind at drone (m/s) ÷ 4")
cursor = axc.axvline(0, color="#14213d", lw=1.5)
axc.set_xlim(0, T_END)
axc.set_ylim(0, 7.0)
axc.set_xlabel("mission time (s)", fontsize=10)
axc.set_ylabel("metres", fontsize=10)
axc.legend(loc="upper left", fontsize=9, ncol=2, frameon=False)
axc.grid(alpha=0.3)
for s in ("top", "right"):
    axc.spines[s].set_visible(False)


def panel(t):
    k = np.searchsorted(T, t)
    k = min(k, len(T) - 1)
    trail.set_data(P[:k + 1, 0], P[:k + 1, 1])
    dot.set_data([P[k, 0]], [P[k, 1]])
    cursor.set_xdata([t, t])
    l_err.set_data(T[:k + 1], ERR[:k + 1])
    l_wind.set_data(T[:k + 1], WSPD[:k + 1] / 4)
    fig.canvas.draw()
    return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]), k


# --------------------------------------------------------------------------- camera
_cache = {}


def cam_img(k):
    if k not in _cache:
        if len(_cache) > 6:
            _cache.clear()
        _cache[k] = Image.open(CAM[k]).convert("RGB").resize((1152, 648), Image.LANCZOS)
    return _cache[k]


def camera(t):
    """Cross-fade between the two recorded frames around time t."""
    k0 = int(np.clip(np.floor(t / FRAME_DT), min(CAM), max(CAM)))
    k1 = min(k0 + 1, max(CAM))
    a = float(np.clip(t / FRAME_DT - k0, 0, 1)) if k1 != k0 else 0.0
    return Image.blend(cam_img(k0), cam_img(k1), a)


# --------------------------------------------------------------------------- drawing helpers
def wrap(draw, text, fnt, width):
    words, lines, cur = text.split(), [], ""
    for wd in words:
        test = (cur + " " + wd).strip()
        if draw.textlength(test, font=fnt) <= width:
            cur = test
        else:
            lines.append(cur)
            cur = wd
    lines.append(cur)
    return lines


def card(title, lines, subtitle=None, footer=None):
    img = Image.new("RGB", (W, H), NAVY)
    d = ImageDraw.Draw(img)
    d.rectangle([0, H - 14, W, H], fill=ORANGE)
    y = 190 if subtitle else 230
    d.text((150, y), title, font=font(64, "b"), fill=WHITE)
    y += 100
    if subtitle:
        d.text((150, y), subtitle, font=font(32, "i"), fill=(190, 205, 230))
        y += 90
    for ln in lines:
        bullet = ln.startswith("• ")
        txt = ln[2:] if bullet else ln
        wl = wrap(d, txt, font(34), W - 400)
        for i, part in enumerate(wl):
            if bullet and i == 0:
                d.ellipse([155, y + 14, 169, y + 28], fill=ORANGE)
            d.text((195 if bullet else 150, y), part, font=font(34), fill=WHITE)
            y += 50
        y += 18
    if footer:
        d.text((150, H - 90), footer, font=font(24), fill=(160, 175, 205))
    return img


def frame(t):
    img = Image.new("RGB", (W, H), (244, 246, 250))
    d = ImageDraw.Draw(img)
    # top bar
    d.rectangle([0, 0, W, 86], fill=NAVY)
    d.text((40, 22), "AirSim replay: wind-aware delivery in a 12 m/s urban wind", font=font(34, "b"), fill=WHITE)
    ph = phase_at(t)
    d.text((W - 40, 28), f"mission time {t:5.0f} s   |   playback {ph[2]:g}× real time", font=font(26),
           fill=(200, 212, 235), anchor="ra")
    # camera
    img.paste(camera(t), (40, 110))
    d.rectangle([40, 110, 40 + 1152, 110 + 648], outline=NAVY2, width=2)
    d.rectangle([40, 110, 420, 152], fill=(20, 33, 61))
    d.text((54, 117), "AirSim chase camera", font=font(24, "b"), fill=WHITE)
    if SKY[0] <= t <= SKY[1]:
        msg = ("The drone is beyond the edge of AirSim's scenery here, so the camera sees only sky. "
               "The flight physics and the injected wind are not affected.")
        lines = wrap(d, msg, font(25, "i"), 980)
        bh = 24 + 36 * len(lines)
        d.rounded_rectangle([86, 740 - bh, 86 + 1060, 740], radius=10, fill=(255, 255, 255))
        for i, ln in enumerate(lines):
            d.text((106, 740 - bh + 12 + 36 * i), ln, font=font(25, "i"), fill=GREY)
    # map + chart
    pim, k = panel(t)
    img.paste(pim.resize((690, 920), Image.LANCZOS), (1210, 100))
    # caption box
    d.rounded_rectangle([40, 780, 1192, 1040], radius=14, fill=WHITE, outline=(210, 216, 228), width=2)
    d.rectangle([40, 780, 52, 1040], fill=ORANGE)
    d.text((80, 800), ph[3], font=font(44, "b"), fill=NAVY)
    for i, ln in enumerate(wrap(d, ph[4], font(30), 1080)):
        d.text((80, 870 + 44 * i), ln, font=font(30), fill=(40, 40, 40))
    # live numbers
    z = P[k, 2]
    d.text((1210, 1022), f"altitude {z:4.1f} m    wind at drone {WSPD[k]:4.1f} m/s    error {ERR[k]:3.1f} m",
           font=font(21, "b"), fill=NAVY)
    return img


def fade(img, a):
    return Image.blend(Image.new("RGB", (W, H), NAVY), img, a)


# --------------------------------------------------------------------------- write
def main():
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                           f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "20",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(OUT)], stdin=subprocess.PIPE)

    def put(img, n=1):
        b = img.tobytes()
        for _ in range(n):
            ff.stdin.write(b)

    def show_card(img, secs):
        n, nf = int(secs * FPS), int(0.6 * FPS)
        for i in range(n):
            a = min(1.0, (i + 1) / nf, (n - i) / nf)
            put(fade(img, a))

    show_card(card("Independent check in AirSim",
                   ["• A delivery mission planned by the wind-aware planner is flown by a different simulator, "
                    "AirSim, with its own drone model and flight controller.",
                    "• Question: can an independent vehicle fly the planned route in the urban wind of this "
                    "study?"],
                   subtitle="Quadrotor Package Delivery through Urban Wind Corridors",
                   footer="University of West London  ·  School of Computing and Engineering"), 9)
    show_card(card("What you will see",
                   ["• The mission: depot → customer C1 (parcel drop) → back to the depot, with a 12 m/s wind "
                    "blowing toward the east.",
                    "• Every 0.05 s, the urban wind field of the study (mean wind plus gusts) is applied to the "
                    "AirSim drone at its current position.",
                    "• Left: AirSim's chase camera. Right: the route on the city map and a chart of tracking error "
                    "and wind. Bottom: what is happening now.",
                    "• AirSim's scenery is its own map, not the city of the study: only the wind and the route are "
                    "transferred. The video runs 2 to 5 times faster than real time."]), 15)
    ts = mission_times()
    first = frame(0.0)
    for i in range(int(0.6 * FPS)):
        put(fade(first, (i + 1) / (0.6 * FPS)))
    put(first, int(1.0 * FPS))
    for t in ts:
        put(frame(t))
    rms, mx = float(np.sqrt((ERR ** 2).mean())), float(ERR.max())
    show_card(card("Result",
                   [f"• Mission completed: the whole route, out and back, flown in {T_END:.0f} s.",
                    f"• The AirSim drone followed the planned route with an average (RMS) error of {rms:.1f} m and "
                    f"at most {mx:.1f} m.",
                    "• This demonstration flight was recorded separately from the main AirSim replay "
                    "(RMS error 1.2 m there), with the route moved onto AirSim's terrain and on a different computer.",
                    "• This shows that the urban wind field can be transferred to an independent simulator and "
                    "that the planned route is flyable by a different drone.",
                    "• It does not replace the main study: AirSim's drone is lighter and has less drag, so where "
                    "its limits lie in strong wind is different."]), 14)
    ff.stdin.close()
    ff.wait()
    print("saved", OUT, f"{len(ts) / FPS:.0f} s of mission footage")


if __name__ == "__main__":
    main()
