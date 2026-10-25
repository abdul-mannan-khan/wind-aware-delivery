"""One delivery mission: depot -> customer (lower, release parcel) -> depot, flown by the 6-DOF
quadrotor in the urban wind field. Returns metrics and (optionally) the logged time series."""
import numpy as np

from city import City
from planner import Planner, Reference, time_parametrise
from quadrotor import G, GeometricController, Params, Quadrotor, rotor_power, trim_power
from wind import DrydenGust, WindField

DEPOT = np.array([18.0, 18.0])
CUSTOMERS = [np.array([618.0, 442.0]), np.array([274.0, 618.0]), np.array([618.0, 106.0]),
             np.array([362.0, 362.0])]
ALTITUDES = (20.0, 35.0, 50.0)     # candidate cruise layers; 35 m is the nominal/baseline layer
Z_NOMINAL = 35.0
SPEEDS = (6.0, 8.0, 10.0, 12.0)   # candidate cruise ground speeds (m/s) per leg (SPEED_SCHEDULE off); 8 m/s is nominal
SPEED_SCHEDULE = True              # energy planners choose the ground speed per straight segment from SPEEDS_FINE
SPEEDS_FINE = (4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0)
V_NOMINAL = 8.0
TILT_LIMIT = 35.0                  # deg, steady tilt allowed by the wind-aware planner (10 deg gust margin)
Z_DROP = 6.0                       # parcel is released while hovering at this height
V_CLIMB = 2.5
V_DESCEND = 2.0
GUST = {}                          # extra DrydenGust arguments (L, floor), set from an experiment config
K_OBS = 4.0                        # disturbance-observer gain of the PID+DOB controller
CLEAR_H = 6.0                      # m, horizontal clearance the route planner keeps from walls
WIND_CUTOFF = 12.0                 # m/s, 'cutoff' baseline: no flight if the free-stream wind at cruise height exceeds it
# Baselines from the literature (all fly the nominal 35 m layer at 8 m/s unless stated):
#   'cutoff'    shortest path, refused if the free-stream wind at cruise height > WIND_CUTOFF (manufacturer rule)
#   'egate'     shortest path, refused if the predicted mission energy exceeds the usable battery
#               (energy-only feasibility check, Betti Sorbelli et al., IEEE T-ITS 2021)
#   'rienecker' minimum-energy route through the wind field at fixed altitude and airspeed, no tilt check
#               (Rienecker, Hildebrand and Pfifer, CEAS Aeronautical Journal 2023)
SHORTEST_LIKE = ("shortest", "cutoff", "egate")


class MissionRejected(Exception):
    """The wind-aware planner found no plan that respects the steady-tilt limit."""


def vertical_energy(z_from, z_to, mass, P):
    dz = z_to - z_from
    vz = V_CLIMB if dz > 0 else -V_DESCEND
    return abs(dz) / abs(vz) * trim_power(np.array([0, 0, vz]), np.zeros(3), mass, P)


def plan_leg(city, wind, a, b, z_from, z_to, mass, planner_mode, cache, P, lam=0.0):
    """Choose cruise altitude and route for one leg.
    'shortest': shortest path at the nominal 35 m layer (common practice).
    'energy0' : minimum predicted energy over the layers, but planned as if the air were still
                (wind-unaware energy planner; isolates the value of wind knowledge).
    'wind'    : minimum predicted energy over the layers using the urban wind map.
    Returns (E_true_pred, z, pts) where E_true_pred is the energy predicted with the real wind."""
    def planner(z, v, still, limit=90.0):
        key = (z, v, round(mass, 2), still, limit)
        if key not in cache:
            w = cache.setdefault("still", WindField(city, 0.0, 0.0)) if still else wind
            sched = v is None                                   # speed chosen per segment
            cache[key] = Planner(city, w, z_cruise=z, v_cruise=max(SPEEDS_FINE) if sched else v, clear_h=CLEAR_H,
                                 mass=mass, P=P, lam_turb=lam, tilt_limit=limit,
                                 speeds=SPEEDS_FINE if sched else None)
        return cache[key]

    # only the wind-aware planner knows the wind, so only it can enforce the steady-tilt margin
    tilt = TILT_LIMIT if planner_mode == "wind" else 90.0

    shortest = planner_mode in SHORTEST_LIKE
    if shortest or planner_mode == "rienecker":
        combos = [(Z_NOMINAL, V_NOMINAL)]
    elif SPEED_SCHEDULE:
        combos = [(z, None) for z in ALTITUDES]
    else:
        combos = [(z, v) for z in ALTITUDES for v in SPEEDS]
    still = planner_mode == "energy0"
    best = None
    for z, v in combos:
        pl = planner(z, v, still, tilt)
        try:
            pts = pl.plan(a, b, "shortest" if shortest else "energy")
        except RuntimeError:
            continue
        if not shortest and np.isfinite(pl.path_energy(pts)):
            # also consider the any-angle shortest route at this layer; keep the cheaper
            alt = pl.plan(a, b, "shortest")
            if pl.path_energy(alt) < pl.path_energy(pts):
                pts = alt
        E = pl.path_energy(pts) + vertical_energy(z_from, z, mass, P) + vertical_energy(z, z_to, mass, P)
        if not np.isfinite(E):
            continue
        if best is None or E < best[0]:
            best = (E, z, v, pts)
    if best is None:
        raise MissionRejected("no feasible route/altitude/speed within the tilt limit")
    E, z, v, pts = best
    pl = planner(z, v, still, tilt)
    v_fly = pl.seg_speeds(pts) if v is None else v          # per-segment speeds of the chosen plan
    if still:   # report the prediction under the real wind for the chosen plan, flown at the chosen speeds
        E = planner(z, v, False).path_energy_at(pts, np.broadcast_to(v_fly, (len(pts) - 1,))) + \
            vertical_energy(z_from, z, mass, P) + vertical_energy(z, z_to, mass, P)
    return E, z, v_fly, pts, planner(z, v, False)


def build_reference(city, wind, customer, planner_mode, P, lam=0.0):
    cache = {}
    m_full, m_empty = P.m_frame + P.m_payload, P.m_frame
    if planner_mode == "cutoff" and wind.U_ref * (Z_NOMINAL / wind.z_ref) ** wind.alpha > WIND_CUTOFF:
        raise MissionRejected("wind above the manufacturer limit")
    out = plan_leg(city, wind, DEPOT, customer, 2.0, Z_DROP, m_full, planner_mode, cache, P, lam)
    back = plan_leg(city, wind, customer, DEPOT, Z_DROP, 2.0, m_empty, planner_mode, cache, P, lam)
    if planner_mode == "egate" and (out[0] + back[0]) / 3600 > P.battery_Wh:
        raise MissionRejected("predicted energy exceeds the usable battery")
    ref = Reference()
    def vmean(v, pts):                                     # length-weighted mean ground speed of a leg
        L = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        return float(np.sum(np.broadcast_to(v, L.shape) * L) / max(L.sum(), 1e-9))
    pred = {"E_pred_out": out[0], "E_pred_back": back[0], "z_out": out[1], "z_back": back[1],
            "v_out": vmean(out[2], out[3]), "v_back": vmean(back[2], back[3]),
            "v_max_out": float(np.max(out[2])), "v_max_back": float(np.max(back[2])),
            "path_out": out[3], "path_back": back[3], "speeds_out": out[2], "speeds_back": back[2]}
    occ = lambda pl: (lambda q: pl.occ_tight[min(int(q[0] / pl.res), len(pl.xs) - 1),
                                            min(int(q[1] / pl.res), len(pl.xs) - 1)])
    p0 = np.array([*DEPOT, 2.0])
    ref.hold(p0, 3.0)
    for (E, z, v, pts, pl), z_end, ev in ((out, Z_DROP, "drop"), (back, 2.0, "land")):
        start = np.array([*pts[0], ref.P[-1][-1][2]])
        ref.add(time_parametrise(np.array([start, [*pts[0], z]]), V_CLIMB, a_max=1.0))
        poly = np.column_stack([pts, np.full(len(pts), z)])
        ref.add(time_parametrise(poly, v, occ_check=occ(pl)))
        ref.add(time_parametrise(np.array([[*pts[-1], z], [*pts[-1], z_end]]), V_DESCEND, a_max=1.0))
        ref.hold(np.array([*pts[-1], z_end]), 8.0, ev)
    ref.finalize()
    return ref, pred


def clearance(city, p):
    if hasattr(city, "clearance"):                      # raster city (osm_city.OSMCity)
        return city.clearance(p)
    b = city.buildings
    m = b[:, 4] > p[2] - 1.0
    if not m.any():
        return 99.0
    bb = b[m]
    dx = np.maximum.reduce([bb[:, 0] - p[0], np.zeros(len(bb)), p[0] - bb[:, 2]])
    dy = np.maximum.reduce([bb[:, 1] - p[1], np.zeros(len(bb)), p[1] - bb[:, 3]])
    return float(np.min(np.hypot(dx, dy)))


def run_mission(U_ref=8.0, theta=0.0, customer=0, planner="shortest", controller="pid", seed=0,
                dt=0.005, log_every=0.1, keep_log=False, city=None, wind=None, P=None, lam=0.0, plan_wind=None):
    """wind: the wind the vehicle flies in; plan_wind: the wind map the planner uses (default: the same)."""
    P = P or Params()
    city = city or City()
    wind = wind or WindField(city, U_ref, theta)
    rng = np.random.default_rng(seed)
    try:
        ref, pred = build_reference(city, plan_wind or wind, CUSTOMERS[customer], planner, P, lam)
    except MissionRejected:
        return dict(U_ref=U_ref, theta=theta, customer=customer, planner=planner, controller=controller,
                    seed=seed, success=False, rejected=True, collided=False)
    p0, _, _ = ref(0.0)
    quad = Quadrotor(P, p0)
    ctrl = GeometricController(quad, use_dob=(controller == "dob"), k_obs=K_OBS)
    gust = DrydenGust(rng, **GUST)
    events = list(ref.events)
    t, E, k = 0.0, 0.0, 0
    n_log = max(int(round(log_every / dt)), 1)
    err2, n_err, emax, cmin, tilt_max, sat_n, wmax, n = 0.0, 0, 0.0, 99.0, 0.0, 0, 0.0, 0
    collided = False
    log = []
    while t < ref.T_end:
        if events and t >= events[0][0]:
            if events[0][1] == "drop":
                quad.drop_payload()
                ctrl.reset_observer()
            events.pop(0)
        p_d, v_d, a_d = ref(t)
        u_mean, ti = wind.mean(quad.p)
        g = gust.step(dt, u_mean, ti, np.linalg.norm(quad.v - u_mean))
        w = u_mean + g
        T_cmd, sat = ctrl(dt, p_d, v_d, a_d)
        quad.step(dt, T_cmd, w)
        P_el = rotor_power(quad.T, quad.v - w, quad.R @ np.array([0, 0, 1.0]), P)
        E += P_el * dt
        e = np.linalg.norm(quad.p - p_d)
        err2 += e * e
        n_err += 1
        emax = max(emax, e)
        sat_n += sat
        n += 1
        wmax = max(wmax, np.linalg.norm(w))
        if k % 10 == 0:
            tilt_max = max(tilt_max, np.degrees(np.arccos(np.clip(quad.R[2, 2], -1, 1))))
            c = clearance(city, quad.p)
            cmin = min(cmin, c)
            if c < 0.4 and quad.p[2] > 0.5:
                collided = True
        if keep_log and k % n_log == 0:
            log.append([t, *quad.p, *p_d, *u_mean, *w, P_el, E, *ctrl.d_hat, *quad.F_aero, quad.mass])
        if collided or e > 25.0 or not np.isfinite(quad.p).all():
            break
        t += dt
        k += 1
    finished = t >= ref.T_end - dt
    res = dict(U_ref=U_ref, theta=theta, customer=customer, planner=planner, controller=controller,
               seed=seed, rejected=False, tracking_fail=bool(e > 25.0), success=bool(finished and not collided and emax < 10.0
                                       and E / 3600 < P.battery_Wh),
               collided=collided, t_mission=t, E_Wh=E / 3600, E_pred_Wh=(pred["E_pred_out"] + pred["E_pred_back"]) / 3600,
               rms_err=np.sqrt(err2 / max(n_err, 1)), max_err=emax, min_clear=cmin, tilt_max=tilt_max,
               sat_frac=sat_n / max(n, 1), wind_max=wmax, z_out=pred["z_out"], z_back=pred["z_back"],
               v_out=pred["v_out"], v_back=pred["v_back"], v_max_out=pred["v_max_out"],
               v_max_back=pred["v_max_back"])
    if keep_log:
        cols = ["t", "x", "y", "z", "xd", "yd", "zd", "umx", "umy", "umz", "wx", "wy", "wz", "P", "E",
                "dhx", "dhy", "dhz", "fax", "fay", "faz", "mass"]
        res["log"] = (cols, np.array(log))
        res["pred"] = pred
    return res


if __name__ == "__main__":
    import time
    t0 = time.time()
    r = run_mission(12.0, 180.0, 0, "wind", "dob")
    print({k: v for k, v in r.items()}, time.time() - t0)
