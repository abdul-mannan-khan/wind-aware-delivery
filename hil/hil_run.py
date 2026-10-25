#!/usr/bin/env python3
"""Hardware-in-the-loop (PX4 HITL) delivery missions in the urban wind field.

The Pixhawk runs the real PX4 firmware (EKF2 estimator, position, attitude and rate controllers) on
simulated sensors. This script, on the Jetson companion computer, is
  - the physics: the 6-DOF quadrotor of sim/quadrotor.py in the urban wind field + Dryden gusts,
    driven by the Pixhawk's motor outputs (HIL_ACTUATOR_CONTROLS) and returning IMU, magnetometer,
    barometer (HIL_SENSOR, 250 Hz) and GPS (HIL_GPS, 10 Hz);
  - the companion: it runs the delivery planner of sim/mission.py on the Jetson (timed), and flies the
    planned reference with PX4 in OFFBOARD mode (position + velocity + acceleration setpoints, 50 Hz).
Tracking error, energy, clearance and success are computed from the true simulated state with the same
rules as sim/mission.py, so HIL and pure simulation are directly comparable.

  python3 hil_run.py --U 12 --theta 0 --customer 0 --planner wind          # one mission
  python3 hil_run.py --grid hil_grid.yaml                                    # many missions
  python3 hil_run.py --planner test --U 0                                    # short test pattern, no wind
  python3 hil_run.py --grid hil_grid_v2_cities.yaml --config ../configs/v2/v2_london.yaml   # a real city
  python3 hil_run.py --grid hil_grid_v2_cities.yaml --config ../configs/v2/v2_london.yaml --dry-run
--config takes the experiment files of sim/run_experiments.py: their city (procedural or OpenStreetMap), planner,
wind and gust settings are used, so HIL and the simulation campaign fly the same missions. --dry-run plans every
mission of the grid without a flight controller (planner outcome and planning time only).
Only one process may open the serial port.
"""
import argparse, concurrent.futures as cf, csv, json, os, sys, time
import numpy as np
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))                # repository layout
sys.path.insert(0, os.path.join(HERE, 'sim'))                      # or a copy of sim/ next to this script
import mission                                                    # noqa: E402
from city import City                                             # noqa: E402
from quadrotor import G, Params, Quadrotor, rotor_power           # noqa: E402
from wind import DrydenGust, WindField                            # noqa: E402
import run_experiments as rx                                      # noqa: E402

# ENU world / FLU body (simulator)  <->  NED world / FRD body (PX4)
C = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
D = np.diag([1.0, -1, -1])
LAT0, LON0, ALT0 = 51.5074, -0.1278, 20.0                         # origin of the simulated city (London)
R_EARTH = 6378137.0
MAG_B, MAG_I, MAG_D = 0.489, np.radians(66.3), np.radians(1.0)    # WMM near the origin (gauss, inclination, declination)
M_NED = MAG_B * np.array([np.cos(MAG_I) * np.cos(MAG_D), np.cos(MAG_I) * np.sin(MAG_D), np.sin(MAG_I)])
M_ENU = C @ M_NED
YAW_NED = np.pi / 2                                               # simulator yaw 0 (nose east) = heading 90 deg
RECOVERY_MIN_Z = 80.0                                             # above every building of the procedural city
CONFIG = None                                                     # experiment file (--config); None = procedural city
_ENV = {}                                                         # per-process cache: config, city


def load_env(config):
    """City and planner/wind/gust settings of an experiment file, installed in this process (cached)."""
    if _ENV.get('config') != config or 'city' not in _ENV:
        cfg = rx.load_config(config) if config else {}
        osm = (cfg.get('city') or {}).get('osm')
        if osm and not os.path.isabs(osm):                        # paths in the configs are relative to sim/
            cfg['city']['osm'] = os.path.normpath(os.path.join(HERE, '..', 'sim', osm))
        rx.apply_config(cfg)
        _ENV.update(config=config, cfg=cfg, city=rx.make_city(cfg.get('city')))
    return _ENV['cfg'], _ENV['city']


def recovery_z(city, a, b, margin=10.0):
    """Altitude that clears every roof on the straight line a -> b (horizontal), at least RECOVERY_MIN_Z."""
    if 'hmap' not in _ENV or _ENV.get('hmap_city') is not city:
        _ENV['hmap'], _ENV['hmap_city'] = city.height_map(2.0), city
    xs, H = _ENV['hmap']
    pts = np.linspace(np.asarray(a)[:2], np.asarray(b)[:2], 400)
    idx = np.clip(np.round(pts / (xs[1] - xs[0]) - 0.5).astype(int), 0, len(xs) - 1)
    return max(RECOVERY_MIN_Z, float(H[idx[:, 0], idx[:, 1]].max()) + margin)
DT_PHYS = 0.004


def plan_job(U, theta, customer, planner, config=None):
    """Runs in a child process so the 250 Hz loop is never blocked."""
    cfg, city = load_env(config)
    t0 = time.time()
    wind = WindField(city, U, theta, **(cfg.get('wind') or {}))
    t_wind = time.time() - t0
    t0 = time.time()
    if planner == 'test':                                         # short pattern to check the HIL loop
        from planner import Reference, time_parametrise
        p0 = np.array([*mission.DEPOT, 2.0])
        pts = np.array([p0, p0 + [0, 0, 8], p0 + [40, 0, 8], p0 + [40, 40, 8], p0 + [0, 0, 8], p0])
        ref = Reference()
        ref.hold(p0, 3.0)
        for a, b in zip(pts[:-1], pts[1:]):
            ref.add(time_parametrise(np.array([a, b]), 5.0, a_max=1.5))
            ref.hold(b, 3.0)
        ref.finalize()
        pred = dict(E_pred_out=0.0, E_pred_back=0.0, z_out=10, z_back=10, v_out=5, v_back=5)
        return dict(city=city, wind=wind, ref=ref, pred=pred, t_plan=0.0, t_wind=t_wind)
    try:
        ref, pred = mission.build_reference(city, wind, mission.CUSTOMERS[customer], planner, Params())
    except mission.MissionRejected:
        ref, pred = None, None
    return dict(city=city, wind=wind, ref=ref, pred=pred, t_plan=time.time() - t0, t_wind=t_wind)


class Sim:
    """Physics + sensors. World ENU, body FLU, as in sim/quadrotor.py."""

    def __init__(self, P, p0, rng):
        self.P, self.rng = P, rng
        self.quad = Quadrotor(P, p0)
        km, L = P.k_m, P.arm / np.sqrt(2)
        # physical yaw reaction: rotors 1,2 spin CCW (seen from above) -> negative torque about up
        self.quad.Mix = np.array([[1, 1, 1, 1], [-L, L, L, -L], [-L, L, -L, L], [-km, -km, km, km]])
        self.quad.T = np.zeros(4)
        self.u = np.zeros(4)
        self.wind_on = True
        self.city = self.wind = self.gust = None
        self.w = np.zeros(3)
        self.u_mean = np.zeros(3)
        self.acc = np.zeros(3)
        self.gyro_bias = rng.normal(0, 0.002, 3)
        self.gps_err = np.zeros(3)

    def set_env(self, city, wind, gust):
        self.city, self.wind, self.gust = city, wind, gust

    def step(self, dt):
        q = self.quad
        airborne = q.p[2] > 0.05 or self.u.sum() * q.P.T_max > 0.9 * q.mass * G
        if self.wind is not None and self.wind_on and q.p[2] > 0.3:
            self.u_mean, ti = self.wind.mean(q.p)
            self.w = self.u_mean + self.gust.step(dt, self.u_mean, ti, np.linalg.norm(q.v - self.u_mean))
        else:
            self.u_mean, self.w = np.zeros(3), np.zeros(3)
        v0 = q.v.copy()
        T_cmd = np.clip(self.u, 0, 1) * q.P.T_max
        if airborne:
            q.step(dt, T_cmd, self.w)
        else:
            q.T = q.T + (T_cmd - q.T) * min(dt / q.P.tau_m, 1.0)
        if q.p[2] < 0.0:                                          # ground contact: stop, level the vehicle
            yaw = np.arctan2(q.R[1, 0], q.R[0, 0])
            q.p[2], q.v[:], q.w[:] = 0.0, 0.0, 0.0
            q.q = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        self.acc = (q.v - v0) / dt

    # -- sensors (PX4 conventions) -----------------------------------------------------------------
    def hil_sensor(self, mav, t_us):
        q, rng = self.quad, self.rng
        R = q.R
        f_flu = R.T @ (self.acc + G * np.array([0, 0, 1.0]))      # specific force
        acc = D @ f_flu + rng.normal(0, 0.03, 3)
        gyro = D @ q.w + self.gyro_bias + rng.normal(0, 0.003, 3)
        mag = D @ (R.T @ M_ENU) + rng.normal(0, 0.003, 3)
        alt = ALT0 + q.p[2] + rng.normal(0, 0.05)
        p_hpa = 1013.25 * (1 - 2.25577e-5 * alt) ** 5.25588
        mav.hil_sensor_send(t_us, *acc, *gyro, *mag, p_hpa, 0.0, alt, 15.0, 0x1FFF)

    def hil_gps(self, mav, t_us):
        q = self.quad
        self.gps_err = 0.98 * self.gps_err + self.rng.normal(0, 0.03, 3)   # slowly varying GPS error
        n, e, d = C @ q.p + self.gps_err
        lat = LAT0 + np.degrees(n / R_EARTH)
        lon = LON0 + np.degrees(e / (R_EARTH * np.cos(np.radians(LAT0))))
        vn, ve, vd = C @ q.v
        cog = (np.degrees(np.arctan2(ve, vn)) + 360) % 360
        mav.hil_gps_send(t_us, 3, int(lat * 1e7), int(lon * 1e7), int((ALT0 - d) * 1000), 30, 40,
                         int(np.hypot(vn, ve) * 100), int(vn * 100), int(ve * 100), int(vd * 100),
                         int(cog * 100), 14)


class Link:
    def __init__(self, dev, origin):
        self.origin = np.asarray(origin, float)                   # PX4 local frame origin (EKF origin) in ENU
        self.m = mavutil.mavlink_connection(dev, baud=921600, source_system=255, source_component=190)
        self.mode = self.armed = None
        self.local = None
        self.status = []
        self.n_act = 0

    def poll(self, sim):
        while True:
            msg = self.m.recv_msg()
            if msg is None:
                return
            t = msg.get_type()
            if t == 'HIL_ACTUATOR_CONTROLS':
                sim.u = np.clip(np.array(msg.controls[:4]), 0.0, 1.0)
                self.n_act += 1
            elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = (msg.custom_mode >> 16) & 0xFF, (msg.custom_mode >> 24) & 0xFF
            elif t == 'LOCAL_POSITION_NED':
                self.local = np.array([msg.x, msg.y, msg.z])
            elif t == 'STATUSTEXT':
                self.status.append(msg.text)
                print(f'  PX4: {msg.text}', flush=True)

    def heartbeat(self):
        self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def cmd(self, c, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(1, 1, c, 0, *p)

    def setpoint(self, p_enu, v_enu, a_enu):
        p, v, a = C @ (p_enu - self.origin), C @ v_enu, C @ a_enu
        mask = 0b0000100000000000                                # use position, velocity, acceleration, yaw
        self.m.mav.set_position_target_local_ned_send(0, 1, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                                                      *p, *v, *a, YAW_NED, 0)


def run(args, missions, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    P = Params()
    depot_ground = np.array([*mission.DEPOT, 0.0])
    rng = np.random.default_rng(1234)
    sim = Sim(P, depot_ground, rng)
    link = Link(args.dev, depot_ground)
    for msg_id, hz in ((32, 20), (0, 2)):                        # LOCAL_POSITION_NED, HEARTBEAT
        link.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
    pool = cf.ProcessPoolExecutor(1)
    summary_path = os.path.join(out_dir, 'hil_summary.csv')
    new_file = not os.path.exists(summary_path)
    fsum = open(summary_path, 'a', newline='')
    wsum = None

    t_start = time.perf_counter()
    t_next = t_start
    k = 0
    state, job, mi = 'boot', None, 0
    st = {}                                                       # per-mission state
    t_state = time.perf_counter()
    print(f'HIL: {len(missions)} missions -> {out_dir}', flush=True)

    def enter(s):
        nonlocal state, t_state
        state, t_state = s, time.perf_counter()
        print(f'{time.strftime("%T")} [{mi + 1}/{len(missions)}] {s}', flush=True)

    while True:
        now = time.perf_counter()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += DT_PHYS
        if time.perf_counter() - t_next > 0.1:                    # fell far behind (e.g. GC pause): resync
            t_next = time.perf_counter()
        t_us = int((time.perf_counter() - t_start) * 1e6)

        link.poll(sim)
        sim.step(DT_PHYS)
        sim.hil_sensor(link.m.mav, t_us)
        if k % 25 == 0:
            sim.hil_gps(link.m.mav, t_us)
        if k % 125 == 0:
            link.heartbeat()
        k += 1
        if k % 5:
            continue

        # ---------------- mission logic at 50 Hz ----------------
        el = time.perf_counter() - t_state
        q = sim.quad
        if args.verbose and k % 500 == 0:
            yaw = np.degrees(np.arctan2(q.R[1, 0], q.R[0, 0]))
            est = np.round(C @ link.local + depot_ground, 2) if link.local is not None else None
            print(f'    {state:8s} p={np.round(q.p, 2)} v={np.round(q.v, 2)} yaw={yaw:.0f} u={np.round(sim.u, 2)} '
                  f'armed={link.armed} mode={link.mode} est_enu={est} act/s={link.n_act / 2:.0f}', flush=True)
            link.n_act = 0
        if state == 'boot':
            if el > args.warmup and link.local is not None:
                enter('plan')
            continue
        if state == 'plan':
            if job is None:
                U, th, cu, pl, seed = missions[mi]
                job = pool.submit(plan_job, U, th, cu, pl, CONFIG)
                st = dict(U=U, theta=th, customer=cu, planner=pl, seed=seed)
            elif job.done():
                r = job.result()
                job = None
                st.update(t_plan=r['t_plan'], t_wind=r['t_wind'])
                sim.wind_on = False
                sim.set_env(r['city'], r['wind'], DrydenGust(np.random.default_rng(st['seed']), **mission.GUST))
                if r['ref'] is None:
                    st.update(rejected=True, success=False)
                    enter('record')
                else:
                    st.update(rejected=False, ref=r['ref'], pred=r['pred'], events=list(r['ref'].events))
                    enter('takeoff')
            continue
        ref = st.get('ref')
        if state == 'takeoff':                                    # climb to the reference start in still air
            p0, _, _ = ref(0.0)
            link.setpoint(p0, np.zeros(3), np.zeros(3))
            if el > 1.0 and link.mode != (6, 0):
                link.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)
            if el > 1.5 and not link.armed and int(el * 50) % 50 == 0:
                link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if link.armed and link.mode == (6, 0) and np.linalg.norm(q.p - p0) < 0.5 and np.linalg.norm(q.v) < 0.3:
                st.update(t0=time.perf_counter(), E=0.0, err2=0.0, n=0, emax=0.0, cmin=99.0, tilt_max=0.0,
                          wmax=0.0, collided=False, fail=False, log=[], mass_dropped=False)
                sim.wind_on = True
                enter('fly')
            elif el > 300:
                st.update(note='takeoff timeout')
                enter('recover')
            continue
        if state == 'fly':
            t = time.perf_counter() - st['t0']
            ev = st['events']
            if ev and t >= ev[0][0]:
                if ev[0][1] == 'drop':
                    q.drop_payload()
                ev.pop(0)
            p_d, v_d, a_d = ref(min(t, ref.T_end))
            link.setpoint(p_d, v_d, a_d)
            dt = 5 * DT_PHYS
            P_el = rotor_power(q.T, q.v - sim.w, q.R @ np.array([0, 0, 1.0]), P)
            st['E'] += P_el * dt
            e = float(np.linalg.norm(q.p - p_d))
            st['err2'] += e * e
            st['n'] += 1
            st['emax'] = max(st['emax'], e)
            st['wmax'] = max(st['wmax'], float(np.linalg.norm(sim.w)))
            tilt = float(np.degrees(np.arccos(np.clip(q.R[2, 2], -1, 1))))
            st['tilt_max'] = max(st['tilt_max'], tilt)
            c = mission.clearance(sim.city, q.p)
            st['cmin'] = min(st['cmin'], c)
            if c < 0.4 and q.p[2] > 0.5:
                st['collided'] = True
            if st['n'] % 5 == 0:
                est = C @ link.local + depot_ground if link.local is not None else np.full(3, np.nan)
                st['log'].append([round(t, 2), *q.p, *p_d, *est, *sim.u_mean, *sim.w, P_el, st['E'], *sim.u, tilt,
                                  q.mass, link.mode[0] if link.mode else -1])
            if st['collided'] or e > 25.0 or not link.armed:
                st['fail'] = True
                st['t_mission'] = t
                enter('recover')
            elif t >= ref.T_end:
                st['t_mission'] = t
                enter('land')
            continue
        if state == 'recover':                                    # wind off, climb above the roofs, fly home
            sim.wind_on = False
            if 'rec_phase' not in st:
                st['rec_phase'] = 0
                st['rec_from'] = q.p.copy()
                st['rec_z'] = recovery_z(sim.city, q.p, mission.DEPOT)
            RECOVERY_Z = st['rec_z']
            home_hi = np.array([*mission.DEPOT, RECOVERY_Z])
            if not link.armed:
                enter('land')
                continue
            tgt = [np.array([*st['rec_from'][:2], RECOVERY_Z]), home_hi, np.array([*mission.DEPOT, 2.0])][st['rec_phase']]
            link.setpoint(tgt, np.zeros(3), np.zeros(3))
            if np.linalg.norm(q.p - tgt) < 1.0:
                st['rec_phase'] += 1
                if st['rec_phase'] == 3:
                    enter('land')
            elif el > 400:
                enter('land')
            continue
        if state == 'land':
            sim.wind_on = False
            if link.armed:
                if int(el * 50) % 100 == 0:
                    link.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, np.nan, np.nan, np.nan, np.nan)
                if el > 90:                                         # still airborne: force disarm (sim only)
                    link.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
            elif el > 3:
                if np.linalg.norm(q.p[:2] - depot_ground[:2]) > 1.0:
                    st['note'] = (st.get('note', '') + ' landed away from depot').strip()
                enter('record')
            continue
        if state == 'record':
            row = dict(U_ref=st['U'], theta=st['theta'], customer=st['customer'], planner=st['planner'],
                       controller='px4', seed=st['seed'], rejected=st['rejected'], t_plan_s=round(st['t_plan'], 2),
                       t_wind_s=round(st['t_wind'], 2), host='jetson-orin-nano-super+pixhawk6c-px4-1.15.1')
            if not st['rejected']:
                pred = st['pred']
                finished = not st['fail']
                Wh = st['E'] / 3600
                row.update(success=bool(finished and not st['collided'] and st['emax'] < 10.0 and Wh < P.battery_Wh),
                           collided=st['collided'], tracking_fail=bool(st['fail'] and not st['collided']),
                           t_mission=round(st['t_mission'], 1), E_Wh=round(Wh, 3),
                           E_pred_Wh=round((pred['E_pred_out'] + pred['E_pred_back']) / 3600, 3),
                           rms_err=round(np.sqrt(st['err2'] / max(st['n'], 1)), 3), max_err=round(st['emax'], 3),
                           min_clear=round(st['cmin'], 2), tilt_max=round(st['tilt_max'], 1),
                           wind_max=round(st['wmax'], 2), z_out=pred['z_out'], z_back=pred['z_back'],
                           v_out=pred['v_out'], v_back=pred['v_back'], note=st.get('note', ''))
                name = f"U{st['U']:g}_th{st['theta']:g}_c{st['customer']}_{st['planner']}_s{st['seed']}"
                cols = ['t', 'x', 'y', 'z', 'xd', 'yd', 'zd', 'xe', 'ye', 'ze', 'umx', 'umy', 'umz', 'wx', 'wy', 'wz',
                        'P', 'E', 'u1', 'u2', 'u3', 'u4', 'tilt', 'mass', 'px4_mode']
                np.savetxt(os.path.join(out_dir, f'log_{name}.csv'), np.array(st['log']), delimiter=',',
                           header=','.join(cols), comments='', fmt='%.4g')
            else:
                row.update(success=False)
            keys = ['U_ref', 'theta', 'customer', 'planner', 'controller', 'seed', 'rejected', 'success', 'collided',
                    'tracking_fail', 't_mission', 'E_Wh', 'E_pred_Wh', 'rms_err', 'max_err', 'min_clear', 'tilt_max',
                    'wind_max', 'z_out', 'z_back', 'v_out', 'v_back', 't_plan_s', 't_wind_s', 'host', 'note']
            if wsum is None:
                wsum = csv.DictWriter(fsum, keys)
                if new_file:
                    wsum.writeheader()
            wsum.writerow({kk: row.get(kk, '') for kk in keys})
            fsum.flush()
            print('  RESULT ' + json.dumps({kk: row.get(kk) for kk in keys if kk in row}), flush=True)
            # reset the vehicle mass and position for the next mission (landed at the depot, disarmed)
            q.mass = P.m_frame + P.m_payload
            if np.linalg.norm(q.p[:2] - depot_ground[:2]) > 3.0:
                print('  vehicle not at the depot; stopping so the estimator can be reset', flush=True)
                break
            mi += 1
            if mi >= len(missions):
                break
            enter('plan')
    fsum.close()
    pool.shutdown()
    print('HIL done', flush=True)


def dry_run(a, missions):
    """Plan every mission (no flight controller): outcome of the planner, planning time, predicted energy."""
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for U, th, cu, pl, seed in missions:
        r = plan_job(U, th, cu, pl, CONFIG)
        ok = r['ref'] is not None
        pred = r['pred'] or {}
        rows.append(dict(U=U, theta=th, customer=cu, planner=pl, seed=seed, planned=ok,
                         T_ref_s=round(r['ref'].T_end, 1) if ok else '', t_plan_s=round(r['t_plan'], 2),
                         E_pred_Wh=round(pred.get('E_pred_out', 0) + pred.get('E_pred_back', 0), 2) if ok else ''))
        print(f"U={U:4.0f} th={th:3.0f} c={cu} {pl:8s} {'planned' if ok else 'REFUSED':8s} "
              f"T_ref={rows[-1]['T_ref_s']} s  plan {r['t_plan']:.1f} s", flush=True)
    f = os.path.join(a.out, 'dry_run.csv')
    with open(f, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'dry run: {len(rows)} missions, {sum(r["planned"] for r in rows)} planned -> {f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dev', default='/dev/ttyACM0')
    ap.add_argument('--U', type=float, default=12.0)
    ap.add_argument('--theta', type=float, default=0.0)
    ap.add_argument('--customer', type=int, default=0)
    ap.add_argument('--planner', default='wind')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--grid', help='YAML with lists U_ref, theta, customer, planner, seed')
    ap.add_argument('--warmup', type=float, default=40.0, help='s of sensor data before the first mission (EKF)')
    ap.add_argument('--out', default=os.path.join(HERE, 'results'))
    ap.add_argument('--verbose', action='store_true', help='print the vehicle state every 2 s')
    ap.add_argument('--config', help='experiment file of sim/run_experiments.py (city, planner, wind, gust settings)')
    ap.add_argument('--dry-run', action='store_true', help='plan every mission without a flight controller')
    a = ap.parse_args()
    global CONFIG
    CONFIG = os.path.abspath(a.config) if a.config else None
    load_env(CONFIG)                                              # sets the depot and customers of the city here too
    if a.grid:
        import itertools, yaml
        g = yaml.safe_load(open(a.grid))
        missions = list(itertools.product(g['U_ref'], g['theta'], g['customer'], g['planner'], g.get('seed', [0])))
    else:
        missions = [(a.U, a.theta, a.customer, a.planner, a.seed)]
    if a.dry_run:
        dry_run(a, missions)
        return
    import signal
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    try:
        run(a, missions, a.out)
    finally:                                                      # never leave PX4 armed when the script ends
        m = mavutil.mavlink_connection(a.dev, baud=921600, source_system=255, source_component=190)
        for _ in range(5):
            m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
            time.sleep(0.1)


if __name__ == '__main__':
    main()
