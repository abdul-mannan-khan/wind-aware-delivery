"""Independent check in AirSim (SimpleFlight + PhysX): fly the planned delivery missions while the
urban wind field is injected at the vehicle position every step through simSetWind.

Frames: simulator world (x east, y north, z up), AirSim NED (x north, y east, z down) relative to the
vehicle start. The mission is shifted up by Z_OFFSET so that the AirSim map geometry (which is not
our procedural city) does not interfere; the wind is still evaluated at the simulator coordinates.
Usage (inside the container, after `start_airsim`):  python3 airsim_replay.py [--one]
"""
import csv
import os
import sys
import time

from airsim_compat import airsim
import numpy as np

from city import City
from mission import CUSTOMERS, build_reference
from quadrotor import Params, quat_to_R, rotor_power
from wind import DrydenGust, WindField

DT = 0.05           # s, control / wind update period (sim time, lock-stepped)
Z_OFFSET = 40.0     # m, keep clear of the AirSim map geometry
KP = 1.2            # outer-loop position gain -> velocity command
OUT = "../results/airsim"


def to_ned(p):
    return np.array([p[1], p[0], -p[2]])


def from_ned(q):
    return np.array([q[1], q[0], -q[2]])


def fly(client, city, wind, cust, planner, seed=0, record_frames=False, tag=""):
    P = Params()
    ref, pred = build_reference(city, wind, CUSTOMERS[cust], planner, P)
    rng = np.random.default_rng(seed)
    gust = DrydenGust(rng)
    p0, _, _ = ref(0.0)
    origin = p0.copy()
    client.reset()
    time.sleep(1.0)
    client.enableApiControl(True)
    client.armDisarm(True)
    client.simSetWind(airsim.Vector3r(0, 0, 0))
    # take off and go to the start point (shifted up by Z_OFFSET)
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -(Z_OFFSET + p0[2] - origin[2]), 3).join()
    client.simPause(True)
    t, log = 0.0, []
    E = 0.0
    frame_k = 0
    while t < ref.T_end:
        st = client.getMultirotorState()
        k = st.kinematics_estimated
        p_ned = np.array([k.position.x_val, k.position.y_val, k.position.z_val])
        v_ned = np.array([k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val])
        p = from_ned(p_ned) + origin - np.array([0, 0, Z_OFFSET])   # simulator coordinates
        v = from_ned(v_ned)
        p_d, v_d, _ = ref(t)
        u_mean, ti = wind.mean(p)
        w = u_mean + gust.step(DT, u_mean, ti, np.linalg.norm(v - u_mean))
        wn = to_ned(w)
        client.simSetWind(airsim.Vector3r(*wn))
        v_cmd = v_d + KP * (p_d - p)
        sp = np.linalg.norm(v_cmd)
        if sp > 15.0:
            v_cmd *= 15.0 / sp
        vc = to_ned(v_cmd)
        client.moveByVelocityAsync(float(vc[0]), float(vc[1]), float(vc[2]), DT * 5,
                                   airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0))
        client.simContinueForTime(DT)
        # rotor thrusts -> electrical power with the same momentum model as the simulator
        rs = client.getRotorStates()
        T = np.array([r["thrust"] for r in rs.rotors]) if hasattr(rs, "rotors") else np.full(4, 0.0)
        q = k.orientation
        R_ned = quat_to_R([q.w_val, q.x_val, q.y_val, q.z_val])   # body FRD -> NED
        zb = from_ned(-R_ned[:, 2])                                # thrust axis in the simulator frame
        Pw = rotor_power(T, v - w, zb, P) if T.sum() > 0 else np.nan
        if np.isfinite(Pw):
            E += Pw * DT
        log.append([t, *p, *p_d, *u_mean, *w, Pw, *T])
        if record_frames and int(t / 2.0) > frame_k:
            frame_k = int(t / 2.0)
            img = client.simGetImage("chase", airsim.ImageType.Scene)
            if img:
                os.makedirs(f"{OUT}/frames_{tag}", exist_ok=True)
                with open(f"{OUT}/frames_{tag}/{frame_k:05d}.png", "wb") as f:
                    f.write(img)
        if np.linalg.norm(p - p_d) > 40:
            break
        t += DT
    client.simPause(False)
    client.simSetWind(airsim.Vector3r(0, 0, 0))
    L = np.array(log)
    err = np.linalg.norm(L[:, 1:4] - L[:, 4:7], axis=1)
    return dict(customer=cust, planner=planner, U_ref=wind.U_ref, theta=np.degrees(wind.theta) % 360,
                t_mission=t, E_Wh=E / 3600, E_pred_Wh=(pred["E_pred_out"] + pred["E_pred_back"]) / 3600,
                rms_err=float(np.sqrt((err ** 2).mean())), max_err=float(err.max()),
                completed=bool(t >= ref.T_end - DT)), L


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    client = airsim.MultirotorClient()
    client.confirmConnection()
    city = City()
    rows = []
    cases = [(12.0, 0.0, "wind"), (12.0, 0.0, "shortest"), (12.0, 180.0, "wind"), (12.0, 180.0, "shortest")]
    if len(sys.argv) > 1 and sys.argv[1] == "--one":
        cases = cases[:1]
    for U, th, pl in cases:
        wind = WindField(city, U, th)
        tag = f"U{U:.0f}_th{th:.0f}_{pl}"
        r, L = fly(client, city, wind, 0, pl, record_frames=(tag == "U12_th0_wind"), tag=tag)
        print(r, flush=True)
        rows.append(r)
        np.save(f"{OUT}/log_{tag}.npy", L)
        with open(f"{OUT}/airsim_summary.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
