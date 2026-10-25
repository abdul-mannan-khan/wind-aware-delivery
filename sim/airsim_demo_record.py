"""Re-record the AirSim demonstration (C1, U_ref = 12 m/s, wind toward 0 deg, wind-aware plan) so that the whole
route lies over the AirSim map's scenery, with the chase camera recorded at 4 frames per second.

The scenario is rigidly rotated by PSI and shifted so that the route centre sits at CENTRE (AirSim NED, metres
from the vehicle spawn point). Route and wind are rotated together, so the relative geometry of the flight in
the wind is unchanged. Physics, control and wind injection are the same as in airsim_replay.py.
Usage (inside the container, after `start_airsim`; the demo used 0 -20 15 0.25):
    python3 airsim_demo_record.py PSI_DEG CENTRE_N CENTRE_E [FRAME_DT] [T_MAX]
"""
import io
import os
import sys
import time

from airsim_compat import airsim
import numpy as np

from airsim_replay import DT, KP, Z_OFFSET
from city import City
from mission import CUSTOMERS, build_reference
from quadrotor import Params
from wind import DrydenGust, WindField

OUT = "../results/airsim/demo"
FRAME_DT = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25     # 0 = no camera frames
T_MAX = float(sys.argv[5]) if len(sys.argv) > 5 else 1e9
PSI = np.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
CENTRE = np.array([float(sys.argv[2]), float(sys.argv[3])]) if len(sys.argv) > 3 else np.zeros(2)
C, S = np.cos(PSI), np.sin(PSI)
ROT = np.array([[C, -S], [S, C]])


def setup(route_xy):
    global MID
    MID = 0.5 * (route_xy.min(0) + route_xy.max(0))


def to_scene(p):
    """Simulator point (x east, y north, z up) -> AirSim NED relative to spawn (rotated, shifted, raised)."""
    q = ROT @ (p[:2] - MID)                         # rotated east/north about the route centre
    return np.array([CENTRE[0] + q[1], CENTRE[1] + q[0], -(p[2] + Z_OFFSET)])


def from_scene(n):
    q = np.array([n[1] - CENTRE[1], n[0] - CENTRE[0]])
    xy = ROT.T @ q + MID
    return np.array([xy[0], xy[1], -n[2] - Z_OFFSET])


def vec_to_scene(v):
    q = ROT @ v[:2]
    return np.array([q[1], q[0], -v[2]])


def vec_from_scene(n):
    xy = ROT.T @ np.array([n[1], n[0]])
    return np.array([xy[0], xy[1], -n[2]])


def main():
    os.makedirs(f"{OUT}/frames", exist_ok=True)
    city = City()
    wind = WindField(city, 12.0, 0.0)
    P = Params()
    ref, pred = build_reference(city, wind, CUSTOMERS[0], "wind", P)
    ts = np.arange(0, ref.T_end, 0.5)
    setup(np.array([ref(t)[0][:2] for t in ts]))
    corners = np.array([to_scene(ref(t)[0])[:2] for t in ts])
    print("route in scene NED: N", corners[:, 0].min(), corners[:, 0].max(), "E", corners[:, 1].min(),
          corners[:, 1].max(), flush=True)
    gust = DrydenGust(np.random.default_rng(0))
    c = airsim.MultirotorClient()
    c.confirmConnection()
    c.reset()
    time.sleep(1.0)
    c.enableApiControl(True)
    c.armDisarm(True)
    c.simSetWind(airsim.Vector3r(0, 0, 0))
    c.takeoffAsync().join()
    p0 = ref(0.0)[0]
    s0 = to_scene(p0)
    if np.hypot(*s0[:2]) > 1.0:                  # teleport only when the start is away from the spawn point
        c.simSetVehiclePose(airsim.Pose(airsim.Vector3r(*s0), airsim.to_quaternion(0, 0, 0)), True)
    c.moveToPositionAsync(*s0, 2).join()
    c.hoverAsync().join()
    time.sleep(2.0)
    for _ in range(3):                           # settle exactly on the start point
        c.moveToPositionAsync(*s0, 1).join()
        time.sleep(1.0)
    k0 = c.getMultirotorState().kinematics_estimated.position
    print("start offset (m):", np.round(np.array([k0.x_val, k0.y_val, k0.z_val]) - s0, 2), flush=True)
    c.simPause(True)
    t, log, nf = 0.0, [], 0
    while t < min(ref.T_end, T_MAX):
        k = c.getMultirotorState().kinematics_estimated
        pn = np.array([k.position.x_val, k.position.y_val, k.position.z_val])
        vn = np.array([k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val])
        p, v = from_scene(pn), vec_from_scene(vn)
        p_d, v_d, _ = ref(t)
        u_mean, ti = wind.mean(p)
        w = u_mean + gust.step(DT, u_mean, ti, np.linalg.norm(v - u_mean))
        c.simSetWind(airsim.Vector3r(*vec_to_scene(w)))
        v_cmd = v_d + KP * (p_d - p)
        sp = np.linalg.norm(v_cmd)
        if sp > 15.0:
            v_cmd *= 15.0 / sp
        vc = vec_to_scene(v_cmd)
        c.moveByVelocityAsync(float(vc[0]), float(vc[1]), float(vc[2]), DT * 5,
                              airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, np.degrees(PSI) * 0))
        if FRAME_DT > 0 and t >= nf * FRAME_DT - 1e-9:
            img = c.simGetImage("chase", airsim.ImageType.Scene)
            if img:
                open(f"{OUT}/frames/{nf:05d}.png", "wb").write(img)
            nf += 1
        log.append([t, *p, *p_d, *u_mean, *w, *pn])
        c.simContinueForTime(DT)
        c.getRotorStates()                       # as in airsim_replay.py: wait for the step to finish
        if np.linalg.norm(p - p_d) > 40:
            print("lost track", t, flush=True)
            break
        t += DT
        if int(t / DT) % 400 == 0:
            print(f"t={t:.0f}s frames={nf}", flush=True)
    c.simPause(False)
    c.simSetWind(airsim.Vector3r(0, 0, 0))
    L = np.array(log)
    np.save(f"{OUT}/log{'' if FRAME_DT > 0 else '_noframes'}.npy", L)
    err = np.linalg.norm(L[:, 1:4] - L[:, 4:7], axis=1)
    print("done", t, "of", ref.T_end, "frames", nf, "rms", np.sqrt((err ** 2).mean()), "max", err.max(), flush=True)


if __name__ == "__main__":
    main()
