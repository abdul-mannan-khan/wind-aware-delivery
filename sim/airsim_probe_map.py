"""Find the extent of the AirSim map: positions of all scene objects, plus top-down images from high up."""
import json
import os

from airsim_compat import airsim
import numpy as np

OUT = "../results/airsim/probe"
os.makedirs(OUT, exist_ok=True)
c = airsim.MultirotorClient()
c.confirmConnection()
names = c.simListSceneObjects()
objs = []
for n in names:
    p = c.simGetObjectPose(n).position
    if np.isfinite([p.x_val, p.y_val, p.z_val]).all():
        objs.append((n, p.x_val, p.y_val, p.z_val))
json.dump(objs, open(f"{OUT}/objects.json", "w"))
xy = np.array([[o[1], o[2]] for o in objs])
print(len(objs), "objects; x range", np.percentile(xy[:, 0], [1, 50, 99]), "y range", np.percentile(xy[:, 1], [1, 50, 99]))
c.enableApiControl(True)
c.armDisarm(True)
c.takeoffAsync().join()
# top-down images on a grid, from 350 m up (FOV 90 -> 700 m wide footprint)
for gx in (-300, 0, 300):
    for gy in (-300, 0, 300):
        c.simSetVehiclePose(airsim.Pose(airsim.Vector3r(gx, gy, -350), airsim.to_quaternion(0, 0, 0)), True)
        c.hoverAsync().join()
        img = c.simGetImage("down", airsim.ImageType.Scene)
        open(f"{OUT}/down_{gx}_{gy}.png", "wb").write(img)
        print("img", gx, gy, len(img) if img else 0, flush=True)
