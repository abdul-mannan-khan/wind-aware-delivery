# AirSim check and demonstration video

The Python simulator in `sim/` is the main tool of the study. AirSim (Microsoft, Unreal Engine 4) is used as an
**independent check**: the planned mission is flown by AirSim's own quadrotor physics (SimpleFlight + PhysX) while
our urban wind field is injected at the drone position every 0.05 s with `simSetWind`. The AirSim map
(the "Neighborhood": houses, streets, trees) is only scenery — the buildings of our procedural city are not in
it, so the flight is lifted 40 m above the ground (`Z_OFFSET`).

AirSim needs a **GPU** instance (any NVIDIA card with 8 GB or more). It runs headless (no screen); pictures come
from the drone's cameras.

## 1. Start and stop
```bash
start_airsim          # starts the map, waits until the API on port 41451 answers (30 s - 6 min)
stop_airsim           # stops it
```
`start_airsim` uses `airsim/settings_demo.json` (SimpleFlight drone with a chase camera and a downward camera).
Another settings file can be given: `start_airsim ~/wind-aware-delivery/airsim/settings_simpleflight.json`.
The simulator log is `/tmp/airsim.log`.

## 2. Replay missions in AirSim
```bash
cd ~/wind-aware-delivery/sim
python3 airsim_replay.py --one      # wind-aware mission, 12 m/s toward east (~6 min)
python3 airsim_replay.py            # four missions: wind-aware and shortest, wind toward east and west (~25 min)
```
Results in `results/airsim/`:
- `airsim_summary.csv`: completion, energy, RMS and maximum tracking error of each flight;
- `log_<case>.npy`: time series (columns: t, position x y z, reference x y z, mean wind x y z, wind incl. gust x y z,
  power, four rotor thrusts);
- `frames_U12_th0_wind/`: chase-camera pictures, one every 2 s of mission time.

The replay runs in lock-step (`simPause` + `simContinueForTime`), so the result does not depend on how fast the
machine is — in principle. In practice the tracking error differed between machines: the wind-aware mission gave
RMS 1.2 m / maximum 3.0 m on two different RTX A4000 hosts, but RMS 2.6 m on a 4-core Quadro RTX 4000 host. Always
report the machine together with AirSim numbers, and use an RTX A4000 (or similar) with 16+ cores to compare with
the reported numbers.

## 3. The demonstration video
```bash
python3 airsim_demo_record.py 0 -20 15 0.25    # fly the demo mission over the map, 4 camera frames per second (~45 min)
python3 airsim_demo_video.py                   # build results/airsim/demo_airsim_explained.mp4 (needs ffmpeg)
```
The first command places the route (600 m × 430 m) in the middle of the Neighborhood map: arguments are the
rotation (deg), the north and east position of the route centre (m from the drone's spawn point), and the time
between camera frames (s). The map's terrain is roughly 750–800 m across and the houses cover the central ~340 m,
so a route placed elsewhere flies over empty background.

The script writes the video to `results/airsim/`.

## 4. Troubleshooting
| Symptom | Cause and fix |
|---|---|
| `WARNING: no NVIDIA GPU visible` | the instance has no GPU: rent one with a GPU |
| `AirSim stopped` with `Vulkan` or `VK_ERROR` in the log | the GPU driver's Vulkan part is missing: rent a different host (the NVIDIA container runtime must provide graphics support) |
| `AirSim did not answer within 6 min` | slow disk on first start; run `start_airsim` again (it continues if AirSim is still loading) |
| Python hangs at `confirmConnection` | AirSim is not running: `start_airsim` |
| Drone drifts away, `lost track` | the error grew above 40 m; check the wind speed and that nothing else is commanding the drone |
| Need a fresh start | `stop_airsim && start_airsim` |
