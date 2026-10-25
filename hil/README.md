# Hardware-in-the-loop (PX4 HITL)

The delivery missions flown by the real PX4 autopilot. The flight controller runs the PX4 firmware (EKF2
estimator, position, attitude and rate controllers, mixer) on simulated sensors; a companion computer runs the
planner, the vehicle physics, the urban wind and the sensor models of `sim/`, and streams offboard setpoints.

```
companion computer (hil_run.py)                          flight controller (PX4, HITL)
  planner (sim/mission.py)  -- setpoints 50 Hz -->        offboard position/velocity/acceleration
  6-DOF physics + urban wind  <-- motor outputs --        HIL_ACTUATOR_CONTROLS
  IMU, mag, baro 250 Hz; GPS 10 Hz  -- sensors -->        EKF2
```

Outcomes (delivered / refused / failed), tracking error, clearance and energy are computed from the true simulated
state with the same rules as `sim/mission.py`, so HIL and pure simulation can be compared mission by mission.

## Hardware used

- Holybro Pixhawk 6C, PX4 v1.15.1, airframe "HIL Quadcopter X" (`SYS_AUTOSTART 1001`, `SYS_HITL 1`).
- NVIDIA Jetson Orin Nano Super (JetPack 6.2) as companion computer, connected over USB (`/dev/ttyACM0`, 921600 baud).
- **Propellers removed and USB power only.** HITL arms the outputs; never run it with propellers fitted.

## Files

| File | Purpose |
|---|---|
| `hil_run.py` | one mission, or a grid of missions (`--grid`), in HITL; writes per-mission JSON and a summary CSV |
| `hil_grid.yaml` | 32 missions: 12 and 16 m/s, 4 directions, 2 customers, shortest path and wind-aware planner |
| `hil_grid_v2.yaml` | the wind-aware planner with per-segment speeds (16 missions) |
| `param_dump.py` | read **all** flight-controller parameters into a QGroundControl `.params` file (read-only) |
| `param_set.py` | set parameters with read-back, or restore a full `.params` backup |
| `backup_by_name.py` | parameter backup while the board runs HITL (it does not answer a full parameter list then) |
| `fc_state.py` | show armed state and mode; optional force-disarm and reboot |

## Procedure

```bash
pip install pymavlink==2.4.49 numpy scipy pyyaml
python3 param_dump.py params_before.params                 # 1. back up every parameter
python3 param_set.py SYS_AUTOSTART=1001 SYS_HITL=1          # 2. HITL airframe (then reboot the board)
python3 param_set.py MPC_XY_VEL_MAX=20                      #    so PX4 does not clip planned speeds up to 18 m/s
python3 hil_run.py --planner test --U 0                     # 3. short test pattern, no wind
python3 hil_run.py --U 12 --theta 0 --customer 0 --planner wind
python3 hil_run.py --grid hil_grid_v2.yaml                  # 4. the campaign
python3 param_set.py --restore params_before.params         # 5. restore, then dump again and compare
```

Only one process may open the serial port; close QGroundControl first.
