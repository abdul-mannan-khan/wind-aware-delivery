# Running and changing experiments

All commands are run from the `sim/` folder of the repository (`cd ~/wind-aware-delivery/sim` in the
container).

## 1. The three tools

| Command | What it does | Output |
|---|---|---|
| `python3 fly_one.py [options]` | flies **one** mission and draws it | `results/single/<name>.png` + numbers on screen |
| `python3 run_experiments.py --config FILE` | flies **every combination** listed in a config file, in parallel | `results/experiments/<name>/campaign.csv` (+ a copy of the config) |
| `python3 summarise.py CSV [CSV ...]` | turns one or more campaign files into a table and two figures | `summary.csv`, `success_rate.png`, `energy_per_parcel.png` next to the first CSV |

### One mission
```bash
python3 fly_one.py --U 12 --theta 180 --customer 0 --planner wind --controller dob
python3 fly_one.py --U 16 --theta 0 --planner shortest --controller pid      # this one fails: watch why
python3 fly_one.py --config ../configs/heavy_parcel.yaml --U 12 --theta 90   # with the changes of a config file
```
`--U` is the wind speed at 60 m (m/s); `--theta` is the direction the wind blows **toward** (0 = east,
90 = north, 180 = west, 270 = south). The depot is in the south-west corner; customer 0 is in the north-east.

### Many missions
```bash
python3 run_experiments.py --config ../configs/baseline_small.yaml --dry-run   # count the missions first
python3 run_experiments.py --config ../configs/baseline_small.yaml             # run
python3 run_experiments.py --config ../configs/baseline_small.yaml --workers 8 # use only 8 cores
```

### Compare experiments
```bash
python3 summarise.py ../results/experiments/baseline_small/campaign.csv \
                     ../results/experiments/heavy_parcel/campaign.csv
python3 summarise.py ../results/campaign.csv                      # the procedural-city campaign
python3 summarise.py A.csv B.csv --x theta --out ../results/compare_dirs   # wind direction on the x axis
```
Each experiment is labelled with its folder name in the legend.

## 2. Writing a config file

A config file has a `name` and up to six sections. Only `grid` is needed; leave out everything you do not change.

```yaml
name: tilt_limit_30
description: Stricter steady-tilt limit for the wind-aware planner.
grid:                       # the factors that are swept; every combination is flown
  U_ref: [4, 8, 12, 16]
  theta: [0, 90, 180, 270]
  customer: [0]
  planner: [wind]
  controller: [dob]
  seed: [0]
planner:                    # changes to the model (here: one planner setting)
  TILT_LIMIT: 30
```

Number of missions = product of the list lengths in `grid` (here 4 × 4 × 1 × 1 × 1 × 1 = 16).

| Section | What it changes | Defined in |
|---|---|---|
| `grid` | swept factors: `U_ref`, `theta`, `customer`, `planner`, `controller`, `seed` | `sim/run_experiments.py` |
| `vehicle` | mass, parcel, thrust, drag, propeller, efficiency, battery | `sim/quadrotor.py`, class `Params` |
| `planner` | candidate heights and speeds, tilt limit, nominal height/speed, drop height, climb/descent speed, observer gain, depot and customer positions | `sim/mission.py` (top of file) |
| `wind` | profile exponent, canopy decay, corridor speed-up, cross-street suppression, reference height | `sim/wind.py`, class `WindField` |
| `gust` | gust length scales and minimum gust level | `sim/wind.py`, class `DrydenGust` |
| `city` | random seed and building heights | `sim/city.py`, class `City` |

**All settings with their default values:** [`configs/template_all_options.yaml`](../configs/template_all_options.yaml).
A misspelt setting stops the run with an error that lists the allowed names, so a typo cannot silently be ignored.

Rules of thumb:
- **Change one thing at a time** and compare with a baseline that has the *same grid* (`baseline_small.yaml`).
- Gusts are random: use at least 2–3 `seed` values before concluding that a difference is real.
- The depot and customers must stay in streets. If you change the city layout (`n_blocks`, `block`, `street`,
  `avenue`), also set `planner: DEPOT/CUSTOMERS`, and check with `fly_one.py` first.
- Give every experiment a new `name`; an existing results folder is overwritten.

## 3. What the results mean

`campaign.csv` has one row per mission:

| Column | Meaning |
|---|---|
| `U_ref`, `theta`, `customer`, `planner`, `controller`, `seed` | the mission |
| `success` | parcel delivered and drone back: no collision, error always < 10 m, energy within the battery |
| `rejected` | the wind-aware planner **refused** the mission before take-off (no route within the tilt limit) |
| `collided`, `tracking_fail` | hit a building / lost the path by more than 25 m |
| `E_Wh`, `E_pred_Wh` | energy used, and energy predicted by the planner (Wh) |
| `rms_err`, `max_err` | path-tracking error (m) |
| `min_clear` | closest distance to a building (m) |
| `tilt_max`, `wind_max`, `sat_frac` | largest tilt (deg), strongest wind met (m/s), fraction of time the motors were saturated |
| `z_out`, `z_back`, `v_out`, `v_back` | cruise height (m) and speed (m/s) chosen for the outbound and return legs |
| `t_mission` | flight time (s) |
| `error` | only if the simulation crashed (should be empty) |

`summary.csv` (from `summarise.py`) per experiment × planner × controller × x value:

| Column | Meaning |
|---|---|
| `delivered_pct` | % of missions that delivered the parcel |
| `refused_pct` | % refused before take-off (only the wind-aware planner refuses) |
| `failed_in_flight_pct` | % that took off and failed (collision, lost track, battery) |
| `energy_delivered_Wh` | mean energy of the successful missions |
| `energy_per_parcel_Wh` | all energy spent in flight (failed flights included) ÷ parcels delivered — the fairest single number |
| `rms_err_m` | mean RMS tracking error of the flown missions |

You can also open `campaign.csv` in Excel, or in Python with `pandas`:
```python
import pandas as pd
d = pd.read_csv("../results/experiments/heavy_parcel/campaign.csv")
print(d.groupby(["planner", "U_ref"]).success.mean())
```

## 4. Running many experiments in one go

`run_batch.sh` runs several configs one after another and then summarises them together:

```bash
tmux new -s batch
bash run_batch.sh ../configs/baseline_small.yaml ../configs/heavy_parcel.yaml ../configs/tilt_limit_30.yaml
# Ctrl+B then D to detach; tmux attach -t batch to come back
```
The combined table and figures go to `results/experiments/batch_<date-time>/`, a log to `.../batch.log`.

To sweep **one** setting over several values, make the configs with a loop, for example parcel mass:
```bash
cd ~/wind-aware-delivery/configs
for m in 0.4 0.8 1.2; do
  sed "s/^name: .*/name: payload_$m/; s/m_payload: .*/m_payload: $m/" heavy_parcel.yaml > payload_$m.yaml
done
cd ../sim && bash run_batch.sh ../configs/baseline_small.yaml ../configs/payload_*.yaml
```

## 5. Single-speed campaign in the procedural city

```bash
python3 run_experiments.py --config ../configs/procedural_full.yaml --out ../results/campaign.csv
python3 make_logs.py          # six logged example missions (for the time-series figures)
python3 analyze.py            # figures and tables -> ../outputs/
python3 figs_v2.py            # tilt envelope and failure-mode figures
python3 demo_animation.py     # top-down animation -> ../results/demo_wind_corridors.mp4
```
