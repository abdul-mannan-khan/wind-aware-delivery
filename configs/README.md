# Experiment configs

| File | Missions | Purpose |
|---|---|---|
| `quick.yaml` | 12 | smoke test |
| `baseline_small.yaml` | 48 | reference for comparisons (default model) |
| `heavy_parcel.yaml` | 48 | parcel 1.0 kg instead of 0.6 kg |
| `tilt_limit_30.yaml` | 16 | wind-aware planner with a 30° tilt limit |
| `strong_gusts.yaml` | 288 | gustier air, three seeds |
| `procedural_full.yaml` | 1,536 | the version 1 campaign |
| `template_all_options.yaml` | – | every setting with its default value: copy and edit |
| `baselines_procedural.yaml` | 1,536 | literature baselines (`cutoff`, `egate`, `rienecker`) on the procedural city |
| `real_london.yaml`, `real_manhattan.yaml` | 3,072 each | all planners in the OpenStreetMap cities (`city: {osm: ...}`) |
| `sensitivity/sens_*.yaml` | 288 each | wind-model coefficients ±30 % (one speed per leg) |
| `tilt_real/tilt_*.yaml` | 128 each | tilt-limit sweep in the real cities (one speed per leg) |
| `v2/v2_*.yaml` | – | version 2 (speed per segment): cities, sensitivity, tilt sweep, check that unchanged planners are unchanged |
| `v2/qes_*.yaml` | – | missions flown in QES-Winds fields; `plan: qes` or `analytic` sets the planner's map |

Run: `cd sim && python3 run_experiments.py --config ../configs/<file>.yaml`.
How to write your own: [docs/EXPERIMENTS.md](../docs/EXPERIMENTS.md).
