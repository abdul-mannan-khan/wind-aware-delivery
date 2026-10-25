"""Writes the v3 campaign files (run from configs/v3): network of 40 demand-weighted customers, extra gust seeds,
a 14 m/s ground-speed cap, a tilt-margin sweep in the QES-Winds field and forecast-error missions.

  python3 make_configs.py
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
THETA = [0, 45, 90, 135, 180, 225, 270, 315]
ALL = ["shortest", "cutoff", "egate", "rienecker", "energy0", "wind"]
REAL = ["london", "manhattan"]


def city_part(c):
    if c == "procedural":
        return {}
    return {"city": {"osm": f"../data/cities/{c}.npz"}, "planner": {"CLEAR_H": 4.0}}


def write(name, desc, c, grid, planner=None, qes=None):
    cfg = {"name": name, "description": desc, **city_part(c)}
    if planner:
        cfg.setdefault("planner", {}).update(planner)
    if qes:
        cfg["qes"] = qes
    cfg["grid"] = grid
    with open(os.path.join(HERE, name + ".yaml"), "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def main():
    for c in REAL:
        cu = yaml.safe_load(open(os.path.join(HERE, f"customers_{c}.yaml")))
        n = len(cu["customers"])
        write(f"v3_net_{c}", f"Service-area network: {n} demand-weighted customers ({c}), all planners, PID+DOB.",
              c, {"U_ref": [4.0, 8.0, 12.0, 16.0], "theta": THETA, "customer": list(range(n)), "planner": ALL,
                  "controller": ["dob"], "seed": [0]}, planner={"CUSTOMERS": cu["customers"]})
        write(f"v3_forecast_{c}", f"Forecast error ({c}): the planner sees U_ref + dU_plan from theta + dth_plan; "
              "the vehicle flies in the true wind.", c,
              {"U_ref": [8.0, 12.0, 16.0], "theta": THETA, "customer": [0, 1, 2, 3], "planner": ["cutoff", "wind"],
               "controller": ["dob"], "seed": [0], "dU_plan": [-4.0, -2.0, 0.0, 2.0, 4.0],
               "dth_plan": [-30.0, 0.0, 30.0]})
    for c in ["procedural"] + REAL:
        write(f"v3_seeds_{c}", f"Gust seeds 2-4 in strong wind ({c}); with seeds 0-1 of the main campaigns this "
              "gives five seeds per cell.", c,
              {"U_ref": [12.0, 16.0], "theta": THETA, "customer": [0, 1, 2, 3], "planner": ALL,
               "controller": ["dob"], "seed": [2, 3, 4]})
        write(f"v3_speedcap_{c}", f"Ground speed capped at 14 m/s ({c}), as under a 15 m/s regulatory limit.", c,
              {"U_ref": [4.0, 8.0, 12.0, 16.0], "theta": THETA, "customer": [0, 1, 2, 3],
               "planner": ["energy0", "wind"], "controller": ["dob"], "seed": [0, 1]},
              planner={"SPEEDS_FINE": [4.0, 6.0, 8.0, 10.0, 12.0, 14.0]})
        for t in [20.0, 22.5, 25.0, 27.5, 32.5]:
            write(f"v3_margin_{c}_{t:g}", f"Tilt-margin sweep ({c}): flown in the QES-Winds field, planned on the "
                  f"analytical map, TILT_LIMIT = {t:g} deg.", c,
                  {"U_ref": [12.0, 16.0], "theta": THETA, "customer": [0, 1, 2, 3], "controller": ["dob"],
                   "seed": [0], "planner": ["wind"]},
                  planner={"TILT_LIMIT": t}, qes={"dir": f"../results/qes/{c}", "fly": "qes", "plan": "analytic"})


if __name__ == "__main__":
    main()
