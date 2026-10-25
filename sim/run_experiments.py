"""Monte-Carlo campaign: planners x controllers x wind speed x wind direction x customer x seed.

Usage:
    python run_experiments.py --config ../configs/quick.yaml          # any experiment described by a YAML file
    python run_experiments.py --config ../configs/procedural_full.yaml    # the 1,536-mission campaign in the procedural city
    python run_experiments.py --quick                                  # 12-mission smoke test (no config needed)

Options: --workers N (default: the usable CPU cores), --out FILE (default: ../results/experiments/<name>/campaign.csv),
--dry-run (only count the missions). The config file lists the values of every swept factor (section `grid`)
and, optionally, changes to the vehicle, planner, wind model, gusts and city (sections `vehicle`, `planner`,
`wind`, `gust`, `city`). See configs/README.md.
"""
import argparse
import csv
import itertools
import os
import shutil
import time
from multiprocessing import Pool

import numpy as np
import yaml

import mission
from city import City
from mission import run_mission
from quadrotor import Params
from wind import WindField

# the default campaign; a config file overrides any of these
DEFAULT_GRID = dict(U_ref=[4.0, 8.0, 12.0, 16.0], theta=[0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
                    customer=[0, 1, 2, 3], planner=["shortest", "energy0", "wind"], controller=["pid", "dob"],
                    seed=[0, 1])
QUICK_GRID = dict(U_ref=[12.0], theta=[0.0, 180.0], customer=[0], planner=["shortest", "energy0", "wind"],
                  controller=["pid", "dob"], seed=[0])
PLANNER_KEYS = ("ALTITUDES", "SPEEDS", "TILT_LIMIT", "Z_NOMINAL", "V_NOMINAL", "Z_DROP", "V_CLIMB", "V_DESCEND",
                "K_OBS", "CUSTOMERS", "DEPOT", "WIND_CUTOFF", "CLEAR_H", "SPEED_SCHEDULE", "SPEEDS_FINE")

_CFG = {}
_CACHE = {}


def available_cpus():
    """CPU cores this process may really use: cloud containers often see all of the host's cores (os.cpu_count)
    but are limited by a cgroup quota."""
    n = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else os.cpu_count()
    try:
        q, p = open("/sys/fs/cgroup/cpu.max").read().split()[:2]                 # cgroup v2
        if q != "max":
            n = min(n, max(1, int(int(q) / int(p))))
    except (OSError, ValueError):
        try:                                                                      # cgroup v1
            q = int(open("/sys/fs/cgroup/cpu/cpu.cfs_quota_us").read())
            p = int(open("/sys/fs/cgroup/cpu/cpu.cfs_period_us").read())
            if q > 0:
                n = min(n, max(1, q // p))
        except (OSError, ValueError):
            pass
    return n


def apply_config(cfg):
    """Install the config in this process (also called in every worker)."""
    global _CFG
    _CFG = cfg
    for k, v in (cfg.get("planner") or {}).items():
        if k not in PLANNER_KEYS:
            raise KeyError(f"unknown planner setting '{k}'; allowed: {', '.join(PLANNER_KEYS)}")
        if k == "CUSTOMERS":
            v = [np.array(c, float) for c in v]
        elif k == "DEPOT":
            v = np.array(v, float)
        elif isinstance(v, bool):
            pass
        elif isinstance(v, list):
            v = tuple(float(x) for x in v)
        setattr(mission, k, v)
    mission.GUST = dict(cfg.get("gust") or {})
    _CACHE.clear()


def make_city(spec):
    """Procedural city (City kwargs) or a real city from OpenStreetMap (`osm: path/to/city.npz`). A real city
    brings its own depot and customers unless the config sets planner DEPOT / CUSTOMERS."""
    spec = dict(spec or {})
    if "osm" not in spec:
        return City(**spec)
    from osm_city import OSMCity
    city = OSMCity(**spec)
    pl = _CFG.get("planner") or {}
    if "DEPOT" not in pl:
        mission.DEPOT = city.depot
    if "CUSTOMERS" not in pl:
        mission.CUSTOMERS = list(city.customers)
    return city


def make_params():
    P = Params()
    for k, v in (_CFG.get("vehicle") or {}).items():
        if not hasattr(Params, k):
            raise KeyError(f"unknown vehicle parameter '{k}' (see sim/quadrotor.py, class Params)")
        setattr(P, k, np.array(v, float) if isinstance(v, list) else float(v))
    return P


def job(args):
    U, th, cust, planner, ctrl, seed = args[:6]
    dU, dth = (args[6], args[7]) if len(args) > 6 else (0.0, 0.0)   # forecast error: plan in (U + dU, theta + dth)
    if "city" not in _CACHE:
        _CACHE["city"] = make_city(_CFG.get("city"))
        _CACHE["P"] = make_params()
    city = _CACHE["city"]
    key = (U, th)
    if key not in _CACHE:
        for k in [k for k in _CACHE if isinstance(k, tuple)]:
            del _CACHE[k]                    # keep one wind field per worker in memory
        _CACHE[key] = WindField(city, U, th, **(_CFG.get("wind") or {}))
        q = _CFG.get("qes")
        if q:   # QES-Winds mean field (same turbulence model); the vehicle flies in it
            from qes_wind import QESWindField
            _CACHE[("qes",) + key] = QESWindField(city, U, th, q["dir"], **(_CFG.get("wind") or {}))
    fly = plan = _CACHE[key]
    q = _CFG.get("qes")
    if q:
        fly = _CACHE[("qes",) + key] if q.get("fly", "qes") == "qes" else _CACHE[key]
        plan = _CACHE[("qes",) + key] if q.get("plan", "analytic") == "qes" else _CACHE[key]
    if dU or dth:   # the planner sees a forecast that differs from the wind the vehicle meets
        if q:
            raise ValueError("forecast offsets (dU_plan, dth_plan) cannot be combined with a qes section")
        pkey = ("plan", max(U + dU, 0.5), (th + dth) % 360.0)
        if pkey not in _CACHE:
            for k in [k for k in _CACHE if isinstance(k, tuple) and k[0] == "plan"]:
                del _CACHE[k]
            _CACHE[pkey] = WindField(city, pkey[1], pkey[2], **(_CFG.get("wind") or {}))
        plan = _CACHE[pkey]
    try:
        r = run_mission(U, th, cust, planner, ctrl, seed, city=city, wind=fly, P=_CACHE["P"], plan_wind=plan)
    except Exception as e:  # keep the campaign going; record the failure
        r = dict(U_ref=U, theta=th, customer=cust, planner=planner, controller=ctrl, seed=seed,
                 success=False, error=repr(e))
    if len(args) > 6:
        r.update(dU_plan=dU, dth_plan=dth)
    return r


def load_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    unknown = set(cfg) - {"name", "description", "grid", "vehicle", "planner", "wind", "gust", "city", "qes"}
    if unknown:
        raise KeyError(f"unknown section(s) in {path}: {sorted(unknown)}")
    cfg.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="YAML experiment file (see ../configs)")
    ap.add_argument("--quick", action="store_true", help="12-mission smoke test")
    ap.add_argument("--workers", type=int, default=available_cpus(), help="parallel missions (default: usable CPU cores)")
    ap.add_argument("--out", help="output CSV (default ../results/experiments/<name>/campaign.csv)")
    ap.add_argument("--dry-run", action="store_true", help="only print the number of missions")
    a = ap.parse_args()

    cfg = load_config(a.config) if a.config else {"name": "quick" if a.quick else "procedural_full"}
    grid = dict(QUICK_GRID if a.quick else DEFAULT_GRID)
    for k, v in (cfg.get("grid") or {}).items():
        if k not in grid and k not in ("dU_plan", "dth_plan"):
            raise KeyError(f"unknown grid factor '{k}'; allowed: {', '.join(grid)}, dU_plan, dth_plan")
        grid[k] = v if isinstance(v, list) else [v]
    apply_config(cfg)
    n_cust = len(mission.CUSTOMERS)
    if any(c >= n_cust for c in grid["customer"]):
        raise ValueError(f"customer index out of range: there are {n_cust} customers (0..{n_cust - 1})")

    factors = [[float(u) for u in grid["U_ref"]], [float(t) for t in grid["theta"]],
               grid["customer"], grid["planner"], grid["controller"], grid["seed"]]
    if "dU_plan" in grid or "dth_plan" in grid:   # optional forecast-error factors (m/s, deg)
        factors += [[float(v) for v in grid.get("dU_plan", [0.0])], [float(v) for v in grid.get("dth_plan", [0.0])]]
    jobs = list(itertools.product(*factors))
    out = a.out or os.path.join("..", "results", "experiments", cfg["name"], "campaign.csv")
    print(f"experiment '{cfg['name']}': {len(jobs)} missions on {a.workers} workers -> {out}", flush=True)
    if a.dry_run:
        return
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    if a.config:   # keep the exact settings next to the results
        shutil.copy(a.config, os.path.join(os.path.dirname(os.path.abspath(out)), "config.yaml"))
    t0 = time.time()
    rows = []
    # same (U, theta) jobs are adjacent so that each worker reuses its wind field
    with Pool(a.workers, initializer=apply_config, initargs=(cfg,)) as pool:
        for i, r in enumerate(pool.imap_unordered(job, jobs, chunksize=2)):
            rows.append(r)
            if i % 20 == 0 or i == len(jobs) - 1:
                el = time.time() - t0
                eta = el / (i + 1) * (len(jobs) - i - 1)
                print(f"{i + 1}/{len(jobs)}  elapsed {el:.0f} s  remaining ~{eta:.0f} s", flush=True)
    keys = sorted({k for r in rows for k in r})
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"done in {time.time() - t0:.0f} s; results in {out}")
    print(f"next: python summarise.py {out}")


if __name__ == "__main__":
    main()
