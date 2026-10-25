"""Demand-weighted delivery points for a real city raster (OSM).

Each candidate is an open street cell at least 7 m from any wall (the rule used for the depot and the four
standard customers). Its demand weight is the building floor area within 40 m (raster volume / 3.5 m storey
height), so streets lined by tall, dense blocks receive more orders. N points are drawn with probability
proportional to the weight, at least 40 m apart and at least 100 m from the depot. The weights are kept so that
the operations analysis can draw orders in proportion to them.

  python3 make_customers.py ../data/cities/london.npz 40 ../configs/v3/customers_london.yaml [--seed 0]
"""
import argparse

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt, uniform_filter

from osm_city import OSMCity

STOREY_M, RADIUS_M, MIN_SEP_M, MIN_DEPOT_M, MIN_CLEAR_M, LATTICE_M = 3.5, 40.0, 40.0, 100.0, 7.0, 4.0


def customers(city, n, seed=0):
    r = city.res1
    H = city.H1
    vol = np.where(H > 2.0, H, 0.0)
    k = int(round(2 * RADIUS_M / r)) | 1
    floor = uniform_filter(vol, size=k, mode="constant") * k * k * r * r / STOREY_M       # m^2 within the window
    dist = distance_transform_edt(H <= 2.0) * r
    step = max(int(round(LATTICE_M / r)), 1)
    ij = np.argwhere(dist >= MIN_CLEAR_M)
    ij = ij[(ij[:, 0] % step == 0) & (ij[:, 1] % step == 0)]
    xy = (ij + 0.5) * r
    w = floor[ij[:, 0], ij[:, 1]]
    keep = (np.hypot(*(xy - city.depot).T) >= MIN_DEPOT_M) & (w > 0)
    xy, w = xy[keep], w[keep]
    rng = np.random.default_rng(seed)
    chosen, wts = [], []
    p = w / w.sum()
    while len(chosen) < n:
        i = rng.choice(len(xy), p=p)
        if all(np.hypot(*(xy[i] - c)) >= MIN_SEP_M for c in chosen):
            chosen.append(xy[i])
            wts.append(w[i])
        p[i] = 0.0
        if p.sum() <= 0:
            raise RuntimeError(f"only {len(chosen)} points satisfy the spacing rules")
        p = p / p.sum()
    return np.array(chosen), np.array(wts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("osm")
    ap.add_argument("n", type=int)
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    city = OSMCity(a.osm)
    c, w = customers(city, a.n, a.seed)
    doc = dict(city=city.name, seed=a.seed, rule=dict(storey_m=STOREY_M, radius_m=RADIUS_M, min_sep_m=MIN_SEP_M,
                                                      min_depot_m=MIN_DEPOT_M, min_clear_m=MIN_CLEAR_M),
               depot=[round(float(v), 1) for v in city.depot],
               customers=[[round(float(x), 1), round(float(y), 1)] for x, y in c],
               demand_weight=[round(float(v / w.sum()), 5) for v in w])
    with open(a.out, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    d = np.hypot(*(c - city.depot).T)
    print(f"{city.name}: {len(c)} customers, depot distance {d.min():.0f}-{d.max():.0f} m (median {np.median(d):.0f})")


if __name__ == "__main__":
    main()
