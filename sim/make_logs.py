"""Run a few representative missions with full logging (for time-series figures)."""
import pickle
from multiprocessing import Pool

from mission import run_mission

CASES = [(12.0, 180.0, 0, "shortest", "pid"), (12.0, 180.0, 0, "shortest", "dob"),
         (12.0, 180.0, 0, "wind", "pid"), (12.0, 180.0, 0, "wind", "dob"),
         (12.0, 0.0, 0, "shortest", "dob"), (12.0, 0.0, 0, "wind", "dob")]


def one(c):
    r = run_mission(*c, seed=0, keep_log=True)
    return c, r


if __name__ == "__main__":
    with Pool(len(CASES)) as p:
        out = dict(p.map(one, CASES))
    with open("../results/example_logs.pkl", "wb") as f:
        pickle.dump(out, f)
    print("saved", len(out))
