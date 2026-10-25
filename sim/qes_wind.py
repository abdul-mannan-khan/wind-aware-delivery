"""Mean urban wind from QES-Winds (Bozorgmehr et al., Environ. Model. Softw. 137, 2021), used in place of the
analytical field of wind.py, e.g. as the "true" wind in which missions planned on the analytical map are flown.

QES-Winds is run once per wind direction at a reference speed U_run (qes_tools.py); its diagnostic, mass-consistent
field scales linearly with the inflow speed, so other speeds are obtained by scaling. Turbulence intensity (for the
gusts) is taken from the analytical model of the same geometry, so that only the mean flow differs.
"""
import os

import numpy as np

from wind import WindField


class QESWindField(WindField):
    def __init__(self, city, U_ref=8.0, theta_deg=0.0, qes_dir=".", **kw):
        super().__init__(city, U_ref, theta_deg, **kw)          # grid, TI, building heights
        f = os.path.join(qes_dir, f"qes_th{int(round(theta_deg)) % 360:03d}.npz")
        d = np.load(f)
        if d["U"].shape != self.U.shape:
            raise ValueError(f"{f}: grid {d['U'].shape} does not match the wind grid {self.U.shape}")
        self.U = d["U"] * (U_ref / float(d["U_run"]))
        self.source = f
