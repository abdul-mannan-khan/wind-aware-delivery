"""Parametric urban wind field with street-canyon channelling, plus Dryden-type gusts.

This is an engineering (semi-empirical) model, not CFD. Its ingredients follow the urban
boundary-layer literature qualitatively:
  * power-law profile above the roofs,               U(z) = U_ref (z / z_ref)^alpha
  * exponential attenuation inside the canopy,        U(z) = U(H) exp(a (z/H - 1))
  * channelling along streets and corridors (avenues): flow turns into the street axis,
    with a speed-up on wide avenues and a suppressed cross-street component,
  * sheltering / wakes downwind of tall buildings: lower mean speed, higher turbulence.
The wind direction theta is the direction the air moves TOWARD (0 = +x/east, 90 deg = +y/north).
"""
import numpy as np
from scipy.ndimage import gaussian_filter, map_coordinates, maximum_filter


class WindField:
    def __init__(self, city, U_ref=8.0, theta_deg=0.0, z_ref=60.0, alpha=0.30, a_canopy=1.5,
                 k_corridor=0.3, k_cross=0.25, k_shelter=0.5, res=4.0, dz=5.0, z_max=100.0):
        self.city, self.U_ref, self.theta = city, U_ref, np.deg2rad(theta_deg)
        self.res, self.dz = res, dz
        self.z_ref, self.alpha = z_ref, alpha
        xs, Hb = city.height_map(res)
        general = hasattr(city, "street_axis")              # real city: streets in any direction
        if general:
            _, canyon, axis, width, aven = city.street_axis(res)
        else:
            _, cls, width, aven = city.street_class(res)
        n = len(xs)
        zs = np.arange(0.0, z_max + dz, dz)
        self.xs, self.zs = xs, zs
        d = np.array([np.cos(self.theta), np.sin(self.theta)])

        H_loc = np.maximum(gaussian_filter(Hb, sigma=30.0 / res), 8.0)     # local canopy height
        H_can = maximum_filter(Hb, size=int(round(30.0 / res)) | 1)        # height of canyon walls

        U = np.zeros((n, n, len(zs), 3))
        TI = np.zeros((n, n, len(zs)))
        for k, z in enumerate(zs):
            zz = max(z, 1.0)
            u_fs = U_ref * (np.maximum(zz, H_loc) / z_ref) ** alpha
            u_c = np.where(zz >= H_loc, u_fs, u_fs * np.exp(a_canopy * (zz / H_loc - 1.0)))
            vx, vy = u_c * d[0], u_c * d[1]
            # channelled flow inside the canyons (below the wall height)
            gain = 1.0 + k_corridor * aven
            if general:
                # along-street component amplified on avenues, cross-street component suppressed
                along = d[0] * axis[..., 0] + d[1] * axis[..., 1]
                cvx = np.where(canyon, u_c * (along * gain * axis[..., 0] + k_cross * (d[0] - along * axis[..., 0])), vx)
                cvy = np.where(canyon, u_c * (along * gain * axis[..., 1] + k_cross * (d[1] - along * axis[..., 1])), vy)
            else:
                ch_x = cls == 1
                ch_y = cls == 2
                cvx = np.where(ch_x, u_c * d[0] * gain, np.where(ch_y, u_c * d[0] * k_cross, vx))
                cvy = np.where(ch_y, u_c * d[1] * gain, np.where(ch_x, u_c * d[1] * k_cross, vy))
            w = 1.0 / (1.0 + np.exp(-(zz - H_can) / 4.0))     # 0 deep in canyon, 1 above roofs
            vx = w * vx + (1 - w) * cvx
            vy = w * vy + (1 - w) * cvy
            # sheltering by upwind buildings taller than z
            S = np.zeros((n, n))
            for s in np.arange(8.0, 100.0, 8.0):
                shift = -s * d / res
                Hup = map_coordinates(Hb, [np.arange(n)[:, None] + shift[0] + 0 * np.arange(n)[None, :],
                                           np.arange(n)[None, :] + shift[1] + 0 * np.arange(n)[:, None]],
                                      order=0, mode="nearest")
                S = np.maximum(S, (Hup > zz) * np.exp(-s / (0.8 * np.maximum(Hup, 1.0))))
            vx *= 1.0 - k_shelter * S
            vy *= 1.0 - k_shelter * S
            ti = np.where(zz >= H_can, 0.12, 0.22) + 0.25 * S
            solid = Hb > zz
            vx[solid] = vy[solid] = 0.0
            ti[solid] = 0.0
            vx = gaussian_filter(vx, 1.2)
            vy = gaussian_filter(vy, 1.2)
            ti = gaussian_filter(ti, 1.2)
            vx[solid] = vy[solid] = 0.0
            U[:, :, k, 0], U[:, :, k, 1] = vx, vy
            TI[:, :, k] = ti
        self.U, self.TI, self.Hb = U, TI, Hb

    # -- queries -------------------------------------------------------------------------
    def _idx(self, p):
        fx = np.clip(p[0] / self.res - 0.5, 0, len(self.xs) - 1.001)
        fy = np.clip(p[1] / self.res - 0.5, 0, len(self.xs) - 1.001)
        fz = np.clip(p[2] / self.dz, 0, len(self.zs) - 1.001)
        i, j, k = int(fx), int(fy), int(fz)
        return i, j, k, fx - i, fy - j, fz - k

    def mean(self, p):
        """Trilinear interpolation of mean wind (3,) and turbulence intensity at point p."""
        i, j, k, a, b, c = self._idx(p)
        U, T = self.U, self.TI
        wts = ((1 - a) * (1 - b) * (1 - c), a * (1 - b) * (1 - c), (1 - a) * b * (1 - c), a * b * (1 - c),
               (1 - a) * (1 - b) * c, a * (1 - b) * c, (1 - a) * b * c, a * b * c)
        idx = ((i, j, k), (i + 1, j, k), (i, j + 1, k), (i + 1, j + 1, k),
               (i, j, k + 1), (i + 1, j, k + 1), (i, j + 1, k + 1), (i + 1, j + 1, k + 1))
        u = np.zeros(3)
        ti = 0.0
        for w, (ii, jj, kk) in zip(wts, idx):
            u += w * U[ii, jj, kk]
            ti += w * T[ii, jj, kk]
        return u, ti

    def layer(self, z):
        """Horizontal slice (nearest level) for plotting and planning: U (n,n,3), TI (n,n)."""
        k = int(round(np.clip(z / self.dz, 0, len(self.zs) - 1)))
        return self.U[:, :, k], self.TI[:, :, k]


class DrydenGust:
    """First-order Dryden-type (Gauss-Markov) gust per axis, with intensity scheduled by the local
    turbulence intensity: sigma = TI * |U_mean| (plus a small floor)."""

    def __init__(self, rng, L=(30.0, 30.0, 12.0), floor=0.2):
        self.rng, self.L, self.floor = rng, np.array(L), floor
        self.g = np.zeros(3)

    def step(self, dt, u_mean, ti, airspeed):
        sigma = ti * np.linalg.norm(u_mean) + self.floor
        sig = np.array([sigma, sigma, 0.6 * sigma])
        V = max(airspeed, 2.0)
        beta = V / self.L
        self.g += -beta * self.g * dt + sig * np.sqrt(2 * beta * dt) * self.rng.standard_normal(3)
        return self.g
