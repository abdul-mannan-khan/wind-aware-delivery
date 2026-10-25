"""6-DOF quadrotor (X configuration) with first-order motors, rotor drag and body drag,
a momentum-theory power model, a geometric SE(3) tracking controller and an optional
disturbance-force observer (DOB)."""
import numpy as np

G = 9.81
RHO = 1.225
E3 = np.array([0.0, 0.0, 1.0])


class Params:
    m_frame = 1.6            # kg, airframe + battery
    m_payload = 0.6          # kg, parcel
    J = np.diag([0.030, 0.030, 0.050])
    arm = 0.25               # m
    k_m = 0.016              # yaw torque / thrust (m)
    T_max = 12.0             # N per rotor
    tau_m = 0.03             # s, motor time constant
    r_prop = 0.127           # m (10 in propeller)
    d_rotor = np.array([0.30, 0.30, 0.05])   # kg/s, linear rotor drag (body frame)
    CdA = 0.05               # m^2, body + parcel quadratic drag area
    kappa = 1.15             # induced power factor
    eta = 0.70               # motor+ESC efficiency
    P_avionics = 15.0        # W
    battery_Wh = 74.0 * 0.8  # usable energy (4S 5 Ah, 80 %)

    @property
    def A_rotor(self):
        return np.pi * self.r_prop ** 2


def hat(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def aero_force(R, v_air, P):
    """Aerodynamic force (world frame) for vehicle velocity relative to the air v_air."""
    vb = R.T @ v_air
    f_rotor = -R @ (P.d_rotor * vb)
    f_body = -0.5 * RHO * P.CdA * np.linalg.norm(v_air) * v_air
    return f_rotor + f_body


def rotor_power(T, v_air, zb, P):
    """Electrical power of four rotors with thrusts T (4,), using momentum theory with Glauert's
    forward-flight induced velocity. v_air: vehicle velocity relative to air (world), zb: thrust axis."""
    A = P.A_rotor
    V_perp = float(v_air @ zb)                       # air flowing down through the disc (climb-like)
    V_par = float(np.linalg.norm(v_air - V_perp * zb))
    Ptot = 0.0
    for Ti in T:
        Ti = max(Ti, 1e-3)
        vh2 = Ti / (2 * RHO * A)
        vi = np.sqrt(vh2)
        for _ in range(8):                           # fixed-point / damped iteration
            vi_new = vh2 / np.sqrt(V_par ** 2 + (V_perp + vi) ** 2)
            vi = 0.5 * vi + 0.5 * vi_new
        Ptot += max(P.kappa * Ti * vi + Ti * V_perp, 0.0)
    return Ptot / P.eta + P.P_avionics


def trim_power(v_ground, wind, mass, P, return_tilt=False):
    """Steady-level-flight power for ground velocity v_ground in mean wind `wind` (planner model).
    With return_tilt=True also returns the steady tilt angle (deg) needed to hold that velocity."""
    v_air = v_ground - wind
    R = np.eye(3)
    for _ in range(3):                               # rotor drag depends on attitude: iterate
        F = mass * G * E3 - aero_force(R, v_air, P)
        zb = F / np.linalg.norm(F)
        # rotation with body z = zb, minimal from e3
        c = np.cross(E3, zb)
        s, co = np.linalg.norm(c), zb[2]
        R = np.eye(3) if s < 1e-9 else np.eye(3) + hat(c) + hat(c) @ hat(c) * ((1 - co) / s ** 2)
    T = np.full(4, np.linalg.norm(F) / 4)
    Pw = rotor_power(T, v_air, zb, P)
    if return_tilt:
        return Pw, float(np.degrees(np.arccos(np.clip(zb[2], -1, 1))))
    return Pw


class Quadrotor:
    def __init__(self, P=None, p0=(0, 0, 0)):
        self.P = P or Params()
        self.mass = self.P.m_frame + self.P.m_payload
        self.p = np.array(p0, float)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0, 0, 0])
        self.w = np.zeros(3)
        self.T = np.full(4, self.mass * G / 4)
        L = self.P.arm / np.sqrt(2)
        km = self.P.k_m
        # rotors: 1 front-right (CCW), 2 rear-left (CCW), 3 front-left (CW), 4 rear-right (CW)
        self.Mix = np.array([[1, 1, 1, 1],
                             [-L, L, L, -L],
                             [-L, L, -L, L],
                             [km, km, -km, -km]])
        self.Mix_inv = np.linalg.inv(self.Mix)
        self.F_aero = np.zeros(3)

    def drop_payload(self):
        self.mass = self.P.m_frame

    def _deriv(self, s, T_cmd, wind):
        p, v, q, w, T = s[0:3], s[3:6], s[6:10], s[10:13], s[13:17]
        R = quat_to_R(q / np.linalg.norm(q))
        fM = self.Mix @ T
        Fa = aero_force(R, v - wind, self.P)
        a = (fM[0] * R @ E3 + Fa) / self.mass - G * E3
        J = self.P.J
        wdot = np.linalg.solve(J, fM[1:] - np.cross(w, J @ w))
        qw, qx, qy, qz = q
        Om = 0.5 * np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])
        qdot = Om @ w
        Tdot = (T_cmd - T) / self.P.tau_m
        return np.concatenate([v, a, qdot, wdot, Tdot]), Fa

    def step(self, dt, T_cmd, wind):
        s = np.concatenate([self.p, self.v, self.q, self.w, self.T])
        k1, Fa = self._deriv(s, T_cmd, wind)
        k2, _ = self._deriv(s + 0.5 * dt * k1, T_cmd, wind)
        k3, _ = self._deriv(s + 0.5 * dt * k2, T_cmd, wind)
        k4, _ = self._deriv(s + dt * k3, T_cmd, wind)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.p, self.v, self.w, self.T = s[0:3], s[3:6], s[10:13], s[13:17]
        self.q = s[6:10] / np.linalg.norm(s[6:10])
        self.F_aero = Fa

    @property
    def R(self):
        return quat_to_R(self.q)


class GeometricController:
    """Lee-Leok-McClamroch SE(3) tracking controller with an integral term and an optional
    momentum-based disturbance-force observer (DOB) whose estimate is fed forward."""

    def __init__(self, quad, use_dob=False, k_obs=4.0):
        self.quad, self.use_dob, self.k_obs = quad, use_dob, k_obs
        self.kp = np.array([4.0, 4.0, 6.0])
        self.kv = np.array([3.5, 3.5, 4.5])
        self.ki = np.array([0.2, 0.2, 0.4]) if use_dob else np.array([1.0, 1.0, 1.5])
        self.kR = np.array([6.0, 6.0, 3.0])
        self.kw = np.array([0.70, 0.70, 0.50])
        self.max_tilt = np.deg2rad(45)
        self.ei = np.zeros(3)
        self.d_hat = np.zeros(3)
        self.z_obs = None
        self.yaw_d = 0.0

    def observe(self, dt):
        """d_hat = k (m v - z),  z_dot = f R e3 - m g e3 + d_hat   (estimates external force)."""
        qd = self.quad
        m = qd.mass
        if self.z_obs is None:
            self.z_obs = m * qd.v.copy()
        f = qd.T.sum()
        self.z_obs += dt * (f * qd.R @ E3 - m * G * E3 + self.d_hat)
        self.d_hat = self.k_obs * (m * qd.v - self.z_obs)

    def reset_observer(self):
        self.z_obs = None

    def __call__(self, dt, p_d, v_d, a_d):
        qd = self.quad
        m = qd.mass
        if self.use_dob:
            self.observe(dt)
        ep = qd.p - p_d
        ev = qd.v - v_d
        self.ei = np.clip(self.ei + ep * dt, -3.0, 3.0)
        F = m * (-self.kp * ep - self.kv * ev - self.ki * self.ei + G * E3 + a_d)
        if self.use_dob:
            F = F - self.d_hat
        # tilt limit
        Fh = np.linalg.norm(F[:2])
        Fz = max(F[2], 0.3 * m * G)
        if Fh > Fz * np.tan(self.max_tilt):
            F[:2] *= Fz * np.tan(self.max_tilt) / Fh
        F[2] = Fz
        R = qd.R
        f = float(F @ R @ E3)
        b3 = F / np.linalg.norm(F)
        c1 = np.array([np.cos(self.yaw_d), np.sin(self.yaw_d), 0.0])
        b2 = np.cross(b3, c1)
        b2 /= np.linalg.norm(b2)
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        ew = qd.w
        M = -self.kR * eR * np.diag(qd.P.J) / 0.03 - self.kw * ew * np.diag(qd.P.J) / 0.03 \
            + np.cross(qd.w, qd.P.J @ qd.w)
        T_cmd = np.clip(qd.Mix_inv @ np.concatenate([[f], M]), 0.0, qd.P.T_max)
        sat = bool(np.any(T_cmd >= qd.P.T_max - 1e-6) or np.any(T_cmd <= 1e-6))
        return T_cmd, sat
