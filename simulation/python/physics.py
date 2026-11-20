"""
Plant model: rigid-body TWIP + two DC gear motors (with backlash and
friction) + DRV8833 + LiPo. Integrated with fixed-step RK4.

State vector (13 floats):
  0 X, 1 Y        axle midpoint on the floor [m]
  2 psi           heading (yaw) [rad]
  3 th            pitch, + = leaning forward [rad]
  4 v             forward speed [m/s]
  5 thd, 6 psid   pitch rate, yaw rate [rad/s]
  7 gL, 8 wL      LEFT motor-side angle / rate, referred to the output shaft,
                  relative to the body [rad, rad/s]  (rotor angle = gear * gL)
  9 gR, 10 wR     same for RIGHT
  11 aL, 12 aR    absolute wheel angles about the axle [rad]
Wheel-side relative angle: phi = a - th. The gearbox couples g and phi
through a spring with a dead zone (backlash).
"""
from math import sin, cos, tanh, exp, pi, copysign
from eom_generated import mass_forcing
from params import LIPO_OCV

IX, IY, IPSI, ITH, IV, ITHD, IPSID, IGL, IWL, IGR, IWR, IAL, IAR = range(13)


def lipo_ocv(soc, cells):
    soc = min(max(soc, 0.0), 1.0)
    for (s0, v0), (s1, v1) in zip(LIPO_OCV, LIPO_OCV[1:]):
        if soc <= s1:
            return cells * (v0 + (v1 - v0) * (soc - s0) / (s1 - s0))
    return cells * LIPO_OCV[-1][1]


class Motor:
    """One N20 channel, all quantities referred to the OUTPUT shaft."""

    def __init__(self, mp, rds_on, kt_scale=1.0, r_scale=1.0, fric_scale=1.0):
        self.G = mp.gear
        self.Kt = mp.Kt * kt_scale
        self.Ke = mp.Ke * kt_scale
        self.R = mp.R * r_scale + rds_on
        self.eta = mp.eta
        self.J = mp.J_out
        self.Tc = mp.T_coulomb_out * fric_scale
        self.Ts = self.Tc * mp.stiction_ratio
        self.b = mp.b_visc_out * fric_scale
        self.half_gap = 0.5 * mp.backlash_deg * pi / 180
        self.k = mp.gear_stiffness
        self.c = 2 * mp.gear_damping_ratio * (self.k * self.J * 0.5) ** 0.5

    def current(self, duty, vbat, w, enabled):
        """DRV8833 driven in slow-decay (brake) PWM: average V = duty * Vbat."""
        if not enabled:
            return 0.0                    # coast: bridge off (Hi-Z)
        return (duty * vbat - self.Ke * self.G * w) / self.R

    def friction(self, w):
        ws = 0.1                           # rad/s smoothing (keeps RK4 stable)
        return (self.Tc * tanh(w / ws)
                + (self.Ts - self.Tc) * exp(-(w / 0.3) ** 2) * tanh(w / ws)
                + self.b * w)

    def gear_torque(self, delta, ddelta):
        """Torque transmitted to the wheel; delta = motor-side minus wheel-side angle."""
        h = self.half_gap
        if delta > h:
            return max(0.0, self.k * (delta - h) + self.c * ddelta)
        if delta < -h:
            return min(0.0, self.k * (delta + h) + self.c * ddelta)
        return 0.0


class Plant:
    def __init__(self, p):
        self.p = p
        mp = p.motor
        self.mL = Motor(mp, p.power.drv_rds_on)
        self.mR = Motor(mp, p.power.drv_rds_on, mp.mismatch_kt, mp.mismatch_R, mp.mismatch_fric)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self, th0=0.0, soc=None):
        self.x = [0.0] * 13
        self.x[ITH] = th0
        self.soc = self.p.power.soc0 if soc is None else soc
        self.vbat = lipo_ocv(self.soc, self.p.power.cells)
        self.ibat = 0.0
        self.iL = self.iR = 0.0
        self.TL = self.TR = 0.0
        self.acc = (0.0, 0.0, 0.0)        # dv, dthd, dpsid of the last step
        self.contact = 0                  # +1 / -1 when lying on the floor
        self.ocp = False
        self.F_ext = 0.0
        self.t = 0.0
        self.energy_Wh = 0.0

    # ------------------------------------------------------------------
    def deriv(self, x, dL, dR, en, Fpush, push_h):
        p = self.p
        th, v, thd, psid = x[ITH], x[IV], x[ITHD], x[IPSID]
        r, d = p.r, p.d
        # wheel-side relative angles / rates
        wl_abs = (v - 0.5 * d * psid) / r
        wr_abs = (v + 0.5 * d * psid) / r
        phiL, phiR = x[IAL] - th, x[IAR] - th
        phidL, phidR = wl_abs - thd, wr_abs - thd
        # motors
        mL, mR = self.mL, self.mR
        iL = mL.current(dL, self.vbat, x[IWL], en)
        iR = mR.current(dR, self.vbat, x[IWR], en)
        TgL = mL.gear_torque(x[IGL] - phiL, x[IWL] - phidL)
        TgR = mR.gear_torque(x[IGR] - phiR, x[IWR] - phidR)
        dwL = (mL.eta * mL.G * mL.Kt * iL - mL.friction(x[IWL]) - TgL) / mL.J
        dwR = (mR.eta * mR.G * mR.Kt * iR - mR.friction(x[IWR]) - TgR) / mR.J
        # rigid body
        (M0, M1, M2), (F0, F1, F2) = mass_forcing(th, v, thd, psid, TgL, TgR, p)
        F0 = list((F0, F1, F2))
        # rolling resistance + yaw scrub
        N = p.total_mass * p.g
        F0[0] -= p.wheel.c_rr * N * tanh(v / 0.02)
        F0[2] -= 0.5 * N * p.wheel.mu * 0.0015 * tanh(psid / 0.2) * p.d / 2
        # external push at height push_h (body frame z) on the centre line
        if Fpush:
            F0[0] += Fpush
            F0[1] += Fpush * (push_h * cos(th))
        # floor contact: body lies on the floor -> drag + no further rotation
        if self.contact:
            F0[0] -= 2.0 * v * p.total_mass * 5
        a, b_, c_, dd = M0[0], M0[1], M1[0], M1[1]
        det = a * dd - b_ * c_
        dv = (F0[0] * dd - b_ * F0[1]) / det
        dthd = (a * F0[1] - c_ * F0[0]) / det
        dpsid = F0[2] / M2[2]
        if self.contact:
            dthd = 0.0
        return ([v * cos(x[IPSI]), v * sin(x[IPSI]), psid, thd, dv, dthd, dpsid,
                 x[IWL], dwL, x[IWR], dwR, wl_abs, wr_abs],
                (iL, iR, TgL, TgR, dv, dthd, dpsid))

    # ------------------------------------------------------------------
    def step(self, dt, dL, dR, enabled=True, Fpush=0.0, push_h=0.07):
        x = self.x
        k1, aux = self.deriv(x, dL, dR, enabled, Fpush, push_h)
        x2 = [xi + 0.5 * dt * ki for xi, ki in zip(x, k1)]
        k2, _ = self.deriv(x2, dL, dR, enabled, Fpush, push_h)
        x3 = [xi + 0.5 * dt * ki for xi, ki in zip(x, k2)]
        k3, _ = self.deriv(x3, dL, dR, enabled, Fpush, push_h)
        x4 = [xi + dt * ki for xi, ki in zip(x, k3)]
        k4, aux4 = self.deriv(x4, dL, dR, enabled, Fpush, push_h)
        self.x = x = [xi + dt / 6 * (a + 2 * b + 2 * c + d)
                      for xi, a, b, c, d in zip(x, k1, k2, k3, k4)]
        self.iL, self.iR, self.TL, self.TR = aux[0], aux[1], aux[2], aux[3]
        self.F_ext = Fpush
        self.acc = aux[4:7]
        # floor contact (inelastic, with a little bounce)
        p = self.p
        if x[ITH] >= p.fall_fwd:
            x[ITH] = p.fall_fwd
            if x[ITHD] > 0:
                x[ITHD] = -0.15 * x[ITHD]
            self.contact = +1 if abs(x[ITHD]) < 0.2 else 0
        elif x[ITH] <= p.fall_back:
            x[ITH] = p.fall_back
            if x[ITHD] < 0:
                x[ITHD] = -0.15 * x[ITHD]
            self.contact = -1 if abs(x[ITHD]) < 0.2 else 0
        else:
            self.contact = 0
        # power
        pw = p.power
        iL, iR = self.iL, self.iR
        if enabled and (abs(iL) > pw.drv_ocp or abs(iR) > pw.drv_ocp):
            self.ocp = True
        i_motor = (abs(dL) * iL * copysign(1, dL) + abs(dR) * iR * copysign(1, dR)) if enabled else 0.0
        ocv = lipo_ocv(self.soc, pw.cells)
        i_logic = pw.logic_power_W / max(self.vbat, 3.0)
        self.ibat = i_motor + i_logic
        self.vbat = ocv - self.ibat * (pw.R_internal + pw.R_wiring)
        self.soc -= self.ibat * dt / (3.6 * pw.capacity_mAh)
        self.energy_Wh += self.vbat * self.ibat * dt / 3600
        self.t += dt

    # ------------------------------------------------------------------
    def body_point_accel_A(self, px, pz):
        """Acceleration of a body point (centre line) in the heading frame A."""
        x = self.x
        th, thd, psid, v = x[ITH], x[ITHD], x[IPSID], x[IV]
        dv, dthd, dpsid = self.acc
        s, c = sin(th), cos(th)
        # point position in A
        rx, rz = px * c + pz * s, -px * s + pz * c
        ry = 0.0
        # axle acceleration in A: dv along x, centripetal v*psid along y
        ax, ay, az = dv, v * psid, 0.0
        # angular velocity / acceleration in A components
        wx, wy, wz = 0.0, thd, psid
        alx, aly, alz = -psid * thd, dthd, dpsid
        # alpha x r
        ax += aly * rz - alz * ry
        ay += alz * rx - alx * rz
        az += alx * ry - aly * rx
        # w x (w x r)
        cx_, cy_, cz_ = wy * rz - wz * ry, wz * rx - wx * rz, wx * ry - wy * rx
        ax += wy * cz_ - wz * cy_
        ay += wz * cx_ - wx * cz_
        az += wx * cy_ - wy * cx_
        return ax, ay, az

    def traction_ratio(self):
        """Needed horizontal force at the tyres / available friction (no-slip check)."""
        p = self.p
        if self.contact:
            return 0.0
        x = self.x
        th, thd = x[ITH], x[ITHD]
        dv, dthd, _ = self.acc
        s, c = sin(th), cos(th)
        # horizontal acceleration of body COM
        acom = dv + p.l * (c * dthd - s * thd * thd)
        F = p.mb * acom + 2 * p.mw * dv - self.F_ext
        return abs(F) / (p.wheel.mu * p.total_mass * p.g)
