"""
Linearised model + LQR -> starting gains for the cascade controller.

Linear model about upright, rigid gearbox (no backlash), both motors equal:
    states  x = [position, speed, pitch, pitch rate]   (m, m/s, rad, rad/s)
    input   u = PWM counts (both wheels)
The rotor's reflected inertia J_out is included (it is about as large as the
whole robot's mass reflected to the wheel, so it matters).

Because   pwm = Kp*(angle - target) + Kd*rate,  target = -(Kp_v*v + Ki_v*x)
expands to pwm = Kp*angle + Kd*rate + Kp*Kp_v*v + Kp*Ki_v*x,
a full-state LQR gain maps EXACTLY onto the cascade gains.
"""
import numpy as np
from math import pi
from scipy.linalg import solve_discrete_are, expm


def linear_model(p, vbat=7.4):
    mo = p.motor
    Rt = mo.R + p.power.drv_rds_on
    Jm = mo.J_out
    r = p.r
    M00 = 2 * p.Iwa / r**2 + p.mb + 2 * p.mw + 2 * Jm / r**2
    M01 = p.mb * p.l - 2 * Jm / r
    M11 = p.Iyy + p.mb * p.l**2 + 2 * Jm
    M = np.array([[M00, M01], [M01, M11]])
    cu = 2 * mo.eta * mo.gear * mo.Kt * vbat / (255 * Rt)       # N m per PWM count
    cw = 2 * (mo.eta * mo.gear**2 * mo.Kt * mo.Ke / Rt + mo.b_visc_out)
    # generalised forces:  [T/r, mb g l th - T],  T = cu*u - cw*(v/r - thd)
    Bt = np.array([1 / r, -1.0])
    Minv = np.linalg.inv(M)
    # d/dt [v, thd] = Minv @ (Bt*T + [0, mb g l th])
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))
    A[0, 1] = 1
    A[2, 3] = 1
    grav = Minv @ np.array([0, p.mb * p.g * p.l])
    dT_dv, dT_dthd = -cw / r, cw
    col_v = Minv @ (Bt * dT_dv)
    col_thd = Minv @ (Bt * dT_dthd)
    col_u = Minv @ (Bt * cu)
    A[1, 1], A[3, 1] = col_v
    A[1, 2], A[3, 2] = grav
    A[1, 3], A[3, 3] = col_thd
    B[1, 0], B[3, 0] = col_u
    return A, B


def c2d(A, B, dt):
    n = A.shape[0]
    Mx = np.zeros((n + 1, n + 1))
    Mx[:n, :n] = A
    Mx[:n, n:] = B
    E = expm(Mx * dt)
    return E[:n, :n], E[:n, n:]


def lqr_gains(p, q=(60.0, 30.0, 600.0, 2.0), r_u=1 / 120.0**2, vbat=7.4):
    """Discrete LQR at the firmware loop rate. Returns (K, cascade dict, poles)."""
    A, B = linear_model(p, vbat)
    dt = 1.0 / p.timing.control_hz
    Ad, Bd = c2d(A, B, dt)
    Q = np.diag(q)
    R = np.array([[r_u]])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)   # u = -K x
    k1, k2, k3, k4 = -K[0]
    d2r = pi / 180
    kp = k3 * d2r
    kd = k4 * d2r
    cascade = dict(kp=kp, kd=kd, kp_v=k2 / kp, ki_v=k1 / kp)
    poles = np.linalg.eigvals(Ad - Bd @ K)
    return K, cascade, poles, np.linalg.eigvals(A)


if __name__ == "__main__":
    from params import make_params
    for sh in (False, True):
        p = make_params(shell=sh)
        K, c, poles, ol = lqr_gains(p)
        print(f"shell={sh}: open-loop poles {np.round(ol.real, 1)}")
        print("  cascade:", {k: round(v, 3) for k, v in c.items()})
        print("  closed-loop |z|:", np.round(np.abs(poles), 3))
