"""
Simulation orchestrator: physics at 2 kHz, firmware at 200 Hz, sensor and
actuator timing like the real board. Used by both app.py (interactive) and
analyze.py (headless).
"""
from collections import deque
from math import pi
from params import make_params
from physics import Plant, ITH, ITHD, IV, IPSID, IX, IY, IPSI
from sensors import MPU6050, Encoder, BatteryADC
from controller import Controller, Gains
from physics import IGL, IGR, IAL, IAR
from lqr import lqr_gains


def default_gains(p=None):
    """Tuned firmware gains (see controller.Gains and analyze.py)."""
    return Gains()


def lqr_starting_gains(p):
    """Pure-theory LQR gains (no backlash/noise): good on paper, chatter with backlash."""
    _, c, _, _ = lqr_gains(p)
    return Gains(kp=round(float(c["kp"]), 1), kd=round(float(c["kd"]), 2),
                 kp_v=round(float(c["kp_v"]), 1), ki_v=round(float(c["ki_v"]), 1))


class Sim:
    LOG_HZ = 200

    def __init__(self, p=None, gains=None, shell=False, log_seconds=8.0, **overrides):
        self.p = p or make_params(shell=shell, **overrides)
        self.gains = gains or default_gains(self.p)
        self.plant = Plant(self.p)
        self.mpu = MPU6050(self.p)
        self.encL = Encoder(self.p, IGL)
        self.encR = Encoder(self.p, IGR)
        self.adc = BatteryADC(self.p)
        self.ctrl = Controller(self.p, self.gains)
        n = int(log_seconds * self.LOG_HZ)
        self.log = {k: deque(maxlen=n) for k in
                    ("t", "th", "est", "tgt", "pwmL", "pwmR", "v", "vcmd", "vbat", "ibat", "x")}
        self.v_cmd = 0.0
        self.yaw_cmd = 0.0
        self.push_F = 0.0
        self.push_until = 0.0
        self.push_h = 0.07
        self.reset()

    def reset(self, th0_deg=3.0, soc=None):
        dt = self.p.timing.physics_dt
        self.plant.reset(th0=0.0, soc=soc)
        self.mpu.reset()
        self.ctrl.reset()
        # setup(): robot held still, 1 s gyro calibration (200 samples)
        samples = []
        for i in range(200):
            for _ in range(int(round(1 / self.p.timing.control_hz / dt))):
                self.mpu.update(self.plant, dt)
                self.plant.t += dt
            samples.append(self.mpu.read(0.0))
        self.ctrl.calibrate(samples)
        # released by hand at a small tilt
        th0 = th0_deg * pi / 180
        self.plant.x[ITH] = th0
        self.plant.x[IAL] = self.plant.x[IAR] = th0   # wheels turn with the body
        self.plant.t = 0.0
        self.next_ctrl = 0.0
        self.pending = None           # (apply_time, pwmL, pwmR, enabled)
        self.duty = (0.0, 0.0, True)
        for q in self.log.values():
            q.clear()
        self.max_traction = 0.0

    def push(self, impulse_Ns, duration=0.05, height=None):
        self.push_F = impulse_Ns / duration
        self.push_until = self.plant.t + duration
        if height is not None:
            self.push_h = height

    def advance(self, seconds):
        p = self.p
        dt = p.timing.physics_dt
        ctrl_dt = 1.0 / p.timing.control_hz
        steps = int(round(seconds / dt))
        plant = self.plant
        for _ in range(steps):
            t = plant.t
            if t >= self.next_ctrl - 1e-9:
                self.next_ctrl += ctrl_dt
                x = plant.x
                raw = self.mpu.read(x[IV])
                cl, cr = self.encL.count(plant), self.encR.count(plant)
                pl, pr, en = self.ctrl.update(raw, cl, cr, self.adc.read(plant.vbat),
                                              self.v_cmd, self.yaw_cmd)
                self.pending = (t + p.timing.output_delay, pl, pr, en)
                self._log(t)
            if self.pending and t >= self.pending[0] - 1e-9:
                _, pl, pr, en = self.pending
                full = 2 ** p.timing.pwm_bits - 1
                self.duty = (pl / full, pr / full, en)
                self.pending = None
            F = self.push_F if t < self.push_until else 0.0
            dL, dR, en = self.duty
            plant.step(dt, dL, dR, en, F, self.push_h)
            self.mpu.update(plant, dt)
            tr = plant.traction_ratio() if plant.t > 0.5 else 0.0
            if tr > self.max_traction:
                self.max_traction = tr

    def _log(self, t):
        x, c, L = self.plant.x, self.ctrl, self.log
        L["t"].append(t)
        L["th"].append(x[ITH] * 180 / pi)
        L["est"].append(c.angle)
        L["tgt"].append(c.target)
        L["pwmL"].append(c.pwmL)
        L["pwmR"].append(c.pwmR)
        L["v"].append(x[IV])
        L["vcmd"].append(c.v_cmd)
        L["vbat"].append(self.plant.vbat)
        L["ibat"].append(self.plant.ibat)
        L["x"].append(x[IX])

    # convenience
    @property
    def fallen(self):
        return self.ctrl.fallen or self.plant.contact != 0
