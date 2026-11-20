"""
The balancing firmware, written the way it will run on the ESP32
(core 1, 200 Hz). It only sees what the real chip sees: raw MPU-6050
registers, encoder counts and a battery ADC reading -- never the true state.

Structure (cascade):
    speed loop  : target lean angle  = Kp_v*(v_cmd - v) + Ki_v*integral(v_cmd - v)
    angle loop  : pwm               = Kp*(angle - target) + Kd*gyro + Ki*integral
    turn loop   : pwm_diff           = Kp_t*(yaw_cmd - yaw_rate) + Ki_t*integral
    left  = pwm - diff,  right = pwm + diff
Units are the firmware's: degrees, deg/s, m/s, PWM counts (0..255).
Port this 1:1 to Arduino: every line maps to a C statement.
"""
from dataclasses import dataclass, asdict
from math import atan2, pi, copysign
from collections import deque


@dataclass
class Gains:
    # Tuned in this simulator (backlash, sensor noise, delays, motor mismatch)
    # and checked with/without shell, 3 deg backlash, low battery, 100 Hz loop.
    kp: float = 55.0          # PWM counts per degree of tilt error
    kd: float = 0.35          # PWM counts per deg/s of pitch rate
    ki: float = 0.0           # PWM counts per degree*second
    kp_v: float = 20.0        # degrees of lean per m/s of speed error
    ki_v: float = 0.9         # degrees of lean per metre of position error
    kp_t: float = 0.3         # PWM counts per deg/s of yaw-rate error
    ki_t: float = 1.0         # PWM counts per degree of heading error
    alpha: float = 0.98       # complementary filter (gyro weight)
    max_lean: float = 10.0    # deg, speed loop output clamp
    deadband: float = 0.0     # PWM counts added to overcome stiction (0: backlash makes it chatter)
    pwm_cap: float = 204.0    # 80 % of 255 (6 V motors on a 8.4 V pack)
    fall_angle: float = 40.0  # deg -> motors off
    trim: float = 0.0         # deg, balance-point offset (MPU mounting error)
    kff: float = 1.0          # speed feed-forward (1.0 = exact back-EMF compensation)
    max_auto_trim: float = 6.0  # deg the position integral may add
    v_ramp: float = 0.6       # m/s^2 command ramp
    yaw_ramp: float = 400.0   # deg/s^2 command ramp

    def as_dict(self):
        return asdict(self)


class Controller:
    ACC_LSB = 8192.0   # +/-4 g
    GYRO_LSB = 65.5    # +/-500 dps

    def __init__(self, p, gains=None):
        self.p = p
        self.g = gains or Gains()
        self.dt = 1.0 / p.timing.control_hz
        self.cpr = p.motor.counts_per_wheel_rev
        self.r = p.wheel.radius
        self.track = p.wheel.track
        pw = p.power
        self.adc_scale = 3.3 / 4095 * (pw.divider_top + pw.divider_bottom) / pw.divider_bottom
        self.v_cutoff = pw.v_cutoff
        # motor constant the firmware needs: volts of back-EMF per m/s of wheel speed
        self.emf_per_mps = p.motor.Ke * p.motor.gear / p.wheel.radius
        self.gyro_off = [0.0, 0.0, 0.0]
        self.reset()

    def reset(self):
        self.angle = 0.0
        self.init = False
        self.lastL = self.lastR = None
        n = max(1, int(round(0.025 / self.dt)))      # 25 ms speed window
        self.win = deque(maxlen=n)
        self.v = 0.0
        self.x = 0.0
        self.pos_i = 0.0
        self.ang_i = 0.0
        self.head_i = 0.0
        self.v_cmd = 0.0
        self.yaw_cmd = 0.0
        self.enabled = True
        self.fallen = False
        self.low_batt = False
        self.low_timer = 0.0
        self.vbat = 8.0
        self.target = 0.0
        self.yaw_rate = 0.0
        self.pwmL = self.pwmR = 0
        self.u = 0.0

    # gyro calibration: robot held still, average N samples (setup())
    def calibrate(self, samples):
        n = len(samples)
        self.gyro_off = [sum(s[3 + i] for s in samples) / n for i in range(3)]

    def update(self, raw, cntL, cntR, adc, v_cmd_in, yaw_cmd_in):
        g, dt = self.g, self.dt
        ax, ay, az, gx, gy, gz = raw
        gyro_pitch = (gy - self.gyro_off[1]) / self.GYRO_LSB         # deg/s
        gyro_yaw = (gz - self.gyro_off[2]) / self.GYRO_LSB           # deg/s
        acc_angle = atan2(-ax, az) * 180 / pi
        if not self.init:
            self.angle = acc_angle
            self.init = True
        self.angle = g.alpha * (self.angle + gyro_pitch * dt) + (1 - g.alpha) * acc_angle

        # --- encoders -> ground speed (wheel rate relative to body + body rate)
        if self.lastL is None:
            self.lastL, self.lastR = cntL, cntR
        dL, dR = cntL - self.lastL, cntR - self.lastR
        self.lastL, self.lastR = cntL, cntR
        self.win.append(dL + dR)
        # counts over a fixed 25 ms window -> less quantisation noise
        rel = sum(self.win) * 0.5 / self.cpr * 2 * pi / (len(self.win) * dt)   # rad/s
        v_raw = self.r * (rel + gyro_pitch * pi / 180)
        self.v += 0.3 * (v_raw - self.v)
        self.x += self.r * ((dL + dR) * 0.5 / self.cpr * 2 * pi + gyro_pitch * pi / 180 * dt)
        self.yaw_rate = gyro_yaw

        # --- battery
        self.vbat += 0.02 * (adc * self.adc_scale - self.vbat)
        if self.vbat < self.v_cutoff:
            self.low_timer += dt
            if self.low_timer > 2.0:
                self.low_batt = True
        else:
            self.low_timer = 0.0

        # --- command ramps (smooth joystick)
        step = g.v_ramp * dt
        self.v_cmd += max(-step, min(step, v_cmd_in - self.v_cmd))
        step = g.yaw_ramp * dt
        self.yaw_cmd += max(-step, min(step, yaw_cmd_in - self.yaw_cmd))

        # --- safety
        if abs(self.angle) > g.fall_angle:
            self.fallen = True
        if self.fallen or self.low_batt:
            self.enabled = False
            self.pos_i = self.ang_i = self.head_i = 0.0
            self.pwmL = self.pwmR = 0
            return 0, 0, False

        # --- speed loop -> lean target
        ev = self.v_cmd - self.v
        lim = g.max_auto_trim / max(g.ki_v, 1e-6)
        self.pos_i = max(-lim, min(lim, self.pos_i + ev * dt))
        tgt = g.kp_v * ev + g.ki_v * self.pos_i
        self.target = max(-g.max_lean, min(g.max_lean, tgt))

        # --- angle loop
        e = self.angle - g.trim - self.target
        self.ang_i = max(-50, min(50, self.ang_i + e * dt))
        u = g.kp * e + g.kd * gyro_pitch + g.ki * self.ang_i
        # feed-forward: PWM that makes the back-EMF of the commanded speed
        u += g.kff * self.v_cmd * self.emf_per_mps / max(self.vbat, 5.0) * 255
        self.u = u

        # --- turn loop (gyro Z = yaw rate)
        ey = self.yaw_cmd - gyro_yaw
        self.head_i = max(-90, min(90, self.head_i + ey * dt))
        diff = g.kp_t * ey + g.ki_t * self.head_i
        # feed-forward: each wheel must run at +/- yaw_rate * track/2
        diff += g.kff * (self.yaw_cmd * pi / 180) * self.track / 2 * self.emf_per_mps / max(self.vbat, 5.0) * 255

        uL, uR = u - diff, u + diff
        out = []
        for ui in (uL, uR):
            if abs(ui) > 0.5:
                ui += copysign(g.deadband, ui)
            ui = max(-g.pwm_cap, min(g.pwm_cap, ui))
            out.append(int(round(ui)))
        self.pwmL, self.pwmR = out
        self.enabled = True
        return out[0], out[1], True
