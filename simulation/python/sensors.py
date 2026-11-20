"""
Sensor models: MPU-6050 (accel + gyro), quadrature Hall encoders, battery ADC.

The MPU model is physically driven: the accelerometer sees the true specific
force at the chip's position (gravity + the chip's own linear acceleration,
which is what makes accelerometer-only tilt useless while the robot moves),
filtered by the on-chip DLPF, plus white noise, bias, quantisation and
saturation, in the chip's axes (mounted level, X forward, Y left, Z up,
with a small mounting tilt error).
"""
import random
from math import sin, cos, pi, sqrt, floor, exp
from physics import ITH, ITHD, IPSID, IGL, IGR, IV


class MPU6050:
    def __init__(self, p):
        sp = p.sensor
        self.sp = sp
        self.p = p
        self.rng = random.Random(sp.seed)
        self.tau = 1 / (2 * pi * sp.dlpf_hz)
        self.mount = sp.mount_error_deg * pi / 180
        self.alsb = 32768 / sp.accel_range_g          # LSB per g
        self.glsb = 32768 / sp.gyro_range_dps         # LSB per dps
        self.reset()

    def reset(self):
        g = self.p.g
        self.f = [0.0, 0.0, g]      # filtered specific force (chip axes)
        self.w = [0.0, 0.0, 0.0]    # filtered rate (chip axes)
        r = self.rng
        b = self.sp.gyro_bias_dps * pi / 180
        self.bias = [r.gauss(0, b), r.gauss(0, b), r.gauss(0, b)]

    def update(self, plant, dt):
        """Call every physics step (runs the DLPF at the physics rate)."""
        x = plant.x
        th = x[ITH] + self.mount
        s, c = sin(th), cos(th)
        px, _, pz = self.sp.mpu_pos
        ax, ay, az = plant.body_point_accel_A(px, pz)
        az += self.p.g                       # specific force = a - g_vec
        # rotate A -> chip (B tilted by mount error)
        fx = c * ax - s * az
        fy = ay
        fz = s * ax + c * az
        wx = -x[IPSID] * s
        wy = x[ITHD]
        wz = x[IPSID] * c
        k = dt / (self.tau + dt)
        f, w = self.f, self.w
        for i, val in enumerate((fx, fy, fz)):
            f[i] += k * (val - f[i])
        for i, val in enumerate((wx, wy, wz)):
            w[i] += k * (val - w[i])
        bw = self.sp.gyro_bias_walk * pi / 180 * sqrt(dt)
        for i in range(3):
            self.bias[i] += self.rng.gauss(0, bw)

    def read(self, wheel_speed):
        """Return raw int16 registers like Wire.read() would: (ax, ay, az, gx, gy, gz)."""
        sp, r = self.sp, self.rng
        bw = 1.57 * sp.dlpf_hz                         # noise bandwidth
        an = sp.accel_noise_density * sqrt(bw) + sp.vibration_g * 9.81 * abs(wheel_speed)
        gn = sp.gyro_noise_density * sqrt(bw)
        out = []
        for i in range(3):
            v = (self.f[i] + r.gauss(0, an)) / self.p.g * self.alsb
            out.append(max(-32768, min(32767, int(round(v)))))
        for i in range(3):
            v = (self.w[i] + self.bias[i] + r.gauss(0, gn)) * 180 / pi * self.glsb
            out.append(max(-32768, min(32767, int(round(v)))))
        return out


class Encoder:
    """Hall encoder on the motor shaft (sees the MOTOR side of the backlash)."""

    def __init__(self, p, idx_angle, sign=1):
        self.cpr_motor = p.motor.ppr * p.motor.quad
        self.G = p.motor.gear
        self.idx = idx_angle
        self.sign = sign

    def count(self, plant):
        motor_angle = plant.x[self.idx] * self.G
        return self.sign * floor(motor_angle / (2 * pi) * self.cpr_motor)


class BatteryADC:
    def __init__(self, p, seed=2):
        self.pw = p.power
        self.rng = random.Random(seed)

    def read(self, vbat):
        pw = self.pw
        v = vbat * pw.divider_bottom / (pw.divider_top + pw.divider_bottom)
        v = v * pw.adc_gain_error + self.rng.gauss(0, 0.01)
        return max(0, min(4095, int(v / 3.3 * 4095)))   # 12-bit, 11 dB atten.
