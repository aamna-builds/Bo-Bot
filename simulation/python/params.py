"""
All physical parameters of YOUR robot, in one place.

Every number has a source or a reason next to it. When the parts arrive,
weigh them (kitchen scale, 0.1 g if you can) and measure your printed
layout, then edit COMPONENTS below. Everything else (COM height, inertia,
LQR gains, fall angles) is recomputed from this table.

Units: SI internally (kg, m, s, N, V, A). The component table uses grams and
millimetres because that is what you will measure.

Body frame (used everywhere):
    origin = midpoint of the wheel axle
    +x = forward (the side the OLED faces), +y = left, +z = up
"""
from dataclasses import dataclass, field, replace
from math import pi, atan2, sqrt, cos, sin
import copy

G0 = 9.81


# ---------------------------------------------------------------------------
# 1. MASS BUDGET / LAYOUT  (grams, millimetres, body frame)
# ---------------------------------------------------------------------------
# name, mass_g, (x, y, z) centre mm, (dx, dy, dz) size mm, colour, group
# group: "frame" printed PLA, "elec" electronics, "power", "motor", "shell"
# Heights: axle is z = 0, so the floor is at z = -17 (wheel radius).
COMPONENTS = [
    # --- drive -------------------------------------------------------------
    # GA12-N20 + Hall encoder: ~10 g motor/gearbox (Pololu micro metal
    # gearmotor = 9.5 g) + ~1.5 g encoder PCB, magnet and 6-wire lead.
    # Body 12 x 10 mm, ~32 mm long incl. encoder; gearbox face ~5 mm from
    # wheel centre, so it reaches from y = 34 to y = 2.
    ("motor_L",      11.5, (0,  18, 0),   (12, 32, 10), (150, 150, 160), "motor"),
    ("motor_R",      11.5, (0, -18, 0),   (12, 32, 10), (150, 150, 160), "motor"),
    # Motors clamp UP into deck 1: nothing hangs below the motors, because with
    # 34 mm wheels the axle is only 17 mm above the floor. (A 45 mm base plate
    # under the motors would hit the floor at ~26 deg of tilt.)
    ("motor_clamps", 10.0, (0,   0, 1.0),  (16, 64, 12), (240, 140, 60), "frame"),
    # --- level 1: battery deck right on top of the motors --------------------
    # Below the wheel tops (z < 17) everything must fit between the tyres:
    # inner tyre faces are at y = +/-35.75 mm.
    ("deck1",         9.0, (0,   0, 6.5),  (45, 68, 3),  (240, 140, 60), "frame"),
    # GenX 2S 1000 mAh: 68 x 37 x 12 mm, 54 g (Robokits listing). Lies across
    # the robot, long side along the axle (1.75 mm clearance to each tyre).
    ("battery",      54.0, (0,   0, 14.0), (37, 68, 12), (60, 120, 200), "power"),
    ("xt30_leads",    4.0, (-20, 30, 14),  (8, 10, 8),   (230, 200, 40), "power"),
    ("pillars_1",     6.0, (0,   0, 15.5), (40, 66, 15), (240, 140, 60), "frame_hidden"),
    # --- level 2: electronics deck (~25 mm shelf) ----------------------------
    ("deck2",         9.0, (0,   0, 24.5), (50, 80, 3),  (240, 140, 60), "frame"),
    # ESP32 DevKit 30-pin: 51 x 28 mm PCB, ~9.5 g with headers
    ("esp32",         9.5, (6,   0, 32.0), (28, 51, 11), (40, 40, 45),   "elec"),
    ("mpu6050",       2.0, (-16, 0, 27.5), (16, 21, 3),  (30, 90, 170),  "elec"),
    ("drv8833",       2.0, (-16, 20, 28.0), (16, 18, 4), (200, 40, 40),  "elec"),
    ("buck_5v",       5.0, (-16, -21, 30.0), (17, 26, 8), (30, 110, 60), "elec"),
    ("pillars_2",     6.0, (0,   0, 38.5), (44, 74, 25), (240, 140, 60), "frame_hidden"),
    # --- level 3: top deck with breadboard and the head ----------------------
    ("deck3",         9.0, (0,   0, 52.5), (50, 80, 3),  (240, 140, 60), "frame"),
    ("breadboard",    9.0, (-5,  0, 58.8), (35, 47, 9.5), (235, 235, 235), "elec"),
    ("oled_mount",    6.0, (20,  0, 70.0), (6, 40, 36),  (240, 140, 60), "frame"),
    # 1.3" SH1106 module: 35.4 x 33.5 mm PCB, ~6 g. Mounted vertical, facing +x
    ("oled",          6.0, (25,  0, 72.0), (4, 35.4, 33.5), (15, 15, 20), "oled"),
    # --- distributed bits ----------------------------------------------------
    ("jumper_wires", 15.0, (0,   0, 35.0), (40, 60, 40), (200, 60, 200), "hidden"),
    ("m3_screws",     8.0, (0,   0, 25.0), (44, 74, 50), (180, 180, 180), "hidden"),
    # --- optional 3D-printed shell (1.2 mm PLA walls) + ears + bow tie --------
    # lower part sits between the wheels, upper part overhangs them
    ("shell_lower",  10.0, (0,   0, 12.0), (62, 68, 16), (250, 235, 215), "shell"),
    ("shell_upper",  48.0, (0,   0, 59.0), (62, 86, 78), (250, 235, 215), "shell"),
    ("ears",          5.0, (0,   0, 104.0), (20, 70, 14), (250, 200, 210), "shell"),
    ("bow_tie",       3.0, (33,  0, 30.0), (4, 30, 12),  (220, 40, 60),  "shell"),
]


# ---------------------------------------------------------------------------
# 2. WHEELS  (Robu "White 3PI miniQ wheel 34 mm for N20")
# ---------------------------------------------------------------------------
@dataclass
class WheelParams:
    radius: float = 0.017        # 34 mm diameter
    width: float = 0.0065        # 6.5 mm tyre
    mass: float = 0.0032         # 3.2 g each (listing)
    track: float = 0.078         # wheel centre-to-centre; 2 x (32 mm motor + 5 mm) + gap
    # Spin inertia: ~2.0 g tyre ring (r 13..17 mm) + ~1.2 g hub (r ~8 mm)
    Iwa: float = 0.5 * 0.0020 * (0.017**2 + 0.013**2) + 0.5 * 0.0012 * 0.008**2
    Iwt: float = 0.0             # filled in post-init (transverse)
    mu: float = 0.8              # rubber on wood/tile, for the slip check only
    c_rr: float = 0.015          # rolling-resistance coefficient, hard floor

    def __post_init__(self):
        if not self.Iwt:
            self.Iwt = 0.5 * self.Iwa + self.mass * self.width**2 / 12


# ---------------------------------------------------------------------------
# 3. MOTOR  GA12-N20 6 V 300 RPM 1:50 with Hall encoder
# ---------------------------------------------------------------------------
# Datasheet (RobotShop RM-ESMO-011, same 6 V 50:1 300 rpm N20):
#   no-load 300 rpm @ 6 V, no-load current 40 mA,
#   stall torque 0.7 kg.cm, stall current 0.55 A, encoder 7 PPR (2 channels)
@dataclass
class MotorParams:
    V_rated: float = 6.0
    rpm_noload: float = 300.0          # at output shaft, 6 V
    I_noload: float = 0.040
    I_stall: float = 0.55
    T_stall_kgcm: float = 0.70         # at output shaft, 6 V
    gear: float = 50.0
    ppr: int = 7                       # encoder pulses / motor rev / channel
    quad: int = 4                      # quadrature decoding (ESP32 PCNT: x4)
    # Rotor inertia: N20 6 V motors spin up in ~20 ms (mechanical time
    # constant tau = J R / (Kt Ke)) -> J ~ 2.3e-8; +10 % for the first gears.
    J_rotor: float = 2.5e-8
    stiction_ratio: float = 1.6        # breakaway / Coulomb friction (~0.45 V start)
    coulomb_frac: float = 0.6          # share of no-load friction that is Coulomb
    backlash_deg: float = 1.5          # free play at the output shaft (N20 metal: 1..3 deg)
    gear_stiffness: float = 3.0        # N m / rad at the output shaft
    gear_damping_ratio: float = 0.25
    # unit-to-unit spread (applied to the RIGHT motor, left is nominal)
    mismatch_kt: float = 0.96
    mismatch_R: float = 1.04
    mismatch_fric: float = 1.2

    # ---- derived (computed in post-init) ----
    R: float = 0.0
    Ke: float = 0.0
    Kt: float = 0.0
    eta: float = 0.0
    J_out: float = 0.0
    T_coulomb_out: float = 0.0
    b_visc_out: float = 0.0

    def __post_init__(self):
        self.R = self.V_rated / self.I_stall                       # 10.9 ohm
        w_nl_motor = self.rpm_noload * self.gear * 2 * pi / 60      # 1571 rad/s
        self.Ke = (self.V_rated - self.I_noload * self.R) / w_nl_motor
        self.Kt = self.Ke                                           # SI: Kt == Ke
        T_stall = self.T_stall_kgcm * 0.0980665
        self.eta = T_stall / (self.gear * self.Kt * self.I_stall)  # ~0.70
        self.J_out = self.J_rotor * self.gear**2                    # reflected
        T_fric_out = self.eta * self.gear * self.Kt * self.I_noload
        self.T_coulomb_out = self.coulomb_frac * T_fric_out
        w_nl_out = self.rpm_noload * 2 * pi / 60
        self.b_visc_out = (1 - self.coulomb_frac) * T_fric_out / w_nl_out

    @property
    def counts_per_wheel_rev(self):
        return self.ppr * self.quad * self.gear


# ---------------------------------------------------------------------------
# 4. POWER: GenX 2S 1000 mAh LiPo + DRV8833 + 5 V buck
# ---------------------------------------------------------------------------
# LiPo open-circuit voltage per cell vs state of charge (typical curve)
LIPO_OCV = [(0.00, 3.27), (0.05, 3.61), (0.10, 3.69), (0.15, 3.71), (0.20, 3.73),
            (0.25, 3.75), (0.30, 3.77), (0.35, 3.79), (0.40, 3.80), (0.45, 3.82),
            (0.50, 3.84), (0.55, 3.85), (0.60, 3.87), (0.65, 3.91), (0.70, 3.95),
            (0.75, 3.98), (0.80, 4.02), (0.85, 4.08), (0.90, 4.11), (0.95, 4.15),
            (1.00, 4.20)]


@dataclass
class PowerParams:
    cells: int = 2
    capacity_mAh: float = 1000.0
    soc0: float = 0.95                 # freshly charged, rested
    R_internal: float = 0.030          # 2S 35C 1000 mAh pack (~15 mohm/cell)
    R_wiring: float = 0.015            # XT30 + 20 AWG leads
    drv_rds_on: float = 0.36           # DRV8833 HS+LS on-resistance (datasheet, 25 C)
    drv_ocp: float = 2.0               # A, overcurrent shutdown (datasheet min)
    logic_power_W: float = 1.0         # ESP32 + BT (~130 mA) + OLED + MPU + encoders at 5 V, / buck eff.
    divider_top: float = 10_000.0
    divider_bottom: float = 4_700.0
    adc_gain_error: float = 1.02       # ESP32 ADC reads ~2 % high uncalibrated
    v_cutoff: float = 6.6              # firmware low-battery cutoff (3.3 V/cell)


# ---------------------------------------------------------------------------
# 5. SENSORS: MPU-6050 + encoders
# ---------------------------------------------------------------------------
@dataclass
class SensorParams:
    # where the MPU sits (body frame, metres) -- from the component table
    mpu_pos: tuple = (-0.016, 0.0, 0.0275)
    mount_error_deg: float = 1.0       # PCB not perfectly level -> standing offset
    dlpf_hz: float = 44.0              # DLPF_CFG = 3 (44 Hz accel / 42 Hz gyro)
    accel_range_g: float = 4.0         # +/-4 g  -> 8192 LSB/g
    gyro_range_dps: float = 500.0      # +/-500 dps -> 65.5 LSB/(deg/s)
    accel_noise_density: float = 400e-6 * G0     # 400 ug/sqrt(Hz) (datasheet)
    gyro_noise_density: float = 0.005 * pi / 180  # 0.005 dps/sqrt(Hz) (datasheet)
    gyro_bias_dps: float = 0.3         # residual after the start-up calibration
    gyro_bias_walk: float = 0.002      # dps/sqrt(s) drift
    vibration_g: float = 0.03          # extra accel noise from motors, per m/s of wheel speed
    seed: int = 1


# ---------------------------------------------------------------------------
# 6. FIRMWARE / CONTROL TIMING
# ---------------------------------------------------------------------------
@dataclass
class TimingParams:
    physics_dt: float = 0.0005         # 2 kHz physics (RK4)
    control_hz: float = 200.0          # ESP32 balance loop (core 1)
    output_delay: float = 0.001        # I2C read (14 bytes @ 400 kHz) + maths
    pwm_bits: int = 8                  # 0..255, 20 kHz LEDC


# ---------------------------------------------------------------------------
# Assembled parameter set with derived mass properties
# ---------------------------------------------------------------------------
@dataclass
class Params:
    shell: bool = False
    wheel: WheelParams = field(default_factory=WheelParams)
    motor: MotorParams = field(default_factory=MotorParams)
    power: PowerParams = field(default_factory=PowerParams)
    sensor: SensorParams = field(default_factory=SensorParams)
    timing: TimingParams = field(default_factory=TimingParams)
    components: list = field(default_factory=lambda: copy.deepcopy(COMPONENTS))
    extra_payload_g: float = 0.0       # add a test weight on the top deck
    g: float = G0

    # derived body properties (filled by compute())
    mb: float = 0.0
    l: float = 0.0
    e: float = 0.0
    Ixx: float = 0.0
    Iyy: float = 0.0
    Izz: float = 0.0
    Ixz: float = 0.0
    mw: float = 0.0
    r: float = 0.0
    d: float = 0.0
    Iwa: float = 0.0
    Iwt: float = 0.0
    fall_fwd: float = 0.0
    fall_back: float = 0.0
    height_mm: float = 0.0

    def active_components(self):
        out = []
        for c in self.components:
            if c[5] == "shell" and not self.shell:
                continue
            out.append(c)
        if self.extra_payload_g > 0:
            out.append(("payload", self.extra_payload_g, (0, 0, 58), (20, 20, 10),
                        (120, 120, 120), "elec"))
        return out

    def compute(self):
        comps = self.active_components()
        m = sum(c[1] for c in comps) / 1000.0
        cx = sum(c[1] * c[2][0] for c in comps) / 1000.0 / m / 1000.0
        cz = sum(c[1] * c[2][2] for c in comps) / 1000.0 / m / 1000.0
        Ixx = Iyy = Izz = Ixz = 0.0
        for name, mg, (x, y, z), (dx, dy, dz), *_ in comps:
            mk = mg / 1000.0
            x, y, z = x / 1000 - cx, y / 1000, z / 1000 - cz
            dx, dy, dz = dx / 1000, dy / 1000, dz / 1000
            # hollow shell: thin-walled box ~ 5/3 of the solid-box value
            k = 5 / 3 if name.startswith("shell") else 1.0
            Ixx += mk * (k * (dy**2 + dz**2) / 12 + y**2 + z**2)
            Iyy += mk * (k * (dx**2 + dz**2) / 12 + x**2 + z**2)
            Izz += mk * (k * (dx**2 + dy**2) / 12 + x**2 + y**2)
            Ixz += -mk * x * z
        self.mb, self.l, self.e = m, cz, cx
        self.Ixx, self.Iyy, self.Izz, self.Ixz = Ixx, Iyy, Izz, Ixz
        w = self.wheel
        self.mw, self.r, self.d, self.Iwa, self.Iwt = w.mass, w.radius, w.track, w.Iwa, w.Iwt
        # fall angles: first body corner to touch the floor
        corners = []
        for c in comps:
            if c[5] in ("hidden", "frame_hidden"):
                continue
            (x, y, z), (dx, dy, dz) = c[2], c[3]
            for sx in (-1, 1):
                for sz in (-1, 1):
                    corners.append(((x + sx * dx / 2) / 1000, (z + sz * dz / 2) / 1000))
        self.height_mm = max(z for _, z in corners) * 1000 + w.radius * 1000

        def first_contact(sign):
            th = 0.0
            while abs(th) < pi / 2:
                th += sign * 0.002
                s, c = sin(th), cos(th)
                if min(self.r - x * s + z * c for x, z in corners) <= 0:
                    return th
            return sign * pi / 2
        self.fall_fwd = first_contact(+1)
        self.fall_back = first_contact(-1)
        return self

    # ---- convenience summaries -----------------------------------------
    @property
    def total_mass(self):
        return self.mb + 2 * self.mw

    def summary(self):
        mo = self.motor
        lines = [
            f"Body mass        {self.mb*1000:7.1f} g   (+2 wheels {2*self.mw*1000:.1f} g -> total {self.total_mass*1000:.1f} g)",
            f"COM above axle   {self.l*1000:7.1f} mm  (forward offset {self.e*1000:+.1f} mm)",
            f"COM above floor  {(self.l+self.r)*1000:7.1f} mm  (robot height {self.height_mm:.0f} mm)",
            f"Pitch inertia    {self.Iyy*1e6:7.1f} kg.mm^2 about COM",
            f"Fall angles      {self.fall_fwd*180/pi:+.0f} deg fwd / {self.fall_back*180/pi:+.0f} deg back (body hits floor)",
            f"Motor R={mo.R:.1f} ohm  Kt={mo.Kt*1000:.2f} mNm/A  gear eff={mo.eta:.2f}  "
            f"reflected rotor J={mo.J_out*1e6:.1f} kg.mm^2/motor",
            f"Encoder          {mo.counts_per_wheel_rev} counts / wheel rev (x{mo.quad})",
        ]
        return "\n".join(lines)


def make_params(shell=False, **overrides):
    p = Params(shell=shell)
    for k, v in overrides.items():
        if "." in k:
            grp, key = k.split(".", 1)
            sub = getattr(p, grp)
            setattr(p, grp, replace(sub, **{key: v}))
        else:
            setattr(p, k, v)
    return p.compute()


if __name__ == "__main__":
    for sh in (False, True):
        print(f"--- shell {'ON' if sh else 'OFF'} ---")
        print(make_params(shell=sh).summary())
