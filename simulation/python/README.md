# ESP32 self-balancing robot — 2.5D physics simulator

A simulator of the BO-BOT robot: GA12-N20 6 V 300 rpm encoder motors, 34 mm
wheels, DRV8833, GenX 2S 1000 mAh LiPo, MPU-6050, 1.3" SH1106 OLED and ESP32.
It runs the same control code the ESP32 will run, seeing only the raw sensor
registers, encoder counts and battery ADC. The physics around it is modelled
in detail, so gains tuned here are a realistic starting point for the real robot.

Demo video: `demo.mp4` (driving, turning, pushes, a fall, shell view).

## Install & run

```bash
pip install -r requirements.txt        # numpy, scipy, pygame, matplotlib
python app.py                          # interactive 2.5D simulator
python app.py --shell                  # start with the 3D-printed shell
python analyze.py                      # headless report + plots in ./report/
python analyze.py --shell
python analyze.py --tune               # search for gains (2-4 min)
python params.py                       # print mass budget / COM / motor constants
```
Works on Windows, macOS and Linux with Python 3.9+.

## Controls (app.py)

| Key | Action |
|---|---|
| ↑ / ↓ (Shift = faster) | drive forward / back (like the Bluetooth joystick) |
| ← / → | turn |
| X / Z (Shift = harder) | push the head forward / backward |
| R | pick it up and put it down again |
| S / O | shell on/off, x-ray view through the shell |
| [ ] and − = | select a firmware gain and change it **live** |
| 1 2 3 4 / P | sim speed 0.25×–2× / pause |
| C, mouse drag, wheel | camera mode, orbit, zoom |
| B | drain the battery to 15 % (see the low-battery face) |

## What is modelled

| Part | Model |
|---|---|
| Rigid body | 3-DOF two-wheeled inverted pendulum (forward, pitch, **yaw**), derived with Kane's method (`derive_eom.py`, sympy) incl. centrifugal and gyroscopic yaw–pitch coupling. Verified: energy conserved to 1e-13 J over 5 s of tumbling. |
| Mass/inertia | Built from a component table (`params.py`): every part as a box with mass and position → COM, full inertia tensor, fall angles. |
| Motors | DC motor from the N20 datasheet (300 rpm, 40 mA no-load, 0.7 kg·cm / 0.55 A stall @ 6 V) → R = 10.9 Ω, Kt = Ke = 3.54 mN·m/A, gearbox efficiency 0.70, back-EMF, Coulomb + viscous friction + stiction, **rotor inertia reflected through the 50:1 gearbox** (as large as the whole robot's mass at the wheel!), **gear backlash 1.5°** with gear compliance, ±4 % left/right motor mismatch. |
| Driver | DRV8833 slow-decay PWM (average voltage = duty × Vbat), 0.36 Ω on-resistance, coast when disabled, 2 A over-current check. |
| Battery | 2S LiPo open-circuit-voltage curve, internal + wiring resistance (voltage sag under load), state of charge, 1 W logic load (ESP32 + BT + OLED via buck). |
| MPU-6050 | True specific force at the chip's position (so linear acceleration corrupts the accelerometer, like in reality), DLPF 44 Hz, datasheet noise densities, gyro bias + drift, ±4 g / ±500 dps quantisation & saturation, 1° mounting error. Gyro calibrated at start-up exactly like `setup()` will. |
| Encoders | 7 PPR Hall on the motor shaft, ×4 quadrature → 1400 counts/wheel rev. Sits on the motor side of the backlash. |
| Firmware | `controller.py`: 200 Hz loop, complementary filter, cascade speed → angle PD loop, turn loop on gyro Z, back-EMF feed-forward, 80 % PWM cap, 8-bit PWM, 1 ms output delay, fall cut-off, low-battery cut-off. Units are the firmware's (deg, deg/s, PWM counts). |
| Face | `face.py`: 128×64 1-bit bitmap with U8g2-style primitives; idle/blink, happy, careful, worried, surprised, dizzy, sleepy, eyes follow turns. |
| Floor | Flat, rigid; no-slip rolling with rolling resistance and yaw scrub. Grip is **checked** (shown live as "grip used") but slipping is not simulated. |

## What the simulation says about your build

1. **Nothing may hang below the motors.** With 34 mm wheels the axle is only
   17 mm above the floor. A 45 mm base plate under the motors would hit the floor
   at 26° of tilt. The default layout clamps the motors up into deck 1, giving
   60° (40° with the shell).
2. **Keep Kd low (~0.35).** The LQR "textbook" gains (Kd 2.3) work on paper
   but buzz hard with N20 gear backlash; the accelerometer then reads the
   shaking as a 7° tilt and the robot runs away.
3. **Mount the MPU-6050 low, near the axle.** On the top deck (60 mm) the
   accelerometer sees the robot's own motion; the wobble goes from 0.2° to 7°.
4. **Trim the balance point.** A 1° crooked MPU is absorbed automatically, but
   3° makes it drift ~80 cm before settling. Use the `trim` gain (deg).
5. **Speed:** top speed is ~0.57 m/s at the 80 % cap on 7.4 V; drive commands
   up to ~0.35 m/s leave enough reserve to stay balanced.
6. **Push tolerance:** ~0.04 N·s at the head (0.054 with the shell, because the
   heavier body reacts more slowly) — a firm finger flick.
7. **Battery:** ~125 mA standing (mostly the ESP32), ~6 h on the 1000 mAh pack.
8. **No dead-band compensation.** With backlash it turns into chatter.
9. **200 Hz loop, speed measured over 25 ms windows** — 100 Hz is noticeably
   worse, 500 Hz gives no real gain.

Full numbers: `report/report.txt`, `report/report_shell.txt` and the PNG plots.

## Tuned starting gains (flash these first)

```
Kp  = 55     PWM counts per degree          Kd  = 0.35  PWM per deg/s
Kp_v = 20    deg of lean per m/s            Ki_v = 0.9  deg of lean per metre
Kp_t = 0.3   PWM per deg/s of yaw error     Ki_t = 1.0  PWM per degree of heading
alpha = 0.98 complementary filter           PWM cap = 204 (80 %)
kff  = 1.0   back-EMF feed-forward          fall cut-off = 40 deg
```
Real-robot tuning order: trim → Kp/Kd (standing) → Kp_v/Ki_v (holding position)
→ turn gains. The simulator shows what each change does.

## Make it match your real robot

When the parts arrive, **weigh them** and edit `COMPONENTS` in `params.py`
(grams, millimetres, body frame: origin at the axle, x forward, z up). If
your printed layout differs, move the boxes. Everything else recomputes.
Also worth measuring: the motor no-load speed at 6 V (count wheel turns), and
the backlash (hold the wheel, wiggle it, estimate the free play).

## Files

```
app.py          interactive 2.5D viewer (pygame)
sim.py          orchestrator: 2 kHz physics, 200 Hz firmware, timing
physics.py      plant: rigid body + motors + gearbox + DRV8833 + LiPo
eom_generated.py  equations of motion (generated by derive_eom.py)
derive_eom.py   Kane's-method derivation (needs sympy)
params.py       every physical parameter + mass budget
sensors.py      MPU-6050, encoders, battery ADC
controller.py   the firmware (port 1:1 to Arduino)
face.py         OLED faces
lqr.py          linearised model + LQR gains
analyze.py      headless tests, sensitivity study, tuner, plots
```

## Limits (be honest with the model)

- Parameters come from datasheets of the **same motor type** (Robu doesn't
  publish one) and estimated part masses. Real units vary ±10–20 %.
- Wheel slip is detected, not simulated; the floor is flat and rigid.
- The 3D-printed frame is assumed rigid; wires don't pull on anything.
- Expect to re-tune by 20–40 % on the real robot. The simulator gets you to
  the right neighbourhood and teaches you what each gain does.
