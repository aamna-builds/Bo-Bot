# Firmware (Arduino Uno build)

The robot that was built runs on an **Arduino Uno**. The current sketch is
[`bobot_v7/bobot_v7.ino`](bobot_v7/bobot_v7.ino). It uses only the built-in `Wire` and `EEPROM` libraries,
so nothing extra has to be installed.

| Folder | Contents |
|---|---|
| [`bobot_v7/`](bobot_v7/) | **Current firmware:** balancing, setup wizard, 16 OLED faces, push button, keep-straight, settings saved in EEPROM |
| [`tests/`](tests/) | Small bench sketches used during bring-up (motors, direction check, trim) |
| [`versions/`](versions/) | Every earlier Uno sketch, kept for reference |

Wiring for this firmware: [`../../wiring/README.md`](../../wiring/README.md).

## Upload

1. Arduino IDE 2.x, **Tools > Board > Arduino Uno**, pick the COM port.
2. Open `bobot_v7/bobot_v7.ino` and press **Upload**.
3. Open the Serial Monitor at **115200 baud** with line ending **Newline**.

The DRV8833 is held asleep (EEP pin D8 low, plus a 10 kΩ pull-down) from the moment the Uno resets, so the motors stay off while
uploading.

## First start: setup wizard

If no settings are saved yet, the wizard runs by itself (it can be started again any time with `check`). It asks six things in the
Serial Monitor and works out every direction setting, so nothing has to be edited in the code:

1. Hold it upright and still, press Enter.
2. Tilt it forward about 20°, press Enter (finds which MPU axis points forward).
3. Hold upright, press Enter, tip it forward within 3 s (finds the gyro sign).
4. Wheels in the air: the left wheel spins, answer `y` / `n` (left motor and encoder direction).
5. Same for the right wheel.
6. Hold it at its balance point, press Enter (measures `trim`).

The results are saved to EEPROM.

## Normal use

- **Power on** while holding it upright and still. It calibrates the gyro for 1 s and starts balancing by itself.
- **Falls over** (more than 40° from trim): the motors stop and the driver goes to sleep. Stand it back up within 8° of the balance
  point for 0.3 s and it starts again.
- **Push button (D12):** press = stop. Let go and hold it still: after 1 s it recalibrates and balances again.

## Serial commands

| Command | What it does |
|---|---|
| `g` / `x` | start / stop balancing |
| `p` | live readout every 0.1 s: angle, motor output, speed, turn, trim, state |
| `t` | measure the trim (only while stopped) |
| `show` | print the current settings |
| `save` | save the settings to EEPROM |
| `check` | run the setup wizard again |
| `oled` | look for the screen again |
| `faces` / `face <0-15>` | show all 16 faces, or one face |
| `kp` `kd` `trim` `kv` `kx` `cap` `ks` `mbal` + value | change a setting (type `save` to keep it) |

## Controller

Cascade control at **200 Hz**:

1. **Speed / position loop** sets a lean target: `target = -(kv * speed + kx * position)`, limited to ±10°.
2. **Angle loop (PD)** drives the motors: `out = kp * (angle - trim - target) + kd * rate`, limited to ±`cap`.
3. **Keep-straight:** the encoder difference adds a steering term `ks * headDiff` (limited to ±60) so it does not drift sideways.
   `mbal` (in %) evens out a weaker motor.

The tilt angle comes from a complementary filter, `angle = 0.996 * (angle + rate * dt) + 0.004 * accel_angle`. The MPU-6050 on
this robot has a dead Z accelerometer, so the accelerometer angle uses the forward axis only: `asin(-a_forward)`.

Default settings in the code (the robot uses whatever was last saved with `save`; type `show` to see them):

| kp | kd | trim | kv | kx | cap | ks | mbal |
|---|---|---|---|---|---|---|---|
| 28 | 0.6 | 0 | 30 | 3 | 230 | 1.5 | 0 |

Constants: MPU at ±500 °/s and ±4 g, 350 encoder counts per wheel turn, wheel radius 17 mm, PWM on D9/D10 set to about 976 Hz to match D5/D6.

## OLED faces

The face is drawn without a screen buffer (the Uno has only 2 KB of RAM): each byte is worked out when it is sent, 16 bytes at a time,
and only when the control loop has spare time, so the display never delays balancing. If the screen stops answering, the firmware
keeps looking for it once a second.

| # | Face | Shown when |
|---|---|---|
| 0 | sleep | stopped |
| 1 | wait | waiting to be stood up |
| 2 | sleepy | waiting for more than 30 s |
| 3 | setup | calibrating or in the wizard |
| 4 | normal | balancing calmly |
| 5, 7, 8 | happy, wink, love | idle emotes every 9 to 15 s |
| 6 | excited | moving faster than 0.15 m/s |
| 9 | surprised | pushed hard |
| 10 | dizzy | big wobble that it saved |
| 11 | angry | pushed 4 times in 10 s |
| 12 | confused | wandered 40 cm |
| 13 | cool | balancing for more than a minute |
| 14 | dead | just fell over |
| 15 | sad | fell 3 times in 30 s |

A preview of all 16 faces, drawn on a PC from the same code, is in [`../../docs/oled-faces/`](../../docs/oled-faces/).

## Safety

- Motors off beyond 40° of tilt, and the DRV8833 is put to sleep.
- If an MPU read fails, the motors are switched off for that cycle. If the MPU is missing at start-up it stops with `MPU-6050 NOT FOUND`.
- I2C has a 3 ms timeout with bus reset, so a glitching OLED cannot freeze the loop.
- There is **no battery voltage cut-off** in the Uno firmware. Do not run the LiPo below 3.3 V per cell (6.6 V for the pack).

## Test sketches

| Sketch | Use |
|---|---|
| [`tests/Uno_MotorTest`](tests/Uno_MotorTest/Uno_MotorTest.ino) | Spin both motors, with an auto test and `f` / `b` commands |
| [`tests/Uno_ControlTest`](tests/Uno_ControlTest/Uno_ControlTest.ino) | Two motors plus a control test, no display |
| [`tests/Uno_TrimFinder`](tests/Uno_TrimFinder/Uno_TrimFinder.ino) | Motors off, prints the balance-point trim |
| [`tests/Uno_AutoCheck`](tests/Uno_AutoCheck/Uno_AutoCheck.ino) | Measures every direction setting and prints lines to paste into `Uno_PID_Balance` |

## Version history

| Date | Sketch | Change |
|---|---|---|
| 16 Nov | `BOBOT_Uno`, `BOBOT_Uno_NoDisplay` | First port of the ESP32 controller to the Uno (U8g2 OLED, then without the display) |
| 16 Nov | `Uno_PID_Balance` and `Uno_PID_1` to `Uno_PID_4` | Simple PID balancing. The four numbered copies try different ways to start: lie down and stand up, a tilt gesture, auto start at power-up, and trim measured at calibration with the driver asleep on D8. `Uno_PID_2b` is the tilt-gesture copy with the IMU axis set the other way round |
| 16 Nov | `BOBOT_OneSketch` | Setup wizard and balancing in one sketch |
| 17 Nov | `BOBOT_OneSketch_Tuned` | The same sketch with the gains tuned on the robot (kp 28, kd 0.6, kv 30, kx 3) and a wider start / re-arm window. These gains became the defaults of every later version |
| 17 Nov | `BOBOT_Face`, `BOBOT_UserFace` | First OLED face |
| 17-18 Nov | `BOBOT_Faces16`, `BOBOT_Faces16b` | 16 faces, SH1106 support |
| 18 Nov | `BOBOT_Button` | Push button on D12 to recalibrate |
| 18-19 Nov | `BOBOT_Steer`, `BOBOT_SteerOldOled` | Keep-straight (`ks`) and motor balance (`mbal`) |
| 19 Nov | `BOBOT_Small`, `BOBOT_Small2` | Shorter rewrite; OLED reconnects by itself |
| 19 Nov | `BOBOT_Alive`, `BOBOT_Reset` | Moving dot on the face; button and `reset` command restart the Uno |
| 20 Nov | **`bobot_v7`** | Current version: button recalibrates in place, OLED retried once a second |

The ESP32 firmware in [`../balance_bot/`](../balance_bot/) was written first, before the build moved to the Uno. It is kept but is not on the robot.
