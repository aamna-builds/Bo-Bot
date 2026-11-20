# Project log and to-do

## Log
| Date | What was done |
|---|---|
| 2026-11-09 | Repository created. Fusion 360 CAD scripts (Python + C++) added. |
| 2026-11-09 | Cover page redesigned. Concept sketches and design sheets added. Rough web simulator added with offline support, video recording and a demo video. |
| 2026-11-09 | Parts list finalised and added to `components/`. CAD updated for the 1.3" OLED (face window, bezel, display module; buck converter moved 2 mm). |
| 2026-11-09 | ESP32 firmware written (balance controller and OLED face ported from the simulator, hardware layer, serial tuning), pin map filled in, tests and CI added. Not yet run on hardware. |
| 2026-11-09 | Simulator animation improved: rounded cat shell, pointed ears and bow tie that swing with the motion, tyre tread, contact shadow, paw for pushes, dizzy stars after a fall, smoother camera. Demo video and GIF re-rendered. Physics and controller unchanged. |
| 2026-11-09 | Equations and graphs added: linear model of the balance robot, firmware control law analysed (poles, margins, gain ranges), checked against the simulator. MATLAB scripts and a Simulink model builder written (not yet run in MATLAB). See `docs/equations.md` and `simulink/`. |
| 2026-11-10 | Repository moved to `Bo-Bot`; live simulator published on GitHub Pages. Simulator pick-up bug fixed (a push survived a reset). MATLAB analysis script made to run without the Control System Toolbox and checked in R2025b. |
| 2026-11-11 | Simulator recoloured blue with a dark mode toggle; new banner. Design sheets redrawn as A3 engineering drawings (general arrangement and electronics). Demo video and GIF re-recorded. |
| 2026-11-12 | Simulink model built and run in MATLAB: 3 degree tilt recovers in about 0.6 s, speed step settles in about 3 s. Screenshots and graphs added to `simulink/results_matlab/`. |
| 2026-11-13 | Project progress report written. Components still on order. |
| 2026-11-14 | Components arrived: wheels, N20 motors with encoders, 1.3" OLED, DRV8833 driver, MPU-6050, buck module, balance charger and XT60 connectors. Photos added to `components/`. Battery and ESP32 still to confirm. |
| 2026-11-14 | Laser-cut base plate drawn (89 x 62 mm, motor clamp slots, encoder windows in v2). Two-piece 3D-printed N20 motor clamp designed. |
| 2026-11-15 | Plate spacers designed and printed in three versions (spacers, snap pins, pins with a peg-size test print). |
| 2026-11-16 | Controller changed from the ESP32 to an **Arduino Uno**. Controller ported to the Uno; motor, trim and direction test sketches written. First PID balancing sketches. |
| 2026-11-16 | Setup wizard written: it finds the IMU axis, gyro sign and motor / encoder directions and saves them to EEPROM. The MPU-6050's Z accelerometer turned out to be dead, so tilt is measured from the forward axis only. |
| 2026-11-17 | OLED face added to the Uno firmware, drawn without a screen buffer to fit in 2 KB of RAM. Face shell concepts drawn. |
| 2026-11-18 | 16 faces with automatic reactions (pushes, falls, wandering, idle emotes). Push button on D12 to stop and recalibrate. Keep-straight (`ks`) and motor balance (`mbal`) added to keep it driving straight. |
| 2026-11-19 | Firmware shortened; OLED reconnects by itself after a glitch; moving dot on the face. |
| 2026-11-20 | Firmware **v7**. Plate stands printed in PLA, front faceplate v4 printed in TPU. Uno wiring diagram, Fusion 360 part files, G-code, build photos, a robot test video, a print time-lapse and the Python simulator added to the repo. |

## To-do
- [ ] Run CAD script in Fusion 360 and export `.f3d` / `.step` to `cad/exports/`
- [ ] Add exported renders to `media/images/`
- [x] Firmware: IMU reading + complementary filter (written)
- [x] Firmware: balance controller + motor/encoder drivers (written)
- [x] Firmware: OLED face animations (written)
- [x] Firmware: moved to the Arduino Uno; bring-up done with the setup wizard (see `firmware/uno/README.md`)
- [ ] Firmware: finish tuning on the robot and record the saved values (`show`)
- [ ] Firmware: battery voltage monitoring
- [x] Simulation / Simulink model of the inverted-pendulum robot (equations, linear model and graphs done; run `build_bobot_model.m` in MATLAB and commit the `.slx`)
- [x] Fill in wiring pin map
- [x] Wire the robot (Arduino Uno build, see `wiring/README.md`)
- [ ] Measure the real 1.3" OLED with calipers and confirm the CAD face window and clearances
- [x] Print the plate stands and the front faceplate
- [ ] Record a balancing demo video
