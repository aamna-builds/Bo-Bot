# Assembly and first power-up checklist

A tick-list for the first build. It follows `wiring/README.md` (pins and power) and the bring-up steps in
`firmware/README.md`. Tick each box as you go and note anything that differs from the docs.

## 1. Before soldering
- [ ] All parts present: 2 wheels, 2 N20 motors with encoders, 1.3" OLED, DRV8833 board, MPU-6050 board, buck module, XT30 set, 2S LiPo, charger, ESP32, jumper wires
- [ ] Measure with calipers and write the numbers in `components/README.md`: OLED board size, OLED active area, wheel diameter
- [ ] Note the markings on the buck module (input range, output) before connecting a battery

## 2. Soldering
- [ ] Headers on the DRV8833 board
- [ ] Headers on the MPU-6050 board
- [ ] Pins on the OLED
- [ ] XT30 connectors on the battery lead and the power switch lead, correct polarity (red = +)
- [ ] Look at every joint under good light: no bridges between neighbouring pins

## 3. Checks with a multimeter, battery disconnected
- [ ] Continuity: no short between battery + and GND
- [ ] Every ground goes to one common ground
- [ ] Wiring matches the pin table in `wiring/README.md`
- [ ] DRV8833 `SLP` / `nSLEEP` pin (if the board has one) tied to 3V3
- [ ] Encoders powered from 3V3, not 5 V

## 4. Power the electronics
- [ ] Connect the battery to the buck module alone and measure the output: about 5 V
- [ ] Only then connect the buck output to the ESP32 VIN pin
- [ ] Never connect USB and the battery to the ESP32 at the same time

## 5. Software bring-up (wheels OFF the ground, in order, see `firmware/README.md`)
- [ ] ESP32 on USB only, motor battery disconnected: the monitor prints the MPU `WHO_AM_I`, then the gyro offsets
- [ ] OLED shows the face
- [ ] Each motor turns the right way, and the encoder counts go up when the wheel goes forward
- [ ] Battery voltage reading is sensible (a full 2S pack is 8.4 V)
- [ ] Tilt the robot by hand: pitch changes the right way and the wheels push towards the lean
- [ ] First balance on the floor with a soft landing around it, starting from the default gains

## 6. Record
- [ ] Photos of the wiring and the finished robot into `media/images/`
- [ ] Control loop rate and pitch data logged, for the model comparison in the report
- [ ] Anything that differed from the docs written down here
