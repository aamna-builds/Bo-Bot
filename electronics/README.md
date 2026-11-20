# Electronics

The robot runs on an **Arduino Uno** (the first plan used an ESP32; see [`../wiring/`](../wiring/)).

- **Power:** 7.4 V 2S LiPo through an XT60 connector, straight to the DRV8833 motor supply and to the Uno VIN pin.
  The Uno's own regulators give 5 V (encoders) and 3.3 V (MPU-6050 and OLED).
- **Motor driver:** DRV8833, slow-decay PWM on D5/D6 (left) and D9/D10 (right), sleep pin on D8 with a 10 kΩ pull-down.
- **Sensors:** MPU-6050 and the 1.3" SH1106 OLED share the I2C bus (A4/A5). Both N20 motors have quadrature encoders on D2/D4 and D3/D7.
- **Input:** one push button on D12 to stop and recalibrate.

Full wiring diagram and pin map: [`../wiring/README.md`](../wiring/README.md). Part list: [`../components/`](../components/).
