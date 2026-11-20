# Credits

BO-BOT is designed and built by **Aamna Khan** ([@aamna-builds](https://github.com/aamna-builds)).
© 2026 Aamna Khan. All rights reserved (see [`LICENSE`](LICENSE)).

The project uses the following third-party software, libraries and tools. They keep their own licences.

## Included in this repository

| Item | Author | Licence | Where |
|---|---|---|---|
| [three.js](https://threejs.org/) | three.js authors | MIT ([licence file](simulation/web/vendor/THREE-LICENSE.txt)) | `simulation/web/vendor/three.min.js`, used by the web simulator |

## Libraries used by the code (not included)

| Item | Author | Licence | Used by |
|---|---|---|---|
| Arduino AVR core, `Wire` and `EEPROM` libraries | Arduino | LGPL 2.1 | Arduino Uno firmware (`firmware/uno/`) |
| [U8g2](https://github.com/olikraus/u8g2) | Oliver Kraus (olikraus) | BSD 2-Clause | `firmware/uno/versions/BOBOT_Uno`, ESP32 firmware |
| [Arduino-ESP32 core](https://github.com/espressif/arduino-esp32) | Espressif Systems | LGPL 2.1 | ESP32 firmware (`firmware/balance_bot/`) |
| [PlatformIO](https://platformio.org/) | PlatformIO Labs | Apache 2.0 | Building the ESP32 firmware |
| [NumPy](https://numpy.org/), [SciPy](https://scipy.org/) | NumPy and SciPy developers | BSD 3-Clause | Python simulator and analysis |
| [Matplotlib](https://matplotlib.org/) | Matplotlib Development Team | Matplotlib licence (BSD-style) | Plots, drawing sheets, base plate check sheet |
| [pygame](https://www.pygame.org/) | pygame community | LGPL 2.1 | Interactive Python simulator |
| [SymPy](https://www.sympy.org/) | SymPy Development Team | BSD 3-Clause | Deriving the equations of motion |
| [ezdxf](https://ezdxf.mozman.at/) | Manfred Moitzi | MIT | Base plate DXF generator |
| [ReportLab](https://www.reportlab.com/) | ReportLab Inc. | BSD | First BOM PDFs |
| [Pillow](https://python-pillow.org/) | Jeffrey A. Clark and contributors | MIT-CMU (HPND) | Face shell concept sheet |
| [Playwright](https://playwright.dev/) | Microsoft | Apache 2.0 | Simulator smoke test and demo video recorder |

## Tools

| Tool | Used for |
|---|---|
| Autodesk Fusion | 3D models of the printed parts and the first full-robot model |
| OpenSCAD | Motor clamp and spacer pins |
| Creality Print | Slicing the printed parts (G-code in `cad/printed-parts/gcode/`) |
| MATLAB and Simulink (MathWorks) | Control analysis and the Simulink model |
| Arduino IDE | Uploading the Uno firmware |
| FFmpeg | Trimming the test videos |

## Parts

Electronic parts were bought from [Robu.in](https://robu.in/) and [Robokits India](https://robokits.co.in/).
Datasheets and part names (MPU-6050 by TDK InvenSense, DRV8833 by Texas Instruments, SH1106 OLED controller by Sino Wealth,
GA12-N20 geared motors) belong to their manufacturers.

## Ideas and references

- The cascade balance controller (a tilt loop inside a speed / position loop) and the complementary filter are standard
  techniques for two-wheeled inverted-pendulum robots, described in many textbooks and open-source balancing robots.
- The equations of motion follow the usual wheeled inverted pendulum model; the derivation is in [`docs/equations.md`](docs/equations.md).
