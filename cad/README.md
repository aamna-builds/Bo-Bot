# CAD

| Path | Contents |
|---|---|
| [`printed-parts/fusion360/`](printed-parts/fusion360/) | Fusion 360 files of the 3D-printed parts on the robot |
| [`printed-parts/gcode/`](printed-parts/gcode/) | Sliced G-code for the plate stands (PLA) |
| [`printed-parts/n20_clamp/`](printed-parts/n20_clamp/) | Two-piece clamp that holds each N20 motor to the base plate (OpenSCAD + STL) |
| [`printed-parts/spacers/`](printed-parts/spacers/) | Plate spacers and snap pins, three versions (OpenSCAD + STL) |
| [`laser-cut/base_plate/`](laser-cut/base_plate/) | Laser-cut base plate (DXF, 1:1 check sheet, Python generator) |
| [`concepts/face_shell/`](concepts/face_shell/) | Early concept renders of a shell around the OLED face (not printed) |
| [`fusion360/python/CatRobot.py`](fusion360/python/CatRobot.py) | Fusion 360 script that builds the first full-robot model (cat shell design) |
| [`fusion360/cpp/`](fusion360/cpp/) | C++ add-in version of that script (untested; Autodesk has deprecated the C++ API) |
| `exports/` | Exports of the full-robot model |

## 3D-printed parts on the robot

| File | Part | Notes |
|---|---|---|
| [`front_faceplate_v4_bobot.f3d`](printed-parts/fusion360/front_faceplate_v4_bobot.f3d) | Front faceplate with the OLED window and the BO-BOT name | Current version, printed in TPU |
| [`front_faceplate_v1_bobot.f3d`](printed-parts/fusion360/front_faceplate_v1_bobot.f3d) | First faceplate | Replaced by v4 |
| [`face_stand_v2_bobot.f3d`](printed-parts/fusion360/face_stand_v2_bobot.f3d) | Stand that holds the faceplate | |
| [`plate_stand_v2_bobot.f3d`](printed-parts/fusion360/plate_stand_v2_bobot.f3d) | Yellow side stands that hold the plates | |

G-code (PLA): [`plate_stand_v1_bobot_PLA_5m0s.gcode`](printed-parts/gcode/plate_stand_v1_bobot_PLA_5m0s.gcode),
[`plate_stand_v2_bobot_PLA_18m11s.gcode`](printed-parts/gcode/plate_stand_v2_bobot_PLA_18m11s.gcode),
[`plate_stand_v2_bobot_PLA_22m52s.gcode`](printed-parts/gcode/plate_stand_v2_bobot_PLA_22m52s.gcode).
The time in each file name is the print time.

## Laser-cut base plate

89 x 62 mm, acrylic (the drawing is for 3 mm sheet). Four Ø3.4 mm corner holes (76 x 46 mm centres), four 7 mm slots for the motor clamps, a wire slot,
and (v2) two windows for the motor encoders. Wheel track with the 34 mm wheels: 101.5 mm.
`make_base.py` (needs `ezdxf` and `matplotlib`) redraws the DXF and the 1:1 check sheet from its parameters. Print the PDF at 100 %
and check it against the real motors before cutting.

## Full-robot model script

1. Fusion 360, new parametric design (history on).
2. Shift+S, Scripts, + , Create Script (Python), paste `CatRobot.py`, Run.
3. Hide `Shell_Upper_Head` / `Shell_Lower` with the lightbulb to see inside.
4. File, Export, save into `exports/`.
