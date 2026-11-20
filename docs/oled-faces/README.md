# OLED face previews

The Uno draws its faces without a screen buffer, so the drawing code was checked on a PC before it went on the robot.

| Path | Contents |
|---|---|
| [`face16/faces16.png`](face16/faces16.png) | All 16 faces as the 128 x 64 OLED shows them |
| [`face16/h.cpp`](face16/h.cpp) | PC harness: compiles the face-drawing code from the firmware and dumps every frame |
| [`face16/oled_block.ino`](face16/oled_block.ino) | The OLED and face part of the 16-face firmware, as used by the harness |
| [`face16/out.txt`](face16/out.txt) | Raw byte output of the harness |
| [`face_preview/faces.png`](face_preview/faces.png), [`prev.py`](face_preview/prev.py) | Preview of the first six faces (Python) |

![All 16 faces](face16/faces16.png)

Face list and when each one shows: [`../../firmware/uno/README.md`](../../firmware/uno/README.md#oled-faces).
