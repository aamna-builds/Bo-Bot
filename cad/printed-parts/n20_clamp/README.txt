GA12-N20 (encoder) two-piece motor clamp for the BO-BOT base plate
=====================================================================
Files
  n20_clamp_set.stl     print this: 2 x TOP + 2 x BOTTOM (one clamp per motor)
  n20_clamp_top.stl     single top saddle
  n20_clamp_bottom.stl  single bottom cap (hex nut pockets on its underside)
  n20_clamp.scad        OpenSCAD source - change motor_w / motor_h / fit if needed
  assembly.png          exploded view

Print
  PLA or PETG, 0.2 mm layers, 4 walls, 40 % infill, NO supports.
  Lay each part flat with the motor channel facing UP (as in the STL).

Hardware (per motor)
  2 x M3 x 20 screw (M3 x 18 ends flush), 2 x M3 nut

Assembly (per motor)
  1. Press 2 M3 nuts into the hex pockets of the BOTTOM cap.
  2. Screw from the TOP of the base plate: plate -> TOP saddle (channel facing down)
     -> motor -> BOTTOM cap (nut pockets facing down). Leave loose.
  3. Slide the motor so its encoder end is ~4 mm from the plate centre line
     (encoders ~8 mm apart), gearbox towards the wheel.
  4. Slide the clamp in the plate slots so it sits on the motor body
     (gearbox + can), NOT on the encoder board. Tighten evenly.
     The two halves should NOT touch (~0.5 mm gap) - that gap is the grip.

Fit check
  Motor drops in loosely?  -> reduce fit (0.35 -> 0.2) or squeeze (0.25 -> 0.4)
  Too tight to push in?    -> increase fit (0.35 -> 0.5)
  Default body size 12.0 x 10.0 mm: measure yours across the gearbox.
