Snap-in plate standoff pins v2 - BO-BOT (two plates, 17 mm apart, 4 corner M3 holes)
==========================================================================================
Files
  plate_pin_snap_pin.stl    ONE snap pin - print this first to test the fit
  plate_pin_snap_set.stl    6 snap pins (4 + 2 spares)               <- main choice
  plate_pin_plain_set.stl   6 solid-peg pins (heat-stake or caps), backup
  plate_pin_caps.stl        domed press-on caps for the solid pins (2.9 / 3.1 mm rows)
  plate_pin.scad            OpenSCAD source (all sizes at the top)
  pin.png, assembly.png     pictures

Shape
  Round waisted column, 17 mm between the seats, 9.6 mm round seats with
  rounded rims that sit flat on each plate. Each end has a split peg that
  slides through the 3.4 mm hole; two barbs spring out on the far side and
  lock the plate. No screws, glue or melting.

Print
  PLA or PETG (PETG is springier = better for snaps), 0.2 mm layers or finer,
  100 % infill, NO supports, exactly as laid out (lying on its side).
  The side touching the bed is flat - that is normal and does not matter.
  Use a brim if the small pegs lift.

Fit check with ONE pin
  - Push a peg through a plate hole: firm push, then a click. Barbs grip the
    plate with no rattle.
  - Too hard to push in / prong snapped  -> barb 0.45 -> 0.35, or file the barbs
  - Clicks in but rattles                -> grip 0.2 -> 0.35 (pulls the plate tighter)
  - Plate not 3.0 mm thick               -> set plate_t to your thickness
  Re-export from OpenSCAD (free) or ask for new STLs.

Assembly
  1. Click 4 pins into the bottom plate's corner holes.
  2. Line the top plate up over the 4 pegs and press it down evenly,
     one corner at a time, until all 4 click.
  To take it apart: pinch the two prongs of a peg together with
  pliers and pull the plate off.
