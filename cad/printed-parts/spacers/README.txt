Screwless plate spacers - BO-BOT (two plates, 17 mm apart, through the 4 corner M3 holes)
==========================================================================================
Print first:  plate_spacer_test.stl   (1 spacer + 2 test caps) -> check the fit
Then:         plate_spacer_set.stl    (6 spacers: 4 + 2 spares)
Optional:     plate_spacer_caps.stl   (12 caps: back row 2.9 mm hole, front row 3.1 mm hole)

Print: PLA or PETG, 0.2 mm layers, 100 % infill, NO supports, exactly as laid out
(spacers lying on their side - this makes the pegs strong).

Fit check (with the test print)
  - Peg must slide through a plate hole. Too tight -> lightly sand/file it,
    or change peg_d 3.1 -> 2.9 in plate_spacer.scad.
  - Gap between the plates = 17 mm. Plates assumed 3 mm thick: if yours differ,
    change plate_t and re-export (OpenSCAD).

Assembly
  1. Push 4 spacers into the bottom plate's corner holes. Turn each one so the
     wide part of the block points INTO the plate, not over the edge.
  2. Lay the top plate on, pegs through its 4 holes.
  3. Lock every peg end (8 in total), pick one:
     A) HEAT-STAKE (strongest, no extra parts): soldering iron ~250 C,
        put a scrap of baking paper on the peg tip, press down 2-3 s until it
        spreads into a flat head ~5 mm wide. On ACRYLIC plates keep the iron
        off the plate itself.
     B) CAPS: press a cap onto the peg (try the 2.9 row first, then 3.1).
        Add a drop of super glue if it is loose.
     C) A blob of hot glue over each peg end.
  Tip: lock the BOTTOM ends permanently (A), and use caps (B) on top
  so you can still open the robot later.
