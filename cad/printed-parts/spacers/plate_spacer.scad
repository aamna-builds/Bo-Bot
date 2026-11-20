// =====================================================================
//  Screwless plate spacer for the BalanceBot (two plates, 17 mm apart)
// =====================================================================
//  A square spacer block with a peg on each end. The pegs go through the
//  M3 holes (3.4 mm) of both plates and stick out 2.5 mm. Lock each peg:
//    A) heat-stake: press the tip flat with the soldering iron -> rivet head
//    B) press a CAP on it (+ a drop of super glue if loose)
//    C) or simply a blob of hot glue
//
//      cap / melted head  ==o==
//      top plate (3 mm)  ======|=|======
//                              | |  <- peg 3.1 mm
//      spacer block           [###]  17 mm gap
//                             [###]
//      bottom plate      ======|=|======
//      cap / melted head  ==o==
//
//  Printed LYING DOWN (pegs horizontal) so the layer lines run along the
//  pegs -> they don't snap off. Pegs have a small flat underneath for the bed.
//  PLA/PETG, 0.2 mm layers, 100 % infill (they're tiny), no supports.
// =====================================================================

part      = "set";   // "spacer", "set" (6 spacers), "caps" (12 caps, 2 sizes), "test", "assembly"

gap       = 17.0;    // space between the plates
plate_t   = 3.0;     // plate thickness (measure yours!)
hole_d    = 3.4;     // plate hole
peg_d     = 3.1;     // peg diameter (0.3 smaller than the hole)
protrude  = 2.5;     // how far the peg sticks out past the plate (for the head/cap)
block     = 7.0;     // spacer block cross-section (7 x 7 mm)
flat      = 0.3;     // flat on the underside of the pegs for bed adhesion
cap_od    = 7.0;
cap_t     = 2.4;
cap_holes = [2.9, 3.1];   // two cap sizes: use whichever press-fits snugly

peg_len   = plate_t + protrude;
$fn = 40;

module peg(len) {
    // horizontal along +X, bottom touching z = 0
    difference() {
        translate([0, 0, peg_d/2]) rotate([0, 90, 0]) cylinder(d = peg_d, h = len);
        translate([-1, -peg_d, -10 + flat]) cube([len + 2, 2*peg_d, 10]);    // flat bottom
        // tip chamfer
        translate([len, 0, peg_d/2]) rotate([0, 90, 0])
            difference() { cylinder(d = peg_d + 2, h = 1); translate([0, 0, -0.01]) cylinder(d1 = peg_d, d2 = peg_d - 1.2, h = 0.6); }
    }
}

module spacer() {
    // block lying on the bed, pegs aligned with its bottom edge so no supports are needed
    translate([0, -block/2, 0]) cube([gap, block, block]);
    translate([gap - 0.01, 0, 0]) peg(peg_len);
    mirror([1, 0, 0]) translate([0.01, 0, 0]) peg(peg_len);
}

module cap(hole) {
    difference() {
        cylinder(d = cap_od, h = cap_t);
        translate([0, 0, -1]) cylinder(d = hole, h = cap_t + 2);
        translate([0, 0, cap_t - 0.5]) cylinder(d1 = hole, d2 = hole + 1.0, h = 0.51);   // lead-in
    }
}

if (part == "spacer") spacer();
else if (part == "set") for (i = [0:5]) translate([0, i * (block + 4), 0]) spacer();
else if (part == "caps") {
    for (j = [0:1]) for (i = [0:5]) translate([i * (cap_od + 3), j * (cap_od + 6), 0]) cap(cap_holes[j]);
    // size marks: 1 notch = 2.9 mm row, 2 notches = 3.1 mm row (cut into the bed-side edge)
}
else if (part == "test") {
    spacer();
    for (i = [0:1]) translate([4 + i * (cap_od + 3), block + 6, 0]) cap(cap_holes[i]);
}
else if (part == "assembly") {
    // pegs vertical, two plates
    color("tan") translate([-12, -12, 0]) cube([24, 24, plate_t]);
    color("tan") translate([-12, -12, plate_t + gap]) cube([24, 24, plate_t]);
    color("orange") translate([0, 0, plate_t + gap]) rotate([0, 90, 0]) translate([0, 0, -peg_d/2])
        translate([0, 0, 0]) spacer();
    color("royalblue") translate([0, 0, -cap_t - 0.5]) cap(3.1);
    color("royalblue") translate([0, 0, 2*plate_t + gap + 0.5]) cap(3.1);
}
