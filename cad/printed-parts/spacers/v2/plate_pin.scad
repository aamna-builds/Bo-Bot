// =====================================================================
//  Snap-in plate standoff pin, v2  (BalanceBot: two plates 17 mm apart)
// =====================================================================
//  Round, waisted column with a wide seat at each end and a split snap peg
//  that clicks through the plate's M3 hole (3.4 mm). No screws, glue or caps.
//
//        barbs spring out past the plate
//             \/
//      ======|  |======   plate
//         ___|  |___      wide round seat
//         \        /
//          |      |       waisted column, gap = 17 mm
//         /________\
//      ======|  |======   plate
//             /\
//
//  Printed lying on its side (small flat underneath, no supports), so the
//  layer lines run ALONG the pegs and they can flex without snapping.
//  All sizes in mm.
// =====================================================================

part  = "set";      // "pin", "set" (6 pins), "caps", "assembly"
style = "snap";     // "snap" = click-in barbs,  "plain" = solid peg (heat-stake or cap)

gap      = 17.0;    // space between the plates
plate_t  = 3.0;     // plate thickness  <-- measure yours
hole_d   = 3.4;     // hole in the plate
peg_d    = 3.2;     // peg diameter (0.1 clearance all round)
flat     = 0.15;    // flat under the peg for bed contact
flange_d = 9.6;     // seat diameter against the plate
waist_d  = 6.4;     // column diameter in the middle
barb     = 0.45;    // how far each barb sticks out sideways
slot_w   = 1.0;     // slot between the two prongs
relief   = 3.5;     // prongs start this deep inside the column (longer = springier)
relief_gap = 0.6;   // free space around the prong roots
lead     = 2.6;     // length of the tapered tip
grip     = 0.2;     // barb ramp starts this far before the plate's far face (preload)

r      = peg_d / 2;
axis_z = r - flat;
$fn = 48;

// ---- column: flange -> smooth fillet -> waist, with rounded rims -------------
flange_len = 1.4; fillet_len = 4.5; edge_r = 0.6; N = 80;
function blend(t) = (1 - cos(180 * min(max(t, 0), 1))) / 2;
function rad(x) = let (d = min(x, gap - x))
    d < edge_r    ? flange_d/2 - edge_r + sqrt(edge_r*edge_r - (edge_r - d)*(edge_r - d)) :
    d < flange_len ? flange_d/2 :
    flange_d/2 - (flange_d - waist_d)/2 * blend((d - flange_len) / fillet_len);

module column() {
    rotate([0, 90, 0]) rotate_extrude($fn = 96)
        polygon(concat([[0, 0]], [for (i = [0:N]) [rad(i * gap / N), i * gap / N]], [[0, gap]]));
}

// ---- peg: chain of elliptical sections along +X (ry sideways, rz up/down) ----
module disc(x, ry, rz) translate([x, 0, 0]) rotate([0, 90, 0]) scale([rz, ry, 1]) cylinder(r = 1, h = 0.01);
module chain(st) for (i = [0 : len(st) - 2]) hull() { disc(st[i][0], st[i][1], st[i][2]); disc(st[i+1][0], st[i+1][1], st[i+1][2]); }

snap_st  = [[-relief, r, r], [plate_t - grip, r, r], [plate_t + 0.3, r + barb, r],
            [plate_t + 0.7, r + barb, r], [plate_t + 0.7 + lead, r - 0.35, r - 0.45]];
plain_st = [[-1, r, r], [plate_t + 2.0, r, r], [plate_t + 2.6, r - 0.5, r - 0.5]];
peg_len  = style == "snap" ? plate_t + 0.7 + lead : plate_t + 2.6;

module peg()    { chain(style == "snap" ? snap_st : plain_st); }
module pocket() { translate([-relief, 0, 0]) rotate([0, 90, 0]) cylinder(r = r + relief_gap, h = relief + 0.01); }
module slot()   { hull() for (x = [-relief + 0.9, peg_len + 1]) translate([x, 0, -(r + relief_gap - 0.1)])
                      cylinder(d = slot_w, h = 2 * (r + relief_gap - 0.1), $fn = 24); }

module ends() { translate([gap, 0, 0]) children(); mirror([1, 0, 0]) children(); }

module pin() {
    intersection() {
        translate([0, 0, axis_z]) difference() {
            union() {
                difference() { column(); if (style == "snap") ends() pocket(); }
                ends() peg();
            }
            if (style == "snap") ends() slot();
        }
        translate([-30, -30, 0]) cube([gap + 60, 60, 30]);     // flat on the bed
    }
}

// ---- domed press-on cap (only for the "plain" style) --------------------------
module cap(hole) {
    difference() {
        hull() { cylinder(d = 7.5, h = 0.8); translate([0, 0, 1.4]) scale([1, 1, 0.45]) sphere(d = 7.5); }
        translate([0, 0, -1]) cylinder(d = hole, h = 3.2);
        translate([0, 0, -0.01]) cylinder(d1 = hole + 0.8, d2 = hole, h = 0.5);
        translate([0, 0, -5]) cube([20, 20, 10], center = true);
    }
}

if (part == "pin") pin();
else if (part == "set") for (i = [0:5]) translate([0, i * (flange_d + 3), 0]) pin();
else if (part == "caps") for (j = [0:1], i = [0:5]) translate([i * 10.5, j * 12, 0]) cap([2.9, 3.1][j]);
else if (part == "assembly") {
    color("tan", 0.85) for (z = [-plate_t, gap]) translate([-14, -14, z]) difference() {
        cube([28, 28, plate_t]); translate([14, 14, -1]) cylinder(d = hole_d, h = plate_t + 2); }
    color("orange") rotate([0, -90, 0]) translate([0, 0, -axis_z]) pin();
}
