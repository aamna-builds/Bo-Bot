// =====================================================================
//  GA12-N20 (with encoder) two-piece clamp for the BalanceBot base plate
// =====================================================================
//  Holds the motor BODY (gearbox + can, 12 x 10 mm) between two halves
//  squeezed by two M3 screws. The encoder end stays free.
//
//   base plate (3 mm)       ===============================
//   TOP saddle (bolts up)   [##]   ___________   [##]
//                           [##]  |  motor    |  [##]   <- M3 screws 20 mm apart,
//   BOTTOM cap (nut traps)  [##]  |___________|  [##]      one each side of the motor
//
//  Print: 2 x TOP + 2 x BOTTOM, flat side on the bed, channel facing up.
//  PLA or PETG, 0.2 mm layers, 4 walls, 40 % infill, no supports.
//  Screws: M3 x 20 (from the top of the plate) + M3 nuts in the cap.
//  All sizes in mm. Change the numbers below if your motor measures differently.
// =====================================================================

part = "set";            // "top", "bottom", "set" (2 tops + 2 bottoms), "assembly"

motor_w   = 12.0;        // motor body width  (measure across the gearbox)
motor_h   = 10.0;        // motor body height (across the flats)
fit       = 0.35;        // extra room in the channel width (FDM)
squeeze   = 0.25;        // each half is this much shallower than half the motor -> grip
L         = 20.0;        // clamp length along the motor
hole_sp   = 20.0;        // M3 hole spacing = base plate slots
B         = 28.0;        // clamp width (across the motor)
wall      = 2.5;         // material above / below the motor
m3        = 3.4;         // M3 clearance hole
nut_af    = 5.6;         // M3 nut across flats (+ a little room)
nut_depth = 2.6;         // nut trap depth in the cap
chamfer   = 0.6;

half_d = motor_h / 2 - squeeze;            // channel depth in each half
H      = wall + half_d;                    // height of each half
ch_w   = motor_w + fit;

module half(nut_trap = false) {
    difference() {
        // body with chamfered vertical edges
        hull() for (x = [-B/2 + chamfer, B/2 - chamfer], y = [-L/2 + chamfer, L/2 - chamfer])
            translate([x, y, 0]) cylinder(r = chamfer, h = H, $fn = 16);
        // motor channel (open on the top face of this half)
        translate([-ch_w/2, -L/2 - 1, wall]) cube([ch_w, L + 2, half_d + 1]);
        // lead-in chamfer along the channel edges
        for (s = [-1, 1]) translate([s * ch_w/2, 0, H]) rotate([90, 0, 0])
            cylinder(r = 0.6, h = L + 2, center = true, $fn = 4);
        // M3 holes
        for (s = [-1, 1]) translate([s * hole_sp/2, 0, -1]) cylinder(d = m3, h = H + 2, $fn = 32);
        // hex nut traps on the outside face of the cap
        if (nut_trap) for (s = [-1, 1]) translate([s * hole_sp/2, 0, -0.01])
            rotate([0, 0, 30]) cylinder(d = nut_af / cos(30), h = nut_depth, $fn = 6);
    }
}

module top()    { half(false); }
module bottom() { half(true); }

module motor_dummy() {   // for the assembly view only
    color("silver") translate([0, 0, 0]) cube([motor_w, 24, motor_h], center = true);
    color("green") translate([0, -12 - 1, 0]) cube([12, 2, 10], center = true);   // encoder board
    color("gray") translate([0, 12, 0]) rotate([-90, 0, 0]) cylinder(d = 3, h = 10, $fn = 24);
}

if (part == "top") top();
else if (part == "bottom") bottom();
else if (part == "set") {
    for (i = [0, 1]) {
        translate([i * (B + 6), 0, 0]) top();
        translate([i * (B + 6), L + 6, 0]) bottom();
    }
}
else if (part == "assembly") {
    // exploded: plate strip, TOP saddle (channel down), motor, BOTTOM cap (nuts down)
    e = 9;   // explode distance
    color("tan") translate([-B/2 - 6, -L/2 - 4, H + 2*e]) cube([B + 12, L + 8, 3]);
    color("orange") translate([0, 0, H + e]) mirror([0, 0, 1]) top();
    rotate([0, 0, 0]) motor_dummy();
    color("orange") translate([0, 0, -H - e]) bottom();
    color("gray") for (s = [-1, 1]) translate([s * hole_sp/2, 0, -H - e - 3]) cylinder(d = 2.8, h = 2*H + 3*e + 9, $fn = 16);
    color("dimgray") for (s = [-1, 1]) translate([s * hole_sp/2, 0, H + 2*e + 3]) cylinder(d = 5.5, h = 2.5, $fn = 24);
}
