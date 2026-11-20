// Snap-in plate standoff pin v3 - 4 mm plates, thicker pegs in selectable sizes.
// Printed lying on its side, no supports. All sizes in mm.
part     = "set";   // "set" (6 pins), "pin", "test" (4 short pins: 3.4 / 3.7 / 4.0 / 4.4 mm pegs)
peg_d    = 3.7;     // peg thickness: choose with the test print
gap      = 17.0;    // space between the plates
plate_t  = 4.0;     // plate thickness
flat     = 0.15;
flange_d = 10.5;    // seat diameter
waist_d  = 7.0;
barb     = 0.45;    // each barb sticks out this far
slot_w   = 1.0;
relief   = 3.5;
relief_gap = 0.6;
lead     = 2.6;
grip     = 0.3;     // preload: barbs pull the plate against the seat
$fn = 48;
flange_len = 1.4; fillet_len = 4.5; edge_r = 0.6; N = 80;
function blend(t) = (1 - cos(180 * min(max(t, 0), 1))) / 2;
function rad(x, g) = let (d = min(x, g - x), fl = min(fillet_len, g/2 - flange_len))
    d < edge_r ? flange_d/2 - edge_r + sqrt(edge_r*edge_r - (edge_r - d)*(edge_r - d)) :
    d < flange_len ? flange_d/2 :
    flange_d/2 - (flange_d - waist_d)/2 * blend((d - flange_len) / fl);
module column(g) rotate([0, 90, 0]) rotate_extrude($fn = 96)
    polygon(concat([[0, 0]], [for (i = [0:N]) [rad(i * g / N, g), i * g / N]], [[0, g]]));
module disc(x, ry, rz) translate([x, 0, 0]) rotate([0, 90, 0]) scale([rz, ry, 1]) cylinder(r = 1, h = 0.01);
module chain(st) for (i = [0 : len(st) - 2]) hull() { disc(st[i][0], st[i][1], st[i][2]); disc(st[i+1][0], st[i+1][1], st[i+1][2]); }
module peg(r, rel) chain([[-rel, r, r], [plate_t - grip, r, r], [plate_t + 0.3, r + barb, r],
                          [plate_t + 0.7, r + barb, r], [plate_t + 0.7 + lead, r - 0.35, r - 0.45]]);
module pin(pd, g) {
    r = pd / 2; rel = min(relief, g/2 - 0.8);
    intersection() {
        translate([0, 0, r - flat]) difference() {
            union() {
                difference() { column(g);
                    for (m = [0, 1]) translate([m * g, 0, 0]) mirror([1 - m, 0, 0])
                        translate([-rel, 0, 0]) rotate([0, 90, 0]) cylinder(r = r + relief_gap, h = rel + 0.01); }
                for (m = [0, 1]) translate([m * g, 0, 0]) mirror([1 - m, 0, 0]) peg(r, rel);
            }
            for (m = [0, 1]) translate([m * g, 0, 0]) mirror([1 - m, 0, 0])
                hull() for (x = [-rel + 0.9, plate_t + lead + 2]) translate([x, 0, -(r + relief_gap - 0.1)])
                    cylinder(d = slot_w, h = 2 * (r + relief_gap - 0.1), $fn = 24);
        }
        translate([-30, -30, 0]) cube([g + 60, 60, 30]);
    }
}
if (part == "pin") pin(peg_d, gap);
else if (part == "set") for (i = [0:5]) translate([0, i * (flange_d + 3), 0]) pin(peg_d, gap);
else if (part == "test") {   // shortest = thinnest peg
    sizes = [3.4, 3.7, 4.0, 4.4]; lens = [6, 8, 10, 12];
    for (i = [0:3]) translate([0, i * (flange_d + 3), 0]) pin(sizes[i], lens[i]);
}
