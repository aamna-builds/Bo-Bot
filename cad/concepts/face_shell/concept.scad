// CONCEPT ONLY - rough shapes to show the idea (not the print file)
// z = 0 : top of the middle plate,  +y = robot front,  x = along the axle
view = "A";          // "A" shell on robot, "Aback" shell from behind, "B" floating face, "Aside"
show_robot = true;
$fn = 40;

W = 80; H = 48; D = 26; WALL = 2.2;
YF = 33; YB = YF - D;               // front face / back edge of the shell
RT = 16; RB = 8;                    // top / bottom corner radius
VIS_W = 50; VIS_H = 30; VIS_Z = 27; // dark visor
WHITE = [0.97,0.97,0.96]; BLACK = [0.07,0.07,0.09]; BLUE = [0.35,0.75,0.97];
CYAN = [0.55,0.95,1]; ACR = [0.80,0.88,0.95,0.55];

module head_solid(o = 0, ext = 0) {
    intersection() {
        hull() {
            for (sx = [-1, 1]) {
                translate([sx*(W/2 - RT), YF - RT, H - RT]) sphere(r = RT - o);
                translate([sx*(W/2 - RB), YF - RB, 0]) sphere(r = RB - o);
                translate([sx*(W/2 - RB), YF - RB, -10]) sphere(r = RB - o);
                translate([sx*(W/2 - RT + 2), YF - 13, H*0.55]) sphere(r = 13 - o);
            }
        }
        translate([-60, YB - ext, -ext*5]) cube([120, 60, 100]);
        translate([-60, -40, 0 - ext*5]) cube([120, 120, 100]);
    }
}
module ear(sx) {
    // rounded cat ear on top, leaning out
    color(WHITE) hull() {
        translate([sx*14, YF - 12, H - 6]) scale([1, 0.55, 1]) sphere(r = 7);
        translate([sx*30, YF - 12, H - 9]) scale([1, 0.55, 1]) sphere(r = 6);
        translate([sx*27, YF - 12, H + 13]) sphere(r = 2.2);
    }
    color(BLUE) hull() {
        translate([sx*18, YF - 8.4, H - 2]) scale([1, 0.2, 1]) sphere(r = 3.5);
        translate([sx*27, YF - 8.4, H - 3]) scale([1, 0.2, 1]) sphere(r = 3);
        translate([sx*26, YF - 8.4, H + 9]) scale([1, 0.2, 1]) sphere(r = 1);
    }
}
module rrect(w, h, r, t) hull() for (sx = [-1, 1], sz = [-1, 1]) translate([sx*(w/2 - r), 0, sz*(h/2 - r)]) rotate([90, 0, 0]) cylinder(r = r, h = t);
module face_dots(cz, y) {    // happy face on the OLED
    color(CYAN) for (sx = [-1, 1]) for (a = [20:32:160])
        translate([sx*5.5 + 3.6*cos(a), y, cz + 2 + 3.2*sin(a)]) sphere(r = 0.55, $fn = 8);
    color(CYAN) for (a = [200:28:340]) translate([4.5*cos(a), y, cz - 1 + 3.2*sin(a)]) sphere(r = 0.55, $fn = 8);
}
module oled_module(cz, y) {   // 0.96" 128x64: PCB 27.3 x 27.8, glass ~26.7 x 19.3
    translate([0, y, cz]) {
        color([0.1,0.15,0.35]) translate([-13.65, -3.2, -13.9]) cube([27.3, 1.6, 27.8]);
        color(BLACK) translate([-13.35, -1.6, -11.2]) cube([26.7, 1.4, 19.3]);
        color([0.85,0.7,0.2]) for (i = [0:3]) translate([-3.81 + i*2.54, -3.2, 11.5]) rotate([90,0,0]) cylinder(d = 0.7, h = 7, $fn = 6);
    }
}
OLED_Z = VIS_Z + 1;
module shell_A() {
    difference() {
        color(WHITE) head_solid();
        color(WHITE) head_solid(WALL, 1);
        // visor recess + screen window
        color(BLACK) translate([0, YF + 1, VIS_Z]) rrect(VIS_W, VIS_H, 7, 1.8);
        color(BLACK) translate([0, YF + 1, OLED_Z + 1.5]) rrect(24, 13, 1.5, 6);
        // whisker grooves
        for (sx = [-1, 1], k = [-1, 0, 1]) color([0.75,0.75,0.75])
            translate([sx*33, YF + 0.5, 17 + k*3.2]) rotate([0, sx*k*12, 0]) rotate([90,0,0]) hull() { translate([-4,0,0]) cylinder(d = 1, h = 1.2, $fn = 8); translate([4,0,0]) cylinder(d = 1, h = 1.2, $fn = 8); }
    }
    color(BLACK) translate([0, YF - 0.75, VIS_Z]) difference() { rrect(VIS_W, VIS_H, 7, 0.3); translate([0, 1, 1.5]) rrect(24, 13, 1.5, 3); }
    for (sx = [-1, 1]) ear(sx);
    // OLED pocket (side rails) behind the window
    color([0.9,0.9,0.88]) for (sx = [-1, 1]) translate([sx*15.2 - 1.2, YF - 7.5, OLED_Z - 15]) cube([2.4, 5.3, 28]);
    color([0.9,0.9,0.88]) translate([-15.2, YF - 7.5, OLED_Z - 16]) cube([30.4, 5.3, 2]);
    oled_module(OLED_Z, YF - 2.2 - 0.2);
    // tool-free feet: socket over the pin tip + hook under the plate edge
    for (sx = [-1, 1]) color([0.88,0.88,0.86]) translate([sx*38, 0, 0]) difference() {
        union() {
            translate([-5, 16.5, 0]) cube([10, YF - 16.5, 5]);
            translate([-5, 31.1, -6.4]) cube([10, YF - 31.1, 6.4]);
            translate([-5, 28.6, -6.4]) cube([10, YF - 28.6, 2.2]);
        }
        translate([0, 23, -1]) cylinder(d = 6.2, h = 4.4);
    }
}
// ---------------- robot context ----------------
module plate(z) color(ACR) difference() {
    translate([0, 0, z]) hull() for (sx = [-1,1], sy = [-1,1]) translate([sx*39.5, sy*26, 0]) cylinder(r = 5, h = 4);
    for (sx = [-1,1], sy = [-1,1]) translate([sx*38, sy*23, z - 1]) cylinder(d = 3.4, h = 6);
}
module robot() {
    plate(-4); plate(-25);
    for (sx = [-1,1], sy = [-1,1]) color(WHITE) translate([sx*38, sy*23, -21]) cylinder(d1 = 9, d2 = 9, h = 17);
    for (sx = [-1,1], sy = [-1,1]) color(WHITE) translate([sx*38, sy*23, 0]) cylinder(d = 4.4, h = 3.3);
    color([0.15,0.35,0.85]) translate([-34, -18.5, -21]) cube([68, 37, 12]);   // battery
    for (sx = [-1,1]) {
        color([0.75,0.75,0.78]) translate([sx*4, 0, -32]) rotate([0, sx*90, 0]) cylinder(d = 11, h = 28);
        color([0.8,0.65,0.3]) translate([sx*32, 0, -32]) rotate([0, sx*90, 0]) cube([12, 10, 9], center = true);
        color([0.12,0.12,0.12]) translate([sx*50.75, 0, -32]) rotate([0, 90, 0]) cylinder(d = 34, h = 6.5, center = true, $fn = 60);
        color(WHITE) translate([sx*(50.75 + sx*0.5), 0, -32]) rotate([0, 90, 0]) cylinder(d = 22, h = 6.6, center = true, $fn = 40);
    }
    color([0,0.45,0.55]) translate([-34.3, -31, 0]) cube([68.6, 45, 1.6]);   // Uno area (assumed) behind the face
}
// ---------------- option B: floating face on two legs ----------------
module shell_B() {
    BW = 52; BH = 42; BD = 13; BZ = 20;
    difference() {
        color(WHITE) hull() for (sx = [-1, 1]) {
            translate([sx*(BW/2 - 12), YF - 12, BZ + BH - 12]) sphere(r = 12);
            translate([sx*(BW/2 - 7), YF - 7, BZ + 7]) sphere(r = 7);
        }
        color(WHITE) translate([-40, -20, 0]) cube([80, YF - BD + 20, 100]);
        color(BLACK) translate([0, YF + 1, BZ + BH/2]) rrect(36, 26, 5, 1.6);
        color(BLACK) translate([0, YF + 1, BZ + BH/2 + 1.5]) rrect(24, 13, 1.5, 6);
    }
    color(BLACK) translate([0, YF - 0.6, BZ + BH/2]) difference() { rrect(36, 26, 5, 0.3); translate([0, 1, 1.5]) rrect(24, 13, 1.5, 3); }
    for (sx = [-1, 1]) {
        color(WHITE) hull() { translate([sx*10, YF - 7, BZ + BH - 6]) scale([1,0.5,1]) sphere(r = 6); translate([sx*22, YF - 7, BZ + BH - 8]) scale([1,0.5,1]) sphere(r = 5); translate([sx*20, YF - 7, BZ + BH + 10]) sphere(r = 2); }
        color(BLUE) hull() { translate([sx*13, YF - 4.2, BZ + BH - 3]) scale([1,0.2,1]) sphere(r = 3); translate([sx*20, YF - 4.2, BZ + BH - 3]) scale([1,0.2,1]) sphere(r = 2.5); translate([sx*19.5, YF - 4.2, BZ + BH + 6]) scale([1,0.2,1]) sphere(r = 0.8); }
        color([0.92,0.92,0.9]) hull() { translate([sx*38, 23, 0]) cylinder(d = 9, h = 5); translate([sx*20, YF - 7, BZ + 4]) rotate([90,0,0]) cylinder(d = 7, h = 6, center = true); }
        color([0.92,0.92,0.9]) translate([sx*38 - 5, 28.6, -6.4]) cube([10, YF - 28.6 - 1, 2.2]);
        color([0.92,0.92,0.9]) translate([sx*38 - 5, 31.1, -6.4]) cube([10, 1.9, 11]);
    }
    face_dots(BZ + BH/2 + 1.5, YF - 1.7);
}
if (view == "A" || view == "Aside") { if (show_robot) robot(); shell_A(); face_dots(OLED_Z + 1.5, YF - 2.3); }
if (view == "Aback") { shell_A(); face_dots(OLED_Z + 1.5, YF - 2.3); plate(-4); for (sx=[-1,1], sy=[-1,1]) color(WHITE) translate([sx*38, sy*23, 0]) cylinder(d = 4.4, h = 3.3); }
if (view == "B") { if (show_robot) robot(); shell_B(); }

// ---------------- dimension marks (concept) ----------------
DIM = [0.85,0.15,0.15];
module seg(a, b, d = 0.6) color(DIM) hull() { translate(a) sphere(d = d, $fn = 8); translate(b) sphere(d = d, $fn = 8); }
module lab(t, p, rot, sz = 4) color(DIM) translate(p) rotate(rot) linear_extrude(0.2) text(t, size = sz, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
GND = -49;
module dims_front() {
    y = 70; R = [90, 0, 180];
    seg([-40, y, -54], [40, y, -54]); seg([-40, y, -57], [-40, y, -51]); seg([40, y, -57], [40, y, -51]);
    seg([-40, y, -51], [-40, y, 0], 0.25); seg([40, y, -51], [40, y, 0], 0.25);
    lab("shell 80 mm wide", [0, y, -59.5], R);
    seg([50, y, 0], [50, y, H]); seg([47, y, 0], [53, y, 0]); seg([47, y, H], [53, y, H]);
    lab("48", [56, y, H/2], R);
    seg([62, y, 0], [62, y, H + 15]); seg([59, y, H + 15], [65, y, H + 15]); seg([59, y, 0], [65, y, 0]);
    lab("~63", [70, y, H/2 + 7], R);
    seg([-8, y, OLED_Z + 3], [-40, y, OLED_Z + 30]);
    lab("OLED window 24 x 13", [-50, y, OLED_Z + 34], R, 3.5);
    seg([-25, y, VIS_Z - 12], [-46, y, VIS_Z - 20]);
    lab("dark visor 50 x 30", [-60, y, VIS_Z - 24], R, 3.5);
    seg([-38, y, -4], [-50, y, -14]);
    lab("clip feet on the", [-60, y, -18], R, 3.2);
    lab("2 front holes", [-60, y, -22.5], R, 3.2);
}
module dims_side() {
    x = 75; yl = -42; R = [90, 0, 90];
    for (z = [GND, 0, OLED_Z + 1.5, H + 15]) seg([x, yl - 3, z], [x, yl + 3, z]);
    seg([x, yl, GND], [x, yl, H + 15]);
    for (z = [0, OLED_Z + 1.5, H + 15]) seg([x, yl + 3, z], [x, YB, z], 0.3);
    module tl(t, z) color(DIM) translate([x, yl - 5, z]) rotate(R) linear_extrude(0.2) text(t, size = 4, halign = "right", valign = "center", font = "Liberation Sans:style=Bold");
    tl("ground", GND); tl("plate top  49 mm", 0); tl("OLED centre  ~79 mm", OLED_Z + 1.5); tl("ear tips  ~112 mm", H + 15);
    seg([x, YB, H + 22], [x, YF, H + 22]); seg([x, YB, H + 19], [x, YB, H + 25]); seg([x, YF, H + 19], [x, YF, H + 25]);
    lab("26 deep", [x, YB + 13, H + 27], R);
    lab("FRONT  >", [x, 48, -20], R, 4);
}
if (view == "Afront") { robot(); shell_A(); face_dots(OLED_Z + 1.5, YF - 2.3); dims_front(); }
if (view == "Aside2") { robot(); shell_A(); dims_side(); }
if (view == "clip") {
    intersection() {
        union() {
            shell_A();
            plate(-4);
            color(WHITE) translate([38, 23, -21]) cylinder(d = 9, h = 17);
            color(WHITE) translate([38, 23, -5.4]) cylinder(d = 10.5, h = 1.4);
            color([0.98,0.98,0.95]) translate([38, 23, -4]) cylinder(d = 3.6, h = 4);
            color([0.98,0.98,0.95]) translate([38, 23, 0]) cylinder(d1 = 4.5, d2 = 3.2, h = 3.3);
        }
        translate([0, 5, -24]) cube([38, 40, 40]);
    }
    translate([38.2, 0, 0]) {
        lab("socket fits over the", [0, 6, 13], [90, 0, 90], 2.6);
        lab("snap-pin tip", [0, 6, 9.5], [90, 0, 90], 2.6);
        seg([0, 12, 7.5], [0, 21.5, 2.5], 0.3);
        lab("hook under the", [0, 46, -9], [90, 0, 90], 2.6);
        lab("plate's front edge", [0, 46, -12.5], [90, 0, 90], 2.6);
        seg([0, 38, -8], [0, 32.5, -5.5], 0.3);
        lab("middle plate (4 mm)", [0, 6, -2], [90, 0, 90], 2.6);
        lab("pin from below", [0, 12, -16], [90, 0, 90], 2.6);
    }
}
