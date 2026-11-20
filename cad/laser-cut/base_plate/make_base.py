"""
BalanceBot base plate: laser-cut DXF + 1:1 check print.

Drawing coordinates (top view, looking down on the robot):
    u = along the axle (right of page = robot's left side... symmetric anyway)
    v = forward (top of page = FRONT of the robot, OLED side)
Origin = centre of the axle line.  Units: mm.

Every number is a parameter below; change and re-run.
"""
import math
import ezdxf
from ezdxf import units

# ------------------------------------------------------------------ inputs
MOTOR_PLUS_WHEEL = 50.0   # measured: motor inner end (encoder) -> wheel outer face
WHEEL_W          = 6.5    # tyre width
GAP_CENTER       = 8.0    # gap between the two encoder ends (wires + clearance)
TYRE_CLEAR       = 3.0    # plate edge to inner tyre face
PLATE_D          = 62.0   # front-to-back depth (small = big fall angle)
CORNER_R         = 5.0
M3               = 3.4    # M3 clearance hole
MOUNT_HOLE_SPACING = 20.0 # snap-fit mount: M3 holes ~20 mm apart, either side of the motor
MOUNT_ADJUST     = 7.0    # slot travel along the axle (+/-3.5 mm)
BODY_CENTER_FROM_INNER_END = 22.0  # motor body (gearbox+can) centre, from encoder end
STANDOFF_U, STANDOFF_V = 38.0, 23.0   # M3 holes for the upper decks
WIRE_SLOT = (18.0, 6.0, -24.5)        # width (u), height (v), centre v
BATTERY = (68.0, 37.0)                # u x v footprint (no longer engraved: see notes)
# Encoder windows: one rectangular cut-out above each motor's encoder end, so the
# encoder board + its 6-pin connector/cable can come up through the plate.
ENC_U0, ENC_U1 = 3.0, 16.0            # from 3 mm off the centre line to 16 mm out (13 mm long)
ENC_H          = 16.0                 # across (front-to-back), centred on the axle line
ENC_R          = 1.5                  # corner radius

# ------------------------------------------------------------------ derived
inner_end_u   = GAP_CENTER / 2
tyre_inner_u  = inner_end_u + MOTOR_PLUS_WHEEL - WHEEL_W
wheel_ctr_u   = inner_end_u + MOTOR_PLUS_WHEEL - WHEEL_W / 2
PLATE_W       = 2 * (tyre_inner_u - TYRE_CLEAR)
mount_u       = inner_end_u + BODY_CENTER_FROM_INNER_END
mount_v       = MOUNT_HOLE_SPACING / 2
TRACK         = 2 * wheel_ctr_u
OVERALL_W     = 2 * (inner_end_u + MOTOR_PLUS_WHEEL)

# fall angle: plate bottom ~7 mm above axle (motor half-height 5 + mount wall 2), wheel r = 17
r_wheel, z_plate = 17.0, 7.0
th = 0.0
while 17.0 + z_plate * math.cos(th) - (PLATE_D / 2) * math.sin(th) > 0:
    th += 0.001
FALL_DEG = math.degrees(th)

# ------------------------------------------------------------------ DXF
doc = ezdxf.new("R2010", setup=True)
doc.units = units.MM
doc.header["$INSUNITS"] = 4
doc.layers.add("CUT", color=1)       # red: cut through
doc.layers.add("ENGRAVE", color=5)   # blue: mark only (optional)
msp = doc.modelspace()

def rounded_rect(cx, cy, w, h, r, layer):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    b = math.tan(math.radians(90) / 4)          # bulge for a 90 deg arc
    pts = [(x0 + r, y0, 0), (x1 - r, y0, b), (x1, y0 + r, 0), (x1, y1 - r, b),
           (x1 - r, y1, 0), (x0 + r, y1, b), (x0, y1 - r, 0), (x0, y0 + r, b)]
    msp.add_lwpolyline([(x, y, 0, 0, bb) for x, y, bb in pts], format="xyseb", close=True,
                       dxfattribs={"layer": layer})

def slot(cx, cy, length, width, layer, horizontal=True):
    """Obround slot; `length` = centre-to-centre travel."""
    r = width / 2
    if horizontal:
        pts = [(cx - length / 2, cy - r, 0), (cx + length / 2, cy - r, 1),
               (cx + length / 2, cy + r, 0), (cx - length / 2, cy + r, 1)]
    else:
        pts = [(cx + r, cy - length / 2, 0), (cx + r, cy + length / 2, 1),
               (cx - r, cy + length / 2, 0), (cx - r, cy - length / 2, 1)]
    msp.add_lwpolyline([(x, y, 0, 0, b) for x, y, b in pts], format="xyseb", close=True,
                       dxfattribs={"layer": layer})

# outline
rounded_rect(0, 0, PLATE_W, PLATE_D, CORNER_R, "CUT")
# motor mount slots (2 per motor)
for su in (-1, 1):
    for sv in (-1, 1):
        slot(su * mount_u, sv * mount_v, MOUNT_ADJUST, M3, "CUT")
# standoff holes for upper decks
for su in (-1, 1):
    for sv in (-1, 1):
        msp.add_circle((su * STANDOFF_U, sv * STANDOFF_V), M3 / 2, dxfattribs={"layer": "CUT"})
# wire slot (encoder + motor cables come up here, behind the battery)
ww, wh, wv = WIRE_SLOT
slot(0, wv, ww - wh, wh, "CUT")
# encoder windows (one per motor)
for su in (-1, 1):
    cu = su * (ENC_U0 + ENC_U1) / 2
    rounded_rect(cu, 0, ENC_U1 - ENC_U0, ENC_H, ENC_R, "CUT")
# ---- engrave: axle line, front arrow
for x0 in range(int(-PLATE_W / 2) + 2, int(PLATE_W / 2) - 2, 6):
    x1 = min(x0 + 3, PLATE_W / 2 - 2)
    if x1 > -ENC_U1 - 1 and x0 < ENC_U1 + 1:      # don't engrave across the windows
        continue
    msp.add_line((x0, 0), (x1, 0), dxfattribs={"layer": "ENGRAVE"})
bw, bh = BATTERY
msp.add_lwpolyline([(-4, 22), (4, 22), (0, 28)], close=True, dxfattribs={"layer": "ENGRAVE"})
msp.add_text("FRONT", dxfattribs={"layer": "ENGRAVE", "height": 3.0}).set_placement((7, 24.5), align=ezdxf.enums.TextEntityAlignment.MIDDLE_LEFT)
doc.saveas("balancebot_base_plate_v2.dxf")

# ------------------------------------------------------------------ 1:1 check print (A4)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle
from matplotlib.backends.backend_pdf import PdfPages

def draw_page(ax, onetoone=True):
    ax.set_aspect("equal"); ax.axis("off")
    # ghost wheels + motors (for orientation only)
    for su in (-1, 1):
        ax.add_patch(Rectangle((su * (inner_end_u + MOTOR_PLUS_WHEEL - WHEEL_W / 2) - WHEEL_W / 2, -17), WHEEL_W, 34,
                               fc="#e6e6e6", ec="#9a9a9a", lw=0.6, ls="--"))
    ax.text(0, -60, "grey dashed = motors (under the plate) and wheels, for orientation only", ha="center", fontsize=6.5, color="#888")
    # plate
    ax.add_patch(FancyBboxPatch((-PLATE_W / 2 + CORNER_R, -PLATE_D / 2 + CORNER_R), PLATE_W - 2 * CORNER_R, PLATE_D - 2 * CORNER_R,
                                boxstyle=f"round,pad={CORNER_R}", fc="#fff6ec", ec="#d22", lw=0.9))
    for su in (-1, 1):
        for sv in (-1, 1):
            cx, cy = su * mount_u, sv * mount_v
            ax.add_patch(FancyBboxPatch((cx - MOUNT_ADJUST / 2, cy), MOUNT_ADJUST, 0, boxstyle=f"round,pad={M3/2}", fc="white", ec="#d22", lw=0.8))
            ax.add_patch(Circle((su * STANDOFF_U, sv * STANDOFF_V), M3 / 2, fc="white", ec="#d22", lw=0.8))
    ax.add_patch(FancyBboxPatch((-(ww - wh) / 2, wv), ww - wh, 0, boxstyle=f"round,pad={wh/2}", fc="white", ec="#d22", lw=0.8))
    for su in (-1, 1):   # motors under the plate (dashed, on top so they show through)
        x_in = su * inner_end_u
        x_out = su * (inner_end_u + MOTOR_PLUS_WHEEL - WHEEL_W - 1)
        ax.add_patch(Rectangle((min(x_in, x_out), -6), abs(x_out - x_in), 12, fc="none", ec="#8a8a8a", lw=0.6, ls=(0, (2, 2))))

    for su in (-1, 1):
        u0, u1 = sorted((su * ENC_U0, su * ENC_U1))
        ax.add_patch(FancyBboxPatch((u0 + ENC_R, -ENC_H / 2 + ENC_R), (u1 - u0) - 2 * ENC_R, ENC_H - 2 * ENC_R,
                                    boxstyle=f"round,pad={ENC_R}", fc="white", ec="#d22", lw=0.8))
        ax.add_patch(Rectangle((u0 + 1, -6), (u1 - u0) - 2, 12, fc="#cfe8cf", ec="#3a8a3a", lw=0.5, ls=(0, (2, 1.5))))
        ax.text((u0 + u1) / 2, 0, "enc", ha="center", va="center", fontsize=5.5, color="#2f6f2f")
    ax.text(0, -16.5, "encoder windows", ha="center", fontsize=6, color="#d22")
    ax.plot([-PLATE_W / 2 + 2, PLATE_W / 2 - 2], [0, 0], color="#2a62c9", lw=0.5, ls=(0, (3, 3)))
    ax.text(mount_u, 1.4, "axle line", ha="center", fontsize=5.5, color="#2a62c9")
    ax.add_patch(plt.Polygon([(-4, 22), (4, 22), (0, 28)], fc="none", ec="#2a62c9", lw=0.6))
    ax.text(7, 24.5, "FRONT", ha="left", va="center", fontsize=6.5, color="#2a62c9")
    ax.text(0, wv - 0.9, "wires", ha="center", fontsize=5.5, color="#d22")

    def dim(x0, y0, x1, y1, label, off=(0, 0), fs=7):
        ax.annotate("", (x0, y0), (x1, y1), arrowprops=dict(arrowstyle="<->", lw=0.6, color="#333", shrinkA=0, shrinkB=0))
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], label, ha="center", va="center", fontsize=fs,
                bbox=dict(fc="white", ec="none", pad=0.6))
    dim(-PLATE_W / 2, 36, PLATE_W / 2, 36, f"plate {PLATE_W:.1f}")
    dim(ENC_U0, -12, ENC_U1, -12, f"{ENC_U1-ENC_U0:.0f}", fs=6)
    dim(-ENC_U1 - 3, -ENC_H / 2, -ENC_U1 - 3, ENC_H / 2, f"{ENC_H:.0f}", off=(-3.2, 0), fs=6)
    dim(-PLATE_W / 2 - 5, -PLATE_D / 2, -PLATE_W / 2 - 5, PLATE_D / 2, f"{PLATE_D:.0f}", off=(-3.5, 0))
    dim(-wheel_ctr_u, -44, wheel_ctr_u, -44, f"track (wheel centres) {TRACK:.1f}")
    dim(-OVERALL_W / 2, -51, OVERALL_W / 2, -51, f"overall width {OVERALL_W:.1f}")
    dim(mount_u + 6.5, -mount_v, mount_u + 6.5, mount_v, f"{MOUNT_HOLE_SPACING:.0f}", off=(0, 5), fs=6.5)
    dim(0, 13.5, mount_u, 13.5, f"{mount_u:.0f} (slot centre)", fs=6)
    dim(STANDOFF_U, -STANDOFF_V, STANDOFF_U, STANDOFF_V, f"{2*STANDOFF_V:.0f}", off=(4, 0), fs=6.5)
    dim(-STANDOFF_U, -36, STANDOFF_U, -36, f"standoff holes {2*STANDOFF_U:.0f}", fs=6.5)

with PdfPages("balancebot_base_plate_v2_1to1.pdf") as pdf:
    W_MM, H_MM = 210, 297
    fig = plt.figure(figsize=(W_MM / 25.4, H_MM / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-W_MM / 2, W_MM / 2); ax.set_ylim(-H_MM / 2 - 40, H_MM / 2 - 40)
    draw_page(ax)
    ax.set_autoscale_on(False)
    # title + notes
    T = H_MM / 2 - 40
    ax.text(-95, T - 12, "BalanceBot — base plate v2 (top view, 1 : 1)", fontsize=12, weight="bold")
    ax.text(-95, T - 19, "Print at 100 % / 'Actual size'. Check the 50 mm bar before trusting it.", fontsize=7.5, color="#444")
    ax.plot([-95, -45], [T - 26, T - 26], color="black", lw=1.2)
    for x in (-95, -45): ax.plot([x, x], [T - 28, T - 24], color="black", lw=1.2)
    ax.text(-70, T - 31.5, "50 mm", ha="center", fontsize=7)
    notes = [
        f"Material: 3 mm acrylic or 3 mm plywood/MDF.  Red = CUT, blue = ENGRAVE (optional).",
        f"Slots: 4 x M3, {MOUNT_ADJUST:.0f} mm travel along the axle -> slide motors to set the gap/track.",
        f"Mount holes assumed {MOUNT_HOLE_SPACING:.0f} mm apart, one each side of the motor (front/back).",
        f"Holes: 4 x M3 ({M3} mm) for standoffs to the upper decks.  Wire slot {ww:.0f} x {wh:.0f} mm at the back.",
        f"Encoder windows: 2 x {ENC_U1-ENC_U0:.0f} x {ENC_H:.0f} mm, {ENC_U0:.0f}-{ENC_U1:.0f} mm from the centre line, over each encoder end.",
        "Battery: the windows sit where it used to go -> put it on the next deck, or on 8-10 mm foam pads here.",
        f"Track {TRACK:.1f} mm, overall width {OVERALL_W:.1f} mm.  Plate stays {TYRE_CLEAR:.0f} mm inside each tyre.",
        f"Body hits the floor at ~{FALL_DEG:.0f} deg of tilt (firmware cuts motors at 40 deg).",
        "Motors + mounts go UNDER the plate (screw heads on top); encoder ends point to the centre, under the windows.",
        "BEFORE CUTTING: lay a printed mount on this sheet - its holes must sit on the red slots.",
    ]
    for i, n in enumerate(notes):
        ax.text(-95, -H_MM / 2 - 40 + 62 - i * 6, ("• " + n), fontsize=7.2, color="#222")
    pdf.savefig(fig)
    fig.savefig("preview.png", dpi=110)
    plt.close(fig)

print(f"plate {PLATE_W:.1f} x {PLATE_D:.1f} mm, track {TRACK:.1f}, overall {OVERALL_W:.1f}, "
      f"mount slots at u=+-{mount_u:.1f}, v=+-{mount_v:.1f}, fall angle {FALL_DEG:.0f} deg")
