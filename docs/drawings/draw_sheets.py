"""Engineering drawing sheets for BO-BOT as built (Arduino Uno build), A3, third angle projection.

Sizes come from the files that made the parts:
  base plate   cad/laser-cut/base_plate/make_base.py
  motor clamp  cad/printed-parts/n20_clamp/n20_clamp.scad
  spacer pins  cad/printed-parts/spacers/v3/plate_pin.scad
Units mm. Axes: x across the robot, y up from the ground (front views), v towards the front (plan views).
Run:  python draw_sheets.py   (writes sheet1-general-arrangement.png and sheet2-electronics.png here)
The drawings of the first plan (ESP32, cat shell) are in archive-esp32-cat-shell/.
"""
import math, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyBboxPatch

OUT = os.path.dirname(os.path.abspath(__file__))
plt.rcParams["font.family"] = ["Inter", "Segoe UI", "DejaVu Sans"]
plt.rcParams["hatch.linewidth"] = 0.6
INK = "#111111"; THIN = 0.45; MED = 0.9; THICK = 1.6
A3W, A3H = 420, 297
DATE = "20-11-2026"; REV = "B"

# ---------------------------------------------------------------- sizes (mm)
# wheels and motors (make_base.py)
WHEEL_D, WHEEL_W = 34.0, 6.5
AX = WHEEL_D / 2                       # axle height
MOTOR_PLUS_WHEEL = 50.0                # encoder end -> wheel outer face (measured)
GAP_CENTER = 8.0                       # gap between the two encoder ends
INNER = GAP_CENTER / 2                 # encoder end, from the centre line
WHEEL_OUT = INNER + MOTOR_PLUS_WHEEL   # 54
WHEEL_IN = WHEEL_OUT - WHEEL_W         # 47.5
WHEEL_C = WHEEL_OUT - WHEEL_W / 2      # 50.75
TRACK, OVERALL = 2 * WHEEL_C, 2 * WHEEL_OUT
MOTOR_OUT = 36.0                       # gearbox output end (N20: encoder + can + gearbox about 32 long)
MOTOR_W, MOTOR_H = 12.0, 10.0
# base plate (make_base.py)
PLATE_W, PLATE_D, CORNER_R = 89.0, 62.0, 5.0
M3 = 3.4
HOLE_U, HOLE_V = 38.0, 23.0
SLOT_U, SLOT_V, SLOT_TRAVEL = INNER + 22.0, 10.0, 7.0
WIRE_W, WIRE_H, WIRE_V = 18.0, 6.0, -24.5
ENC_U0, ENC_U1, ENC_H, ENC_R = 3.0, 16.0, 16.0, 1.5
BASE_T = 3.0
# motor clamp (n20_clamp.scad)
CL_L, CL_B, CL_SP = 20.0, 28.0, 20.0
CL_WALL, CL_SQUEEZE, CL_FIT = 2.5, 0.25, 0.35
CL_HALF = CL_WALL + MOTOR_H / 2 - CL_SQUEEZE       # 7.25, height of each half
# stack (plate_pin.scad v3: 17 mm gap, 4 mm decks)
GAP, DECK_T = 17.0, 4.0
Y_BASE = AX + CL_HALF                  # underside of the base plate
Y_D2 = Y_BASE + BASE_T + GAP           # underside of deck 2 (battery)
Y_D3 = Y_D2 + DECK_T + GAP             # underside of deck 3 (Uno)
BATT = (68.0, 37.0, 12.0)
UNO_L, UNO_W, UNO_PCB, UNO_H = 68.6, 53.4, 1.6, 12.5   # board, PCB thickness, height over PCB (headers, USB)


# ---------------------------------------------------------------- drawing helpers
def new_sheet():
    fig = plt.figure(figsize=(A3W / 25.4, A3H / 25.4), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, A3W); ax.set_ylim(0, A3H); ax.axis("off")
    ax.add_patch(Rectangle((0, 0), A3W, A3H, fc="white", ec="none"))
    ax.add_patch(Rectangle((20, 10), A3W - 30, A3H - 20, fc="none", ec=INK, lw=1.6))
    ax.add_patch(Rectangle((15, 5), A3W - 20, A3H - 10, fc="none", ec=INK, lw=0.5))
    for i in range(1, 8):
        x = 20 + i * (A3W - 30) / 8
        ax.plot([x, x], [10, 14], color=INK, lw=0.5); ax.plot([x, x], [A3H - 10, A3H - 14], color=INK, lw=0.5)
    for i in range(8):
        x = 20 + (i + .5) * (A3W - 30) / 8
        ax.text(x, 6.7, str(i + 1), ha="center", va="center", fontsize=6)
        ax.text(x, A3H - 7.3, str(i + 1), ha="center", va="center", fontsize=6)
    for i, c in enumerate("FEDCBA"):
        y = 10 + (i + .5) * (A3H - 20) / 6
        ax.text(17.4, y, c, ha="center", va="center", fontsize=6)
    return fig, ax


def line(ax, pts, lw=MED, ls="-", c=INK, z=3):
    xs, ys = zip(*pts); ax.plot(xs, ys, color=c, lw=lw, ls=ls, solid_capstyle="round", zorder=z)


def rect(ax, x0, y0, x1, y1, lw=MED, ls="-", fc="none", hatch=None, z=3, ec=INK):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1))
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=ec, lw=lw, ls=ls, hatch=hatch, zorder=z))


def rrect(ax, cx, cy, w, h, r, lw=MED, ls="-", fc="none", z=3):
    ax.add_patch(FancyBboxPatch((cx - w / 2 + r, cy - h / 2 + r), w - 2 * r, h - 2 * r, boxstyle=f"round,pad={r}",
                                fc=fc, ec=INK, lw=lw, ls=ls, zorder=z))


def obround(ax, cx, cy, travel, width, lw=MED, z=4):
    """slot with round ends; travel = centre to centre, horizontal"""
    r = width / 2
    ax.add_patch(FancyBboxPatch((cx - travel / 2, cy), travel, 0, boxstyle=f"round,pad={r}", fc="white", ec=INK, lw=lw, zorder=z))


def centre(ax, x0, y0, x1, y1):
    ax.plot([x0, x1], [y0, y1], color=INK, lw=THIN, ls=(0, (8, 2, 1.5, 2)), zorder=2)


def txt(ax, x, y, s, size=7, ha="left", va="center", weight="normal", rot=0, color=INK):
    ax.text(x, y, s, fontsize=size, ha=ha, va=va, fontweight=weight, rotation=rot, color=color, zorder=7)


def arrow(ax, p, q):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="-|>,head_length=0.35,head_width=0.12",
                                                   color=INK, lw=0.5, shrinkA=0, shrinkB=0), zorder=5)


def dim_h(ax, x1, x2, y, label, off=-9, size=6.5, ext=True):
    yd = y + off; s = 1 if off > 0 else -1
    if ext:
        for x in (x1, x2):
            ax.plot([x, x], [y + s * 1.2, yd + s * 1.5], color=INK, lw=THIN, zorder=2)
    arrow(ax, ((x1 + x2) / 2, yd), (x1, yd)); arrow(ax, ((x1 + x2) / 2, yd), (x2, yd))
    ax.plot([x1, x2], [yd, yd], color=INK, lw=THIN, zorder=2)
    txt(ax, (x1 + x2) / 2, yd + s * 2.2, label, size, ha="center")


def dim_v(ax, y1, y2, x, label, off=-9, size=6.5):
    xd = x + off; s = 1 if off > 0 else -1
    for y in (y1, y2):
        ax.plot([x + s * 1.2, xd + s * 1.5], [y, y], color=INK, lw=THIN, zorder=2)
    arrow(ax, (xd, (y1 + y2) / 2), (xd, y1)); arrow(ax, (xd, (y1 + y2) / 2), (xd, y2))
    ax.plot([xd, xd], [y1, y2], color=INK, lw=THIN, zorder=2)
    txt(ax, xd - s * 2.4, (y1 + y2) / 2, label, size, ha="center", rot=90)


def leader(ax, tip, text_xy, label, size=6, ha="left"):
    arrow(ax, text_xy, tip)
    txt(ax, text_xy[0] + (1.2 if ha == "left" else -1.2), text_xy[1], label, size, ha=ha)


def view_title(ax, x, y, name, scale="SCALE 1 : 1"):
    txt(ax, x, y, name, 9, ha="center", weight="bold")
    txt(ax, x, y - 4.6, scale, 6.5, ha="center")
    ax.plot([x - 24, x + 24], [y - 2.3, y - 2.3], color=INK, lw=0.7)


def balloon(ax, x, y, n, tip):
    ax.add_patch(Circle((x, y), 3.1, fc="white", ec=INK, lw=0.7, zorder=6))
    txt(ax, x, y, str(n), 6.3, ha="center", weight="bold")
    d = math.hypot(tip[0] - x, tip[1] - y)
    arrow(ax, (x + (tip[0] - x) * 3.1 / d, y + (tip[1] - y) * 3.1 / d), tip)


def table(ax, x0, y_top, colw, rows, rowh=6.0, size=6.0, aligns=None):
    n = len(rows); W = sum(colw)
    ax.add_patch(Rectangle((x0, y_top - n * rowh), W, n * rowh, fc="white", ec=INK, lw=1.0, zorder=2))
    for i, r in enumerate(rows):
        y = y_top - (i + .5) * rowh
        if i > 0:
            ax.plot([x0, x0 + W], [y_top - i * rowh] * 2, color=INK, lw=1.0 if i == 1 else 0.5, zorder=3)
        cx = x0
        for j, c in enumerate(r):
            al = aligns[j] if aligns else "left"
            tx = cx + 1.6 if al == "left" else cx + colw[j] / 2
            txt(ax, tx, y, str(c), size, ha=al, weight="bold" if i == 0 else "normal")
            cx += colw[j]
    cx = x0
    for w in colw[:-1]:
        cx += w
        ax.plot([cx, cx], [y_top, y_top - n * rowh], color=INK, lw=0.5, zorder=3)


def title_block(ax, title, sub, sheet, dwg):
    x0, y0, w, h = 230, 10, 180, 46
    ax.add_patch(Rectangle((x0, y0), w, h, fc="white", ec=INK, lw=1.6, zorder=2))
    ax.plot([x0, x0 + w], [y0 + 26, y0 + 26], color=INK, lw=0.8)
    ax.plot([x0, x0 + w], [y0 + 13, y0 + 13], color=INK, lw=0.5)
    for xv in (x0 + 60, x0 + 120): ax.plot([xv, xv], [y0, y0 + 26], color=INK, lw=0.5)
    ax.plot([x0 + 120, x0 + 120], [y0 + 26, y0 + h], color=INK, lw=0.5)
    txt(ax, x0 + 3, y0 + h - 4, "TITLE", 5, color="#444"); txt(ax, x0 + 3, y0 + h - 12, title, 11, weight="bold")
    txt(ax, x0 + 3, y0 + 30.5, sub, 7)
    txt(ax, x0 + 123, y0 + h - 4, "DRAWING NO.", 5, color="#444"); txt(ax, x0 + 123, y0 + h - 12, dwg, 10, weight="bold")
    txt(ax, x0 + 123, y0 + 30.5, f"REV  {REV}", 7)
    cells = [("DRAWN BY", "Aamna Khan", 0, 13), ("DATE", DATE, 60, 13), ("SCALE", "as noted", 120, 13),
             ("CHECKED", "", 0, 0), ("UNITS", "mm", 60, 0), ("SHEET", sheet, 120, 0)]
    for lab, val, dx, dy in cells:
        txt(ax, x0 + dx + 3, y0 + dy + 10.5, lab, 4.8, color="#444"); txt(ax, x0 + dx + 3, y0 + dy + 4.6, val, 7.5)
    px, py = x0 + 150, y0 + 6.5
    ax.add_patch(Polygon([[px - 7, py - 4.2], [px - 7, py + 4.2], [px + 2, py + 2.4], [px + 2, py - 2.4]], fc="none", ec=INK, lw=0.7, zorder=6))
    ax.add_patch(Circle((px + 13, py), 4.2, fc="none", ec=INK, lw=0.7, zorder=6)); ax.add_patch(Circle((px + 13, py), 2.0, fc="none", ec=INK, lw=0.7, zorder=6))
    ax.plot([px - 9, px + 19], [py, py], color=INK, lw=0.4, ls=(0, (8, 2, 1.5, 2)), zorder=6)
    txt(ax, px + 3, py + 8.4, "THIRD ANGLE", 4.6, ha="center", color="#444")


def ground(ax, x0, x1, y):
    line(ax, [(x0, y), (x1, y)], THICK)
    x = x0 + 3
    while x < x1:
        ax.plot([x, x - 2.4], [y, y - 2.4], color=INK, lw=0.4, zorder=2); x += 5


def wheel_side(ax, cx, cy):
    ax.add_patch(Circle((cx, cy), WHEEL_D / 2, fc="white", ec=INK, lw=MED, zorder=6))
    ax.add_patch(Circle((cx, cy), 13, fc="none", ec=INK, lw=THIN, zorder=6))
    ax.add_patch(Circle((cx, cy), 1.5, fc="none", ec=INK, lw=MED, zorder=6))
    for k in range(3):
        th = 2 * math.pi * k / 3 + math.pi / 2
        line(ax, [(cx + 2.5 * math.cos(th), cy + 2.5 * math.sin(th)), (cx + 12 * math.cos(th), cy + 12 * math.sin(th))], THIN, z=6)


# ================================================================== SHEET 1
def sheet1():
    fig, ax = new_sheet()

    # -------- FRONT VIEW (from the face side; faceplate removed) --------
    GX, GY = 97, 166
    X = lambda u: GX + u
    Y = lambda y: GY + y
    for s in (1, -1):
        # wheel (rectangle in this view) and hub
        rect(ax, X(s * WHEEL_IN), Y(0), X(s * WHEEL_OUT), Y(WHEEL_D), MED, fc="white", z=4)
        rect(ax, X(s * (WHEEL_IN + 0.8)), Y(4), X(s * (WHEEL_OUT - 0.8)), Y(WHEEL_D - 4), THIN, z=5)
        # motor: encoder end + can (visible), clamp in front of the gearbox, shaft to the wheel
        rect(ax, X(s * INNER), Y(AX - MOTOR_H / 2), X(s * (INNER + 22 - CL_L / 2)), Y(AX + MOTOR_H / 2), MED, fc="white", z=3)
        rect(ax, X(s * INNER), Y(AX - 6), X(s * (INNER + 1.2)), Y(AX + 6), MED, fc="white", z=4)       # encoder disc / board
        line(ax, [(X(s * MOTOR_OUT), Y(AX)), (X(s * WHEEL_IN), Y(AX))], MED)
        cx0, cx1 = SLOT_U - CL_L / 2, SLOT_U + CL_L / 2
        rect(ax, X(s * cx0), Y(AX + CL_SQUEEZE), X(s * cx1), Y(Y_BASE), MED, fc="white", z=4)          # top saddle
        rect(ax, X(s * cx0), Y(AX - CL_SQUEEZE), X(s * cx1), Y(AX - CL_HALF), MED, fc="white", z=4)    # bottom cap
        # spacer pins (two gaps)
        for y0 in (Y_BASE + BASE_T, Y_D2 + DECK_T):
            rect(ax, X(s * HOLE_U - 3.5), Y(y0), X(s * HOLE_U + 3.5), Y(y0 + GAP), MED, fc="white", hatch="////", z=3)
    # plates
    rect(ax, X(-PLATE_W / 2), Y(Y_BASE), X(PLATE_W / 2), Y(Y_BASE + BASE_T), MED, fc="black", z=5)
    rect(ax, X(-PLATE_W / 2), Y(Y_D2), X(PLATE_W / 2), Y(Y_D2 + DECK_T), MED, fc="black", z=5)
    rect(ax, X(-PLATE_W / 2), Y(Y_D3), X(PLATE_W / 2), Y(Y_D3 + DECK_T), MED, fc="black", z=5)
    # battery on deck 2
    yb = Y_D2 + DECK_T
    rect(ax, X(-BATT[0] / 2), Y(yb), X(BATT[0] / 2), Y(yb + BATT[2]), MED, fc="white", hatch="....", z=4)
    # Uno on deck 3: PCB + parts above it
    yu = Y_D3 + DECK_T
    rect(ax, X(-UNO_W / 2), Y(yu), X(UNO_W / 2), Y(yu + UNO_PCB), MED, fc="#555", z=5)
    rect(ax, X(-UNO_W / 2 + 1), Y(yu + UNO_PCB), X(-UNO_W / 2 + 4), Y(yu + UNO_H), MED, fc="white", z=4)   # header
    rect(ax, X(UNO_W / 2 - 4), Y(yu + UNO_PCB), X(UNO_W / 2 - 1), Y(yu + UNO_H), MED, fc="white", z=4)     # header
    rect(ax, X(-9), Y(yu + UNO_PCB), X(9), Y(yu + UNO_PCB + 3.5), THIN, fc="white", z=4)                   # chip
    centre(ax, GX, GY - 6, GX, GY + 98); centre(ax, GX - 60, GY + AX, GX + 60, GY + AX)
    ground(ax, GX - 62, GX + 62, GY)
    # dimensions
    dim_h(ax, X(-WHEEL_C), X(WHEEL_C), GY, f"{TRACK:.1f}  (track)", off=-10)
    dim_h(ax, X(-WHEEL_OUT), X(WHEEL_OUT), GY, f"{OVERALL:.0f}", off=-18)
    dim_h(ax, X(-PLATE_W / 2), X(PLATE_W / 2), Y(Y_D3 + DECK_T), f"{PLATE_W:.0f}", off=30)
    dim_h(ax, X(-UNO_W / 2), X(UNO_W / 2), Y(yu + UNO_H), f"{UNO_W}", off=4, size=6)
    dim_v(ax, GY, Y(Y_BASE), X(-WHEEL_OUT), f"{Y_BASE:.1f}", off=-5, size=6)
    dim_v(ax, Y(Y_BASE + BASE_T), Y(Y_D2), X(-PLATE_W / 2), f"{GAP:.0f}", off=-14, size=6)
    dim_v(ax, Y(Y_D2 + DECK_T), Y(Y_D3), X(-PLATE_W / 2), f"{GAP:.0f}", off=-14, size=6)
    dim_v(ax, GY, Y(Y_D3 + DECK_T), X(-WHEEL_OUT), f"{Y_D3 + DECK_T:.1f}", off=-12.5, size=6)
    # balloons (right side, in height order)
    for n, by, tip in ((13, 92, (20, yu + UNO_PCB / 2)), (3, 80, (40, Y_D3 + DECK_T / 2)), (11, 68, (30, yb + 6)),
                       (2, 56, (40, Y_D2 + DECK_T / 2)), (4, 44, (HOLE_U + 3.5, Y_BASE + BASE_T + 12)),
                       (1, 34, (42, Y_BASE + BASE_T / 2)), (5, 22, (30, AX - 4)), (9, 12, (12, AX + 2)), (10, 2, (52, 5))):
        balloon(ax, X(70), Y(by), n, (X(tip[0]), Y(tip[1])))
    view_title(ax, GX, 137, "FRONT VIEW  (FACEPLATE REMOVED)")

    # -------- SIDE VIEW (from the right; front of the robot to the right) --------
    SX = 214
    Z = lambda v: SX + v
    for v in (HOLE_V, -HOLE_V):
        for y0 in (Y_BASE + BASE_T, Y_D2 + DECK_T):
            rect(ax, Z(v - 3.5), Y(y0), Z(v + 3.5), Y(y0 + GAP), MED, fc="white", hatch="////", z=3)
    for y0, t in ((Y_BASE, BASE_T), (Y_D2, DECK_T), (Y_D3, DECK_T)):
        rect(ax, Z(-PLATE_D / 2), Y(y0), Z(PLATE_D / 2), Y(y0 + t), MED, fc="black", z=5)
    rect(ax, Z(-BATT[1] / 2), Y(yb), Z(BATT[1] / 2), Y(yb + BATT[2]), MED, fc="white", hatch="....", z=4)
    rect(ax, Z(-UNO_L / 2), Y(yu), Z(UNO_L / 2), Y(yu + UNO_PCB), MED, fc="#555", z=5)
    rect(ax, Z(-UNO_L / 2 + 6), Y(yu + UNO_PCB), Z(UNO_L / 2 - 6), Y(yu + UNO_H - 4), THIN, ls=(0, (4, 2)), z=4)
    rect(ax, Z(-CL_B / 2), Y(AX - CL_HALF), Z(CL_B / 2), Y(Y_BASE), THIN, ls=(0, (4, 2)), z=2)   # clamp, behind the wheel
    wheel_side(ax, SX, GY + AX)
    centre(ax, SX, GY - 6, SX, GY + 98); centre(ax, SX - 42, GY + AX, SX + 42, GY + AX)
    ground(ax, SX - 44, SX + 44, GY)
    dim_h(ax, Z(-PLATE_D / 2), Z(PLATE_D / 2), Y(Y_D3 + DECK_T), f"{PLATE_D:.0f}", off=30, size=6)
    dim_h(ax, Z(-UNO_L / 2), Z(UNO_L / 2), Y(yu + UNO_PCB), f"{UNO_L}", off=14.5, size=6)
    dim_h(ax, Z(-WHEEL_D / 2), Z(WHEEL_D / 2), GY, f"Ø{WHEEL_D:.0f}", off=-10)
    dim_v(ax, GY, Y(AX), Z(-PLATE_D / 2), f"{AX:.0f}", off=-5, size=6)
    txt(ax, Z(PLATE_D / 2 + 3), Y(50), "FRONT", 6.5, weight="bold"); arrow(ax, (Z(PLATE_D / 2 + 3), Y(45)), (Z(PLATE_D / 2 + 13), Y(45)))
    view_title(ax, SX, 137, "SIDE VIEW  (FROM RIGHT)")

    # -------- TOP VIEW of the base plate (the laser-cut part) --------
    TX, TY = 97, 74
    U = lambda u: TX + u
    V = lambda v: TY + v
    for s in (1, -1):
        rect(ax, U(s * WHEEL_IN), V(-WHEEL_D / 2), U(s * WHEEL_OUT), V(WHEEL_D / 2), THIN, ls=(0, (4, 2)), z=2)
        rect(ax, U(s * INNER), V(-MOTOR_W / 2), U(s * MOTOR_OUT), V(MOTOR_W / 2), THIN, ls=(0, (4, 2)), z=2)
    rrect(ax, TX, TY, PLATE_W, PLATE_D, CORNER_R, THICK, z=3)
    for su in (-1, 1):
        for sv in (-1, 1):
            ax.add_patch(Circle((U(su * HOLE_U), V(sv * HOLE_V)), M3 / 2, fc="white", ec=INK, lw=MED, zorder=4))
            obround(ax, U(su * SLOT_U), V(sv * SLOT_V), SLOT_TRAVEL, M3)
        rrect(ax, U(su * (ENC_U0 + ENC_U1) / 2), TY, ENC_U1 - ENC_U0, ENC_H, ENC_R, MED, fc="white", z=4)
    obround(ax, TX, V(WIRE_V), WIRE_W - WIRE_H, WIRE_H)
    centre(ax, U(-60), TY, U(60), TY); centre(ax, TX, V(-38), TX, V(38))
    dim_h(ax, U(-PLATE_W / 2), U(PLATE_W / 2), V(PLATE_D / 2), f"{PLATE_W:.0f}", off=12)
    dim_h(ax, U(-HOLE_U), U(HOLE_U), V(HOLE_V), f"{2 * HOLE_U:.0f}", off=10)
    dim_h(ax, U(-WHEEL_C), U(WHEEL_C), V(-WHEEL_D / 2), f"{TRACK:.1f}  (track)", off=-22)
    dim_v(ax, V(-PLATE_D / 2), V(PLATE_D / 2), U(PLATE_W / 2), f"{PLATE_D:.0f}", off=15)
    dim_v(ax, V(-HOLE_V), V(HOLE_V), U(-HOLE_U), f"{2 * HOLE_V:.0f}", off=-23)
    dim_v(ax, V(-SLOT_V), V(SLOT_V), U(SLOT_U + 5), f"{2 * SLOT_V:.0f}", off=4, size=5.8)
    leader(ax, (U(HOLE_U + 1.2), V(HOLE_V + 1.2)), (U(64), V(36)), "4 x Ø3.4 (M3)")
    leader(ax, (U(SLOT_U + 5), V(SLOT_V + 1)), (U(64), V(27)), f"4 slots 3.4 wide, {SLOT_TRAVEL:.0f} travel,")
    txt(ax, U(65.2), V(22.5), f"centres at ±{SLOT_U:.0f}, ±{SLOT_V:.0f}", 6)
    leader(ax, (U(ENC_U1 - 1), V(-ENC_H / 2 + 1)), (U(64), V(-12)), f"2 encoder windows {ENC_U1 - ENC_U0:.0f} x {ENC_H:.0f},")
    txt(ax, U(65.2), V(-16.5), f"{ENC_U0:.0f} to {ENC_U1:.0f} from the centre line, R{ENC_R}", 6)
    leader(ax, (U(WIRE_W / 2 - 1), V(WIRE_V - 1)), (U(64), V(-27)), f"wire slot {WIRE_W:.0f} x {WIRE_H:.0f},")
    txt(ax, U(65.2), V(-31.5), f"centre {abs(WIRE_V)} behind the axle", 6)
    leader(ax, (U(-PLATE_W / 2 + 1.6), V(PLATE_D / 2 - 1.6)), (U(-62), V(40)), f"R{CORNER_R:.0f}", ha="right")
    txt(ax, U(-58), V(27), "FRONT", 6.5, ha="center", weight="bold"); arrow(ax, (U(-58), V(12)), (U(-58), V(24)))
    txt(ax, TX, V(-PLATE_D / 2 - 3.5), "grey dashed: motors and wheels under the plate", 5.5, ha="center", color="#555")
    view_title(ax, TX, 19, "BASE PLATE  (TOP VIEW, LASER CUT)")

    # -------- DETAIL C: motor clamp section, 2:1 --------
    DX, DY, k = 318, 102, 2.0
    P = lambda x, y: (DX + x * k, DY + y * k)
    def R(x0, y0, x1, y1, **kw): rect(ax, *P(x0, y0), *P(x1, y1), **kw)
    R(-CL_B / 2, CL_SQUEEZE, CL_B / 2, CL_HALF, lw=MED, fc="white", hatch="////", z=3)
    R(-CL_B / 2, -CL_HALF, CL_B / 2, -CL_SQUEEZE, lw=MED, fc="white", hatch="\\\\\\\\", z=3)
    R(-(MOTOR_W + CL_FIT) / 2, -MOTOR_H / 2, (MOTOR_W + CL_FIT) / 2, MOTOR_H / 2, lw=MED, fc="white", z=4)
    ax.add_patch(FancyBboxPatch(P(-MOTOR_W / 2 + 1.5, -MOTOR_H / 2 + 1.5), (MOTOR_W - 3) * k, (MOTOR_H - 3) * k,
                                boxstyle=f"round,pad={1.5 * k}", fc="#ddd", ec=INK, lw=MED, zorder=5))
    R(-CL_B / 2 - 4, CL_HALF, CL_B / 2 + 4, CL_HALF + BASE_T, lw=MED, fc="white", hatch="....", z=3)        # base plate
    for sx in (-1, 1):
        R(sx * CL_SP / 2 - M3 / 2, -CL_HALF, sx * CL_SP / 2 + M3 / 2, CL_HALF + BASE_T, lw=THIN, fc="white", z=5)   # screw hole
        R(sx * CL_SP / 2 - 2.8, CL_HALF + BASE_T, sx * CL_SP / 2 + 2.8, CL_HALF + BASE_T + 2, lw=MED, fc="white", z=5)  # screw head
        R(sx * CL_SP / 2 - 2.8, -CL_HALF, sx * CL_SP / 2 + 2.8, -CL_HALF + 2.6, lw=MED, fc="#bbb", z=5)               # nut
    centre(ax, *P(-CL_B / 2 - 7, 0), *P(CL_B / 2 + 7, 0)); centre(ax, *P(0, -CL_HALF - 4), *P(0, CL_HALF + 8))
    x0, _ = P(-CL_B / 2, 0); x1, _ = P(CL_B / 2, 0); _, yb0 = P(0, -CL_HALF)
    dim_h(ax, x0, x1, yb0, f"{CL_B:.0f}", off=-7)
    xs0, _ = P(-CL_SP / 2, 0); xs1, _ = P(CL_SP / 2, 0); _, yt = P(0, CL_HALF + BASE_T + 2)
    dim_h(ax, xs0, xs1, yt, f"{CL_SP:.0f}", off=6)
    _, ya = P(0, CL_SQUEEZE); _, yb1 = P(0, CL_HALF)
    dim_v(ax, ya, yb1, x1, f"{CL_HALF - CL_SQUEEZE:.0f}", off=7, size=5.8)
    _, ym0 = P(0, -MOTOR_H / 2); _, ym1 = P(0, MOTOR_H / 2)
    dim_v(ax, ym0, ym1, x0, f"{MOTOR_H:.0f}", off=-7, size=5.8)
    leader(ax, P(0, -MOTOR_H / 2 + 2), (DX + 40, DY - 26), "N20 motor body 12 x 10", 5.8)
    leader(ax, P(CL_SP / 2, -CL_HALF + 1.3), (DX + 40, DY - 31), "M3 nut in pocket", 5.8)
    leader(ax, P(CL_SP / 2 + 2, CL_HALF + BASE_T + 1), (DX + 40, DY + 30), "M3 x 20 from the top", 5.8)
    leader(ax, P(-CL_B / 2 - 2, CL_HALF + 1.5), (DX - 44, DY + 30), "base plate", 5.8, ha="right")
    view_title(ax, DX, 66, "DETAIL C  MOTOR CLAMP (SECTION)", "SCALE 2 : 1")

    # -------- parts list --------
    rows = [("NO.", "PART", "QTY", "SIZE / FILE"),
            ("1", "Base plate, laser-cut acrylic", "1", "89 x 62 x 3"),
            ("2", "Deck 2 (battery)", "1", "4 thick"),
            ("3", "Deck 3 (Arduino Uno)", "1", "4 thick"),
            ("4", "Spacer pin v3 (PLA)", "8", "17 gap, see spacers/v3"),
            ("5", "Motor clamp, top + bottom (PLA)", "2", "28 x 20, n20_clamp"),
            ("6", "Plate stand v2 (PLA, yellow)", "4", "50 x 13.25, plate_stand_v2"),
            ("7", "Front faceplate v4 (TPU)", "1", "front_faceplate_v4"),
            ("8", "Face stand v2", "1", "face_stand_v2"),
            ("9", "N20 6 V 300 rpm motor + encoder", "2", "12 x 10 body"),
            ("10", "Wheel and tyre", "2", "Ø34 x 6.5"),
            ("11", "LiPo 2S 1000 mAh, XT60", "1", "68 x 37 x 12"),
            ("12", "M3 x 20 screw + M3 nut", "4", "motor clamps"),
            ("13", "Arduino Uno", "1", "68.6 x 53.4"),
            ("14", "MPU-6050 (GY-521)", "1", "on the mini breadboard"),
            ("15", "DRV8833 motor driver", "1", "on the mini breadboard"),
            ("16", "OLED 1.3\" SH1106", "1", "behind the faceplate"),
            ("17", "Push button, 10 kΩ resistor", "1 + 1", "see sheet 2")]
    table(ax, 262, 282, [9, 60, 11, 63], rows, rowh=4.75, size=5.9, aligns=["center", "left", "center", "left"])

    # -------- notes --------
    nx, ny = 262, 189
    txt(ax, nx, ny, "NOTES", 8, weight="bold")
    notes = ["1.  Dimensions in mm. Base plate from cad/laser-cut/base_plate/make_base.py,",
             "     clamp and pins from the OpenSCAD files in cad/printed-parts/.",
             "2.  Plate heights follow the clamp (7.25 per half) and the v3 pins",
             "     (17 gap, 4 mm decks). Decks 2 and 3 are drawn with the base plate outline.",
             "3.  Faceplate, face stand, plate stands and wiring are left out of the views;",
             "     their Fusion 360 files are in cad/printed-parts/fusion360/.",
             "4.  The body touches the floor at about 45° of tilt; the firmware stops",
             "     the motors at 40°.",
             "5.  Motor + wheel = 50 measured from the encoder end to the wheel outer face.",
             "6.  Clamp halves stay about 0.5 apart when tight: that gap is the grip."]
    for i, n in enumerate(notes): txt(ax, nx, ny - 6 - i * 4.4, n, 5.8)

    title_block(ax, "BO-BOT  Self-Balancing Robot", "General arrangement, Arduino Uno build", "1 of 2", "BB-GA-02")
    fig.savefig(os.path.join(OUT, "sheet1-general-arrangement.png"), dpi=200)
    plt.close(fig)


# ================================================================== SHEET 2
def box_(ax, x, y, w, h, lab, sub="", size=7):
    rect(ax, x, y, x + w, y + h, 1.0, fc="white")
    txt(ax, x + w / 2, y + h / 2 + (1.7 if sub else 0), lab, size, ha="center", weight="bold")
    if sub: txt(ax, x + w / 2, y + h / 2 - 2.5, sub, 5.6, ha="center")


def wire(ax, pts, lab=None, lpos=None, ls="-", lw=0.9, size=5.8, ha="center", arrows="end"):
    line(ax, pts, lw, ls)
    if arrows in ("end", "both"): arrow(ax, pts[-2], pts[-1])
    if arrows == "both": arrow(ax, pts[1], pts[0])
    if lab: txt(ax, lpos[0], lpos[1], lab, size, ha=ha)


def sheet2():
    fig, ax = new_sheet()
    # ------------------------------------------------ block diagram
    txt(ax, 30, 280, "SYSTEM BLOCK DIAGRAM", 8.5, weight="bold")
    box_(ax, 30, 238, 40, 22, "LiPo 2S", "7.4 V, 1000 mAh")
    box_(ax, 120, 226, 50, 44, "Arduino Uno", "200 Hz control loop")
    box_(ax, 120, 194, 50, 18, "DRV8833", "dual H-bridge")
    box_(ax, 205, 194, 44, 18, "N20 motors", "L + R, with encoders", size=6.5)
    box_(ax, 205, 255, 40, 13, "MPU-6050", "I2C 0x68", size=6.5)
    box_(ax, 205, 238, 40, 13, "OLED 1.3\"", "SH1106, I2C 0x3C", size=6.5)
    box_(ax, 205, 222, 40, 11, "Push button", "to GND", size=6.5)
    wire(ax, [(70, 249), (92, 249), (92, 203), (120, 203)], arrows="end")
    wire(ax, [(92, 249), (92, 258), (120, 258)], arrows="end")
    ax.add_patch(Circle((92, 249), 0.9, fc=INK, ec=INK, zorder=6))
    txt(ax, 81, 252, "XT60", 5.8, ha="center")
    txt(ax, 106, 261, "VIN", 5.8, ha="center"); txt(ax, 106, 206, "VCC", 5.8, ha="center")
    wire(ax, [(135, 226), (135, 212)], "D5 D6 D9 D10 (PWM)", (137, 220.5), ha="left")
    txt(ax, 137, 216, "EEP: D8 (+10 k to GND)", 5.6)
    wire(ax, [(170, 203), (205, 203)], "OUT1-4", (187.5, 206))
    wire(ax, [(212, 212), (212, 218), (160, 218), (160, 226)], arrows="end", ls=(0, (4, 2)))
    txt(ax, 186, 215.5, "encoders D2/D4, D3/D7", 5.6, ha="center")
    wire(ax, [(170, 261.5), (205, 261.5)], arrows="both"); wire(ax, [(170, 244.5), (205, 244.5)], arrows="both")
    txt(ax, 187.5, 253, "I2C A4 / A5", 5.8, ha="center")
    wire(ax, [(205, 227.5), (170, 227.5)], "D12", (187.5, 230))
    txt(ax, 30, 228, "Solid: power and signals.  Dashed: encoder lines.", 5.8)
    txt(ax, 30, 223, "All grounds joined.  Uno 5V -> encoders, 3.3V -> MPU and OLED.", 5.8)

    # ------------------------------------------------ pin map and supplies
    rows = [("SIGNAL", "UNO PIN", "GOES TO"),
            ("Left encoder A (green)", "D2", "interrupt, rising edge"),
            ("Right encoder A (green)", "D3", "interrupt, rising edge"),
            ("Left encoder B (yellow)", "D4", "direction"),
            ("Right encoder B (yellow)", "D7", "direction"),
            ("Left motor", "D5 / D6", "DRV8833 IN1 / IN2"),
            ("Right motor", "D9 / D10", "DRV8833 IN3 / IN4"),
            ("Driver sleep", "D8", "DRV8833 EEP, 10 k to GND"),
            ("Button", "D12", "button, other leg to GND"),
            ("I2C SDA", "A4", "MPU-6050, OLED"),
            ("I2C SCL", "A5", "MPU-6050, OLED"),
            ("Serial", "D0 / D1", "USB, 115200 baud")]
    txt(ax, 262, 280, "PIN MAP", 8.5, weight="bold")
    table(ax, 262, 276, [44, 20, 80], rows, rowh=5.2, size=5.8, aligns=["left", "center", "left"])
    rows2 = [("SUPPLY", "FROM", "TO"),
             ("7.4 V", "LiPo (XT60)", "DRV8833 VCC, Uno VIN"),
             ("5 V", "Uno 5V", "both encoders (black)"),
             ("3.3 V", "Uno 3.3V", "MPU-6050, OLED"),
             ("GND", "common", "everything, battery −")]
    table(ax, 262, 208, [20, 34, 90], rows2, rowh=5.2, size=5.8)
    rows3 = [("MOTOR WIRE", "LEFT", "RIGHT"),
             ("red (motor +)", "OUT1", "OUT3"),
             ("white (motor −)", "OUT2", "OUT4"),
             ("green (encoder A)", "D2", "D3"),
             ("yellow (encoder B)", "D4", "D7"),
             ("black (encoder 5 V)", "5V", "5V"),
             ("blue (encoder GND)", "GND", "GND")]
    table(ax, 262, 177, [64, 40, 40], rows3, rowh=5.2, size=5.8, aligns=["left", "center", "center"])

    # ------------------------------------------------ drive module plan
    CX, CY = 112, 128
    U = lambda u: CX + u
    V = lambda v: CY + v
    rrect(ax, CX, CY, PLATE_W, PLATE_D, CORNER_R, THIN, ls=(0, (6, 2, 1, 2)), z=2)
    for s in (1, -1):
        rect(ax, U(s * WHEEL_IN), V(-WHEEL_D / 2), U(s * WHEEL_OUT), V(WHEEL_D / 2), MED, fc="white", z=4)
        rect(ax, U(s * (WHEEL_IN + 0.8)), V(-WHEEL_D / 2 + 1), U(s * (WHEEL_OUT - 0.8)), V(WHEEL_D / 2 - 1), THIN, z=5)
        rect(ax, U(s * INNER), V(-MOTOR_W / 2), U(s * MOTOR_OUT), V(MOTOR_W / 2), MED, fc="white", z=3)
        rect(ax, U(s * (MOTOR_OUT - 9)), V(-MOTOR_W / 2), U(s * MOTOR_OUT), V(MOTOR_W / 2), MED, fc="white", hatch="||||", z=3)
        line(ax, [(U(s * MOTOR_OUT), V(0)), (U(s * WHEEL_IN), V(0))], MED)
        rect(ax, U(s * (SLOT_U - CL_L / 2)), V(-CL_B / 2), U(s * (SLOT_U + CL_L / 2)), V(CL_B / 2), MED, fc="none", z=4)
        for sv in (-1, 1):
            ax.add_patch(Circle((U(s * SLOT_U), V(sv * CL_SP / 2)), M3 / 2, fc="white", ec=INK, lw=MED, zorder=5))
        rrect(ax, U(s * (ENC_U0 + ENC_U1) / 2), CY, ENC_U1 - ENC_U0, ENC_H, ENC_R, THIN, ls=(0, (3, 2)), z=6)
        rect(ax, U(s * INNER), V(-6), U(s * (INNER + 1.2)), V(6), MED, fc="#ccc", z=5)
    centre(ax, U(-62), CY, U(62), CY); centre(ax, CX, V(-38), CX, V(38))
    dim_h(ax, U(-INNER), U(INNER), V(6), f"{GAP_CENTER:.0f}", off=34, size=5.8)
    dim_h(ax, CX, U(SLOT_U), V(CL_B / 2), f"{SLOT_U:.0f}", off=9, size=5.8)
    dim_h(ax, U(SLOT_U - CL_L / 2), U(SLOT_U + CL_L / 2), V(CL_B / 2), f"{CL_L:.0f}", off=19, size=5.8)
    dim_v(ax, V(-CL_B / 2), V(CL_B / 2), U(SLOT_U + CL_L / 2), f"{CL_B:.0f}", off=31, size=5.8)
    dim_h(ax, U(-INNER), U(-WHEEL_OUT), V(-WHEEL_D / 2), f"{MOTOR_PLUS_WHEEL:.0f}  (measured)", off=-8, size=5.8)
    dim_h(ax, U(-WHEEL_C), U(WHEEL_C), V(-WHEEL_D / 2), f"{TRACK:.1f}  (track)", off=-17, size=6.2)
    dim_h(ax, U(WHEEL_IN), U(WHEEL_OUT), V(WHEEL_D / 2), f"{WHEEL_W}", off=6, size=5.6)
    leader(ax, (U(-SLOT_U), V(CL_SP / 2 + 1.5)), (U(-52), V(40)), "motor clamp, M3 screws 20 apart", 5.8, ha="right")
    leader(ax, (U(-(MOTOR_OUT - 4)), V(-MOTOR_W / 2)), (U(-56), V(-37)), "gearbox", 5.8, ha="right")
    leader(ax, (U(-INNER - 0.6), V(-6)), (U(-14), V(-40)), "encoder ends face the centre", 5.8, ha="right")
    txt(ax, CX, V(-50), "chain line: base plate.  dashed: encoder windows above the encoders", 5.5, ha="center", color="#555")
    view_title(ax, CX, 62, "DRIVE MODULE  (PLAN, PLATE TRANSPARENT)")

    # ------------------------------------------------ notes
    nx, ny = 262, 133
    txt(ax, nx, ny, "NOTES", 8, weight="bold")
    for i, n in enumerate(["1.  Pins match firmware/uno/bobot_v7/bobot_v7.ino.",
                           "2.  Full wiring diagram: wiring/uno-wiring-diagram.svg.",
                           "3.  Encoders run from the Uno 5V (Uno inputs are 5 V).",
                           "4.  The 10 k on EEP keeps the driver asleep during reset and upload.",
                           "5.  MPU-6050 flat, chip up, near the axle. Its Z accelerometer is dead,",
                           "     so tilt is measured from the forward axis only.",
                           "6.  The setup wizard (check) finds motor and encoder directions,",
                           "     so the motor wires do not need swapping.",
                           "7.  Check XT60 polarity before power-up. No battery sensing on the",
                           "     Uno: keep the pack above 6.6 V."]):
        txt(ax, nx, ny - 6 - i * 4.5, n, 5.8)
    title_block(ax, "BO-BOT  Self-Balancing Robot", "Electronics and drive module, Arduino Uno build", "2 of 2", "BB-EL-02")
    fig.savefig(os.path.join(OUT, "sheet2-electronics.png"), dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    sheet1()
    sheet2()
    print(f"track {TRACK:.1f}, overall {OVERALL:.0f}, base plate underside {Y_BASE:.2f}, deck 3 top {Y_D3 + DECK_T:.2f}")
