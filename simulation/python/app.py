"""
Interactive 2.5D simulator.   python app.py   [--shell]

Drive it like the Bluetooth version will:
  Up / Down        drive forward / backward      (hold Shift = faster)
  Left / Right     turn
  X / Z            push forward / backward at the head   (Shift = harder)
  R                pick up and put down again (reset)
  S                shell on/off              O  x-ray (see inside the shell)
  [ / ]            select gain               - / =  decrease / increase it
  1 2 3 4          sim speed 0.25x 0.5x 1x 2x        P  pause
  C                camera: chase / orbit / side      mouse drag = orbit, wheel = zoom
  B                drain battery to 15 %             H  help on/off
"""
import argparse
import math
import sys
import time

import numpy as np
import pygame

from sim import Sim
from params import make_params
from face import Face
from physics import ITH, IPSI, IX, IY, IAL, IAR, IV, ITHD, IPSID

WIN_W, WIN_H = 1360, 800
VIEW_W = 860
PANEL_X = VIEW_W
BG_TOP = (196, 214, 232)
BG_BOT = (232, 236, 240)
PANEL_BG = (24, 26, 32)
TEXT = (225, 228, 235)
DIM = (140, 146, 160)
ACCENT = (90, 200, 255)
OLED_ON = (110, 200, 255)     # blue SH1106
GAIN_KEYS = ["kp", "kd", "ki", "kp_v", "ki_v", "kp_t", "ki_t", "trim", "kff"]
LIGHT = np.array([-0.35, 0.45, 0.82])
LIGHT /= np.linalg.norm(LIGHT)


# ---------------------------------------------------------------------- math
def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


BOX_FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
#  corner index bits: x(4) y(2) z(1)


def box_corners(c, s):
    cx, cy, cz = c
    dx, dy, dz = s[0] / 2, s[1] / 2, s[2] / 2
    return np.array([[cx + sx * dx, cy + sy * dy, cz + sz * dz]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def convex_hull(pts):
    pts = sorted(set(map(tuple, pts)))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


class Camera:
    def __init__(self):
        self.az = math.radians(40)
        self.el = math.radians(18)
        self.dist = 0.42
        self.fov = math.radians(45)
        self.mode = 0          # 0 chase, 1 orbit (free), 2 side
        self.target = np.array([0, 0, 0.05])

    def setup(self, robot_xy, robot_psi):
        if self.mode == 0:
            az = robot_psi + math.radians(40)
            # smooth follow
            d = (az - self.az + math.pi) % (2 * math.pi) - math.pi
            self.az += d * 0.06
        elif self.mode == 2:
            self.az = robot_psi - math.pi / 2
        tgt = np.array([robot_xy[0], robot_xy[1], 0.05])
        self.target = self.target + (tgt - self.target) * 0.15
        ce = math.cos(self.el)
        self.eye = self.target + self.dist * np.array([ce * math.cos(self.az), ce * math.sin(self.az), math.sin(self.el)])
        f = self.target - self.eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        self.basis = np.array([r, u, f])
        self.fpx = (WIN_H / 2) / math.tan(self.fov / 2)

    def project(self, P):
        """P: (n,3) world -> (n,2) screen, (n,) depth."""
        C = (P - self.eye) @ self.basis.T
        z = C[:, 2]
        zc = np.maximum(z, 1e-4)
        sx = VIEW_W / 2 + self.fpx * C[:, 0] / zc
        sy = WIN_H / 2 - self.fpx * C[:, 1] / zc
        return np.stack([sx, sy], 1), z


# ---------------------------------------------------------------------- app
class App:
    def __init__(self, shell=False):
        pygame.init()
        pygame.display.set_caption("ESP32 self-balancing robot — 2.5D simulator")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.font = pygame.font.SysFont("dejavusansmono,consolas,menlo,monospace", 14)
        self.font_b = pygame.font.SysFont("dejavusans,arial,helvetica", 18, bold=True)
        self.font_s = pygame.font.SysFont("dejavusansmono,consolas,menlo,monospace", 12)
        self.cam = Camera()
        self.face = Face()
        self.shell = shell
        self.xray = False
        self.speed = 1.0
        self.paused = False
        self.help = True
        self.sel = 0
        self.trail = []
        self.push_vis = 0.0
        self.push_dir = 1
        self.face_img = np.zeros((64, 128), bool)
        self.mood = "idle"
        self.drag = None
        self.sim = Sim(shell=shell, log_seconds=8)
        self.gains = self.sim.gains
        self.sky = self._make_sky()
        self.clock = pygame.time.Clock()
        self.real_factor = 1.0

    def _make_sky(self):
        s = pygame.Surface((VIEW_W, WIN_H))
        for y in range(WIN_H):
            t = y / WIN_H
            c = [int(BG_TOP[i] + (BG_BOT[i] - BG_TOP[i]) * t) for i in range(3)]
            pygame.draw.line(s, c, (0, y), (VIEW_W, y))
        return s

    def rebuild(self):
        g = self.gains
        self.sim = Sim(shell=self.shell, gains=g, log_seconds=8)
        self.trail = []

    # ------------------------------------------------------------ input
    def handle(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            if ev.type == pygame.KEYDOWN:
                k = ev.key
                shift = ev.mod & pygame.KMOD_SHIFT
                if k == pygame.K_ESCAPE:
                    return False
                elif k == pygame.K_r:
                    self.sim.reset()
                    self.trail = []
                elif k == pygame.K_s:
                    self.shell = not self.shell
                    self.rebuild()
                elif k == pygame.K_o:
                    self.xray = not self.xray
                elif k in (pygame.K_x, pygame.K_z):
                    d = 1 if k == pygame.K_x else -1
                    imp = 0.035 if shift else 0.02
                    self.sim.push(d * imp, 0.05, height=self.sim.p.l + 0.04)
                    self.push_vis, self.push_dir = 0.35, d
                elif k == pygame.K_p:
                    self.paused = not self.paused
                elif k in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    self.speed = {pygame.K_1: 0.25, pygame.K_2: 0.5, pygame.K_3: 1.0, pygame.K_4: 2.0}[k]
                elif k == pygame.K_c:
                    self.cam.mode = (self.cam.mode + 1) % 3
                elif k == pygame.K_h:
                    self.help = not self.help
                elif k == pygame.K_b:
                    self.sim.plant.soc = 0.15
                elif k == pygame.K_LEFTBRACKET:
                    self.sel = (self.sel - 1) % len(GAIN_KEYS)
                elif k == pygame.K_RIGHTBRACKET:
                    self.sel = (self.sel + 1) % len(GAIN_KEYS)
                elif k in (pygame.K_MINUS, pygame.K_EQUALS, pygame.K_KP_MINUS, pygame.K_KP_PLUS):
                    up = k in (pygame.K_EQUALS, pygame.K_KP_PLUS)
                    name = GAIN_KEYS[self.sel]
                    v = getattr(self.gains, name)
                    if name == "trim":
                        v += 0.25 if up else -0.25
                    elif v == 0:
                        v = 0.1 if up else 0.0
                    else:
                        v *= 1.1 if up else 1 / 1.1
                        if v < 0.005:
                            v = 0.0
                    setattr(self.gains, name, round(v, 4))
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and ev.pos[0] < VIEW_W:
                self.drag = ev.pos
                if self.cam.mode != 1:
                    self.cam.mode = 1
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self.drag = None
            elif ev.type == pygame.MOUSEMOTION and self.drag:
                dx, dy = ev.pos[0] - self.drag[0], ev.pos[1] - self.drag[1]
                self.cam.az -= dx * 0.008
                self.cam.el = min(math.radians(80), max(math.radians(3), self.cam.el + dy * 0.006))
                self.drag = ev.pos
            elif ev.type == pygame.MOUSEWHEEL:
                self.cam.dist = min(2.0, max(0.18, self.cam.dist * (0.9 if ev.y > 0 else 1.1)))
        keys = pygame.key.get_pressed()
        fast = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        v = 0.0
        if keys[pygame.K_UP]:
            v = 0.35 if fast else 0.2
        elif keys[pygame.K_DOWN]:
            v = -(0.35 if fast else 0.2)
        yaw = 0.0
        if keys[pygame.K_LEFT]:
            yaw = 120.0
        elif keys[pygame.K_RIGHT]:
            yaw = -120.0
        self.sim.v_cmd, self.sim.yaw_cmd = v, yaw
        return True

    # ------------------------------------------------------------ geometry
    def robot_polys(self):
        sim = self.sim
        p = sim.p
        x = sim.plant.x
        T = np.array([x[IX], x[IY], p.r])
        RA = rot_z(x[IPSI])
        RB = RA @ rot_y(x[ITH])
        polys = []      # (depth, pts2d, color, kind/extra)
        shadows = []
        mm = 0.001

        FI = np.array(BOX_FACES)
        eye = self.cam.eye

        def add_box(center_mm, size_mm, color, R=RB, T_=T, overlay=None, shade=True, shadow=True):
            C = box_corners(np.array(center_mm) * mm, np.array(size_mm) * mm)
            W = C @ R.T + T_
            if shadow:
                shadows.append(W)
            S, Z = self.cam.project(W)
            if (Z < 0.02).any():
                return
            a, b, c_ = W[FI[:, 0]], W[FI[:, 1]], W[FI[:, 2]]
            n = np.cross(b - a, c_ - a)
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-15
            ctr = W[FI].mean(1)
            vis = np.einsum("ij,ij->i", n, eye - ctr) > 0
            lam = n @ LIGHT
            depths = Z[FI].mean(1)
            for fi in np.nonzero(vis)[0]:
                k = 0.55 + 0.45 * max(0.0, float(lam[fi])) if shade else 1.0
                col = (int(min(255, color[0] * k)), int(min(255, color[1] * k)), int(min(255, color[2] * k)))
                polys.append((float(depths[fi]), S[FI[fi]], col, None))

        comps = p.active_components()
        shell_on = self.shell
        for name, mg, c, s, col, grp in comps:
            if grp == "hidden":
                continue
            if grp == "shell":
                continue
            if shell_on and not self.xray and name not in ("oled",):
                continue
            if grp == "frame_hidden":
                # draw 4 thin posts
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        add_box((c[0] + sx * (s[0] / 2 - 3), c[1] + sy * (s[1] / 2 - 3), c[2]),
                                (4, 4, s[2]), (220, 220, 225))
                continue
            add_box(c, s, col)
        if shell_on:
            for name, mg, c, s, col, grp in comps:
                if grp != "shell":
                    continue
                if self.xray:
                    self._wire_box(c, s, RB, T, polys)
                    continue
                if name == "ears":
                    for sy in (-1, 1):
                        add_box((c[0], sy * 24, c[2]), (14, 14, 12), (250, 200, 210))
                        add_box((c[0], sy * 24, c[2] - 6), (18, 18, 2), (250, 235, 215))
                elif name == "bow_tie":
                    for sy in (-1, 1):
                        add_box((c[0], sy * 8, c[2]), (s[0], 14, s[2]), col)
                    add_box((c[0] + 1, 0, c[2]), (s[0], 6, 7), (180, 20, 40))
                else:
                    add_box(c, s, col)
        # wheels (in heading frame, not pitched), rotate with wheel angle
        for side, ang in ((1, x[IAL]), (-1, x[IAR])):
            self._wheel(side, ang, RA, T, polys, shadows)
        return polys, shadows

    def _wire_box(self, c, s, R, T, polys):
        C = box_corners(np.array(c) * 0.001, np.array(s) * 0.001) @ R.T + T
        S, Z = self.cam.project(C)
        for f in BOX_FACES:
            polys.append((float(np.mean(Z[list(f)])) - 1.0, S[list(f)], (250, 235, 215), "wire"))

    def _wheel(self, side, ang, RA, T, polys, shadows):
        p = self.sim.p
        r, w = p.r, p.wheel.width
        n = 22
        yc = side * p.d / 2
        a = np.linspace(0, 2 * math.pi, n, endpoint=False) + ang
        outer = np.stack([r * np.sin(a), np.full(n, yc + w / 2 * side), r * np.cos(a)], 1)
        inner = np.stack([r * np.sin(a), np.full(n, yc - w / 2 * side), r * np.cos(a)], 1)
        hub = np.stack([0.7 * r * np.sin(a), np.full(n, yc + w / 2 * side + 0.0003 * side), 0.7 * r * np.cos(a)], 1)
        Wo, Wi, Wh = outer @ RA.T + T, inner @ RA.T + T, hub @ RA.T + T
        shadows.append(np.vstack([Wo, Wi]))
        So, Zo = self.cam.project(Wo)
        Si, Zi = self.cam.project(Wi)
        Sh, Zh = self.cam.project(Wh)
        if (Zo < 0.02).any():
            return
        tyre = (48, 48, 52)
        for i in range(n):
            j = (i + 1) % n
            quad = np.array([Wo[i], Wo[j], Wi[j], Wi[i]])
            nrm = (quad[0] + quad[1]) / 2 - (T + RA @ np.array([0, yc, 0]))
            nrm /= np.linalg.norm(nrm) + 1e-12
            if np.dot(nrm, self.cam.eye - quad.mean(0)) <= 0:
                continue
            k = 0.55 + 0.45 * max(0.0, float(nrm @ LIGHT))
            polys.append((float((Zo[i] + Zo[j] + Zi[i] + Zi[j]) / 4), np.array([So[i], So[j], Si[j], Si[i]]),
                          tuple(int(c * k) for c in tyre), None))
        axis = RA @ np.array([0, side, 0])
        if np.dot(axis, self.cam.eye - Wo.mean(0)) > 0:
            polys.append((float(Zo.mean()) - 0.0005, So, (60, 60, 64), None))
            polys.append((float(Zh.mean()) - 0.0010, Sh, (245, 245, 245), ("spokes", Sh)))
        else:
            polys.append((float(Zi.mean()) - 0.0005, Si, (60, 60, 64), None))

    # ------------------------------------------------------------ drawing
    def draw_ground(self):
        cam = self.cam
        X, Y = self.sim.plant.x[IX], self.sim.plant.x[IY]
        tile = 0.1
        R = 12
        x0, y0 = math.floor(X / tile) * tile, math.floor(Y / tile) * tile
        xs = x0 + np.arange(-R, R + 1) * tile
        ys = y0 + np.arange(-R, R + 1) * tile
        GX, GY = np.meshgrid(xs, ys, indexing="ij")
        P = np.stack([GX.ravel(), GY.ravel(), np.zeros(GX.size)], 1)
        S, Z = cam.project(P)
        S = S.reshape(len(xs), len(ys), 2)
        Z = Z.reshape(len(xs), len(ys))
        quads = []
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                zz = (Z[i, j], Z[i + 1, j], Z[i + 1, j + 1], Z[i, j + 1])
                if min(zz) < 0.03:
                    continue
                dist = math.hypot(xs[i] + tile / 2 - X, ys[j] + tile / 2 - Y)
                fade = max(0.0, 1 - dist / (R * tile))
                checker = (int(round(xs[i] / tile)) + int(round(ys[j] / tile))) % 2
                base = (208, 200, 186) if checker else (196, 188, 174)
                col = tuple(int(BG_BOT[k] + (base[k] - BG_BOT[k]) * fade) for k in range(3))
                quads.append((sum(zz), [S[i, j], S[i + 1, j], S[i + 1, j + 1], S[i, j + 1]], col))
        quads.sort(key=lambda q: -q[0])
        for _, pts, col in quads:
            pygame.draw.polygon(self.screen, col, pts)
        # trail
        if len(self.trail) > 2:
            P = np.array([[a, b, 0.0005] for a, b in self.trail])
            S, Z = cam.project(P)
            pts = [tuple(s) for s, z in zip(S, Z) if z > 0.03]
            if len(pts) > 2:
                pygame.draw.lines(self.screen, (120, 150, 190), False, pts, 2)

    def draw_shadows(self, shadows):
        if not hasattr(self, "_shadow_surf"):
            self._shadow_surf = pygame.Surface((VIEW_W, WIN_H)).convert()
            self._shadow_surf.set_colorkey((255, 0, 255))
        surf = self._shadow_surf
        surf.fill((255, 0, 255))
        L = LIGHT
        for W in shadows:
            G = W - np.outer(W[:, 2] / L[2], L)
            G[:, 2] = 0.0002
            S, Z = self.cam.project(G)
            if (Z < 0.03).any():
                continue
            hull = convex_hull([tuple(np.round(s, 1)) for s in S])
            if len(hull) >= 3:
                pygame.draw.polygon(surf, (40, 40, 60), hull)
        surf.set_alpha(70)
        self.screen.blit(surf, (0, 0))

    def draw_robot(self, polys):
        polys.sort(key=lambda q: -q[0])
        for depth, pts, col, extra in polys:
            pts = [tuple(p) for p in pts]
            if extra == "wire":
                pygame.draw.polygon(self.screen, (150, 140, 130), pts, 1)
                continue
            pygame.draw.polygon(self.screen, col, pts)
            if isinstance(extra, tuple) and extra[0] == "spokes":
                S = extra[1]
                c = np.mean(S, 0)
                for k in range(0, len(S), len(S) // 3):
                    pygame.draw.line(self.screen, (120, 120, 125), c, S[k], 2)
                pygame.draw.circle(self.screen, (90, 90, 95), c.astype(int), 3)
        self.draw_oled_on_robot()

    def draw_oled_on_robot(self):
        """Paint the live face bitmap onto the OLED glass (front face of the module)."""
        sim, p = self.sim, self.sim.p
        oled = [c for c in p.active_components() if c[0] == "oled"]
        if not oled:
            return
        _, _, (cx, cy, cz), (dx, dy, dz), *_ = oled[0]
        x = sim.plant.x
        R = rot_z(x[IPSI]) @ rot_y(x[ITH])
        T = np.array([x[IX], x[IY], p.r])
        normal = R @ np.array([1.0, 0, 0])
        front = (np.array([cx + dx / 2 + 0.3, cy, cz]) * 0.001) @ R.T + T
        if np.dot(normal, self.cam.eye - front) <= 0:
            return
        # 1.3" active area ~29.4 x 14.7 mm, upper part of the module
        aw, ah = 29.4, 14.7
        top = cz + dz / 2 - 6
        img = self.face_img[::2, ::2]           # 32 x 64 blocks
        rows, cols = img.shape
        bw, bh = aw / cols, ah / rows
        ys, xs = np.nonzero(img)
        # glass background
        glass = np.array([[cx + dx / 2 + 0.2, cy - aw / 2 - 1.5, top + 1.5], [cx + dx / 2 + 0.2, cy + aw / 2 + 1.5, top + 1.5],
                          [cx + dx / 2 + 0.2, cy + aw / 2 + 1.5, top - ah - 1.5], [cx + dx / 2 + 0.2, cy - aw / 2 - 1.5, top - ah - 1.5]]) * 0.001
        Sg, _ = self.cam.project(glass @ R.T + T)
        pygame.draw.polygon(self.screen, (8, 10, 16), [tuple(s) for s in Sg])
        if len(xs) == 0:
            return
        # viewer's left = robot's right (-y)
        y0 = cy - aw / 2 + xs * bw
        z0 = top - ys * bh
        xf = np.full(len(xs), cx + dx / 2 + 0.4)
        corners = np.stack([
            np.stack([xf, y0, z0], 1), np.stack([xf, y0 + bw, z0], 1),
            np.stack([xf, y0 + bw, z0 - bh], 1), np.stack([xf, y0, z0 - bh], 1)], 1) * 0.001
        Wc = corners.reshape(-1, 3) @ R.T + T
        S, _ = self.cam.project(Wc)
        S = S.reshape(-1, 4, 2)
        for q in S:
            pygame.draw.polygon(self.screen, OLED_ON, [tuple(v) for v in q])

    def draw_push_arrow(self):
        if self.push_vis <= 0:
            return
        sim, p = self.sim, self.sim.p
        x = sim.plant.x
        R = rot_z(x[IPSI]) @ rot_y(x[ITH])
        T = np.array([x[IX], x[IY], p.r])
        h = p.l + 0.04
        d = self.push_dir
        tip = np.array([-d * 0.035, 0, h]) @ R.T + T
        tail = np.array([-d * 0.11, 0, h]) @ R.T + T
        S, Z = self.cam.project(np.array([tail, tip]))
        if (Z > 0.03).all():
            pygame.draw.line(self.screen, (230, 60, 50), S[0], S[1], 5)
            pygame.draw.circle(self.screen, (230, 60, 50), S[1].astype(int), 7)

    # ------------------------------------------------------------ panel
    def text(self, s, pos, col=TEXT, font=None):
        self.screen.blit((font or self.font).render(s, True, col), pos)

    def plot(self, rect, series, ylim, title, unit=""):
        x0, y0, w, h = rect
        pygame.draw.rect(self.screen, (34, 37, 46), rect, border_radius=6)
        self.text(title, (x0 + 8, y0 + 4), DIM, self.font_s)
        lo, hi = ylim
        if lo < 0 < hi:
            yz = y0 + h - (0 - lo) / (hi - lo) * h
            pygame.draw.line(self.screen, (60, 64, 76), (x0, yz), (x0 + w, yz))
        n_keep = 6 * 200
        for data, col, label in series:
            d = list(data)[-n_keep:]
            if len(d) < 2:
                continue
            step = max(1, len(d) // w)
            d = d[::step]
            pts = []
            for i, v in enumerate(d):
                px = x0 + w - (len(d) - 1 - i) * w / (n_keep / step)
                py = y0 + h - (min(hi, max(lo, v)) - lo) / (hi - lo) * h
                pts.append((px, py))
            pygame.draw.lines(self.screen, col, False, pts, 2 if label else 1)
        lx = x0 + w - 8
        for data, col, label in reversed(series):
            if label:
                img = self.font_s.render(label, True, col)
                lx -= img.get_width() + 10
                self.screen.blit(img, (lx, y0 + 4))
        self.text(f"{hi:g}{unit}", (x0 + 4, y0 + 18), (90, 96, 110), self.font_s)
        self.text(f"{lo:g}{unit}", (x0 + 4, y0 + h - 16), (90, 96, 110), self.font_s)

    def draw_panel(self):
        sim, p, g, c = self.sim, self.sim.p, self.gains, self.sim.ctrl
        pygame.draw.rect(self.screen, PANEL_BG, (PANEL_X, 0, WIN_W - PANEL_X, WIN_H))
        x0 = PANEL_X + 16
        # OLED
        self.text("SH1106 1.3\" OLED  (128x64)", (x0, 12), DIM, self.font_s)
        img = np.zeros((64, 128, 3), np.uint8)
        img[self.face_img] = OLED_ON
        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
        surf = pygame.transform.scale(surf, (256, 128))
        pygame.draw.rect(self.screen, (6, 8, 12), (x0 - 6, 28, 268, 140), border_radius=6)
        self.screen.blit(surf, (x0, 34))
        # status
        state = "FALLEN  (press R)" if sim.fallen else ("LOW BATTERY" if c.low_batt else "BALANCING")
        scol = (255, 110, 90) if sim.fallen or c.low_batt else (120, 230, 140)
        sx = x0 + 280
        self.text(state, (sx, 30), scol, self.font_b)
        x = sim.plant.x
        lines = [
            f"t      {sim.plant.t:6.1f} s  x{self.speed:g}{'  PAUSED' if self.paused else ''}",
            f"pitch  {x[ITH]*180/math.pi:+6.2f} deg",
            f"est.   {c.angle:+6.2f} deg",
            f"speed  {x[IV]:+5.2f} m/s",
            f"yaw    {x[IPSID]*180/math.pi:+5.0f} deg/s",
            f"batt   {sim.plant.vbat:4.2f} V {sim.plant.soc*100:3.0f}%",
            f"draw   {sim.plant.ibat*1000:4.0f} mA",
            f"face   {self.mood}",
        ]
        for i, s in enumerate(lines):
            self.text(s, (sx, 58 + i * 16), TEXT, self.font_s)
        # plots
        L = sim.log
        units = dict(kp="PWM/deg", kd="PWM/(deg/s)", ki="PWM/(deg s)", kp_v="deg/(m/s)", ki_v="deg/m",
                     kp_t="PWM/(deg/s)", ki_t="PWM/deg", trim="deg", kff="x")
        pw = WIN_W - PANEL_X - 32
        y = 182
        self.plot((x0, y, pw, 104), [(L["th"], (255, 200, 80), "true"), (L["est"], (90, 200, 255), "MPU est."),
                                     (L["tgt"], (180, 120, 255), "target")], (-15, 15), "pitch [deg]")
        y += 110
        self.plot((x0, y, pw, 90), [(L["pwmL"], (120, 230, 140), "L"), (L["pwmR"], (255, 120, 120), "R")],
                  (-255, 255), "motor PWM [counts]")
        y += 96
        self.plot((x0, y, pw, 80), [(L["v"], (255, 200, 80), "speed"), (L["vcmd"], (90, 200, 255), "command")],
                  (-0.6, 0.6), "speed [m/s]")
        y += 86
        self.plot((x0, y, pw, 56), [(L["vbat"], (120, 230, 140), "V")], (6.4, 8.6), "battery [V]")
        y += 64
        # gains
        self.text("FIRMWARE GAINS   [ ] select   - = adjust   unit: " + units[GAIN_KEYS[self.sel]], (x0, y), DIM, self.font_s)
        y += 18
        for i, k in enumerate(GAIN_KEYS):
            col_ = ACCENT if i == self.sel else TEXT
            xx = x0 + (i % 3) * 160
            yy = y + (i // 3) * 18
            self.text(f"{'>' if i == self.sel else ' '}{k:5s}{getattr(g, k):7.3g}", (xx, yy), col_, self.font_s)
        # live diagnostics
        y += 62
        pl = list(L["pwmL"])[-400:]
        sat = sum(1 for v in pl if abs(v) >= g.pwm_cap - 1) / max(1, len(pl))
        trac = sim.plant.traction_ratio()
        self.peak_trac = max(getattr(self, "peak_trac", 0.0) * 0.995, trac)
        diag = [
            (f"PWM at cap (last 2 s)  {sat*100:4.0f} %", (255, 160, 90) if sat > 0.1 else TEXT),
            (f"grip used (peak)       {self.peak_trac*100:4.0f} %  {'SLIP!' if self.peak_trac > 1 else ''}",
             (255, 110, 90) if self.peak_trac > 1 else TEXT),
            (f"motor current L/R  {sim.plant.iL*1000:4.0f} / {sim.plant.iR*1000:4.0f} mA", TEXT),
        ]
        for i, (s_, col_) in enumerate(diag):
            self.text(s_, (x0, y + i * 16), col_, self.font_s)
        y += 56
        tips = ["drifts away while standing -> adjust trim (balance point)",
                "buzzing / PWM chatter -> lower kd (gear backlash)",
                "slow sway back and forth -> lower kp_v or ki_v",
                "falls when you push -> raise kp a little"]
        for i, s_ in enumerate(tips):
            self.text(s_, (x0, y + i * 15), (110, 116, 130), self.font_s)
        info = f"{p.total_mass*1000:.0f} g  COM {p.l*1000:.0f} mm  shell {'ON' if self.shell else 'off'}  " \
               f"{'x-ray' if self.xray else ''}"
        self.text(info, (x0, WIN_H - 22), DIM, self.font_s)
        if self.real_factor < 0.97 and not self.paused:
            self.text(f"(running at {self.real_factor*100:.0f}% real time)", (x0 + 280, WIN_H - 22), (255, 160, 90), self.font_s)

    def draw_help(self):
        lines = [
            "Up/Down drive (Shift faster)   Left/Right turn",
            "X / Z push fwd/back (Shift harder)   R pick up / reset",
            "S shell   O x-ray   B drain battery   C camera   drag/wheel orbit/zoom",
            "1-4 sim speed   P pause   [ ] - = tune gains   H hide help",
        ]
        w = 540
        surf = pygame.Surface((w, 20 * len(lines) + 14), pygame.SRCALPHA)
        surf.fill((20, 24, 32, 180))
        self.screen.blit(surf, (12, 12))
        for i, s in enumerate(lines):
            self.text(s, (22, 19 + i * 20), (235, 238, 245), self.font_s)

    # ------------------------------------------------------------ loop
    def frame(self, dt_real):
        sim = self.sim
        if not self.paused:
            want = min(0.05, dt_real) * self.speed
            t0 = time.perf_counter()
            sim.advance(want)
            took = time.perf_counter() - t0
            self.real_factor = min(1.0, (want / self.speed) / max(took, 1e-6)) if took > want / self.speed else 1.0
            x = sim.plant.x
            if not self.trail or math.hypot(self.trail[-1][0] - x[IX], self.trail[-1][1] - x[IY]) > 0.01:
                self.trail.append((x[IX], x[IY]))
                self.trail = self.trail[-400:]
            c = sim.ctrl
            info = dict(angle=c.angle, rate=x[ITHD] * 180 / math.pi, v_cmd=c.v_cmd, yaw_cmd=c.yaw_cmd,
                        fallen=sim.fallen, low_batt=c.low_batt, pushed=self.push_vis > 0.3)
            self.face_img, self.mood = self.face.update(want, info)
            self.push_vis = max(0.0, self.push_vis - want)
        x = sim.plant.x
        self.cam.setup((x[IX], x[IY]), x[IPSI])
        self.screen.blit(self.sky, (0, 0))
        self.draw_ground()
        polys, shadows = self.robot_polys()
        self.draw_shadows(shadows)
        self.draw_robot(polys)
        self.draw_push_arrow()
        if self.help:
            self.draw_help()
        self.draw_panel()

    def run(self, max_seconds=None, screenshot=None, script=None):
        t_start = time.time()
        last = time.perf_counter()
        frame_i = 0
        while True:
            if script:
                script(self, frame_i)
            if not self.handle():
                break
            now = time.perf_counter()
            dt = now - last
            last = now
            if max_seconds is not None:
                dt = 1 / 60
            self.frame(dt)
            pygame.display.flip()
            frame_i += 1
            if max_seconds is not None and frame_i >= max_seconds * 60:
                if screenshot:
                    pygame.image.save(self.screen, screenshot)
                break
            if max_seconds is None:
                self.clock.tick(60)
        pygame.quit()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shell", action="store_true", help="start with the 3D-printed shell on")
    a = ap.parse_args()
    App(shell=a.shell).run()
