"""
The robot's face on the 1.3" SH1106 OLED (128 x 64, 1-bit).

Drawn with a handful of primitives (filled rounded rectangles, ellipses,
thick lines, arcs) that map directly onto U8g2 calls on the ESP32
(drawRBox, drawFilledEllipse, drawLine, drawCircle...), so faces you design
here can be ported to the firmware unchanged.

Expressions are chosen from what the firmware knows (angle, pitch rate,
commands, fallen, low battery) -- exactly the information the real robot has.
"""
import math
import random
import numpy as np

W, H = 128, 64
YY, XX = np.mgrid[0:H, 0:W]


def rbox(img, x0, y0, x1, y1, r, val=True):
    r = max(0.0, min(r, (x1 - x0) / 2, (y1 - y0) / 2))
    cx = np.clip(XX, x0 + r, x1 - r)
    cy = np.clip(YY, y0 + r, y1 - r)
    m = (XX >= x0) & (XX <= x1) & (YY >= y0) & (YY <= y1) & ((XX - cx) ** 2 + (YY - cy) ** 2 <= r * r + 0.5)
    img[m] = val


def ellipse(img, cx, cy, rx, ry, val=True):
    if rx <= 0 or ry <= 0:
        return
    img[((XX - cx) / rx) ** 2 + ((YY - cy) / ry) ** 2 <= 1.0] = val


def tline(img, x0, y0, x1, y1, w, val=True):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy or 1e-9
    t = np.clip(((XX - x0) * dx + (YY - y0) * dy) / L2, 0, 1)
    px, py = x0 + t * dx, y0 + t * dy
    img[(XX - px) ** 2 + (YY - py) ** 2 <= (w / 2) ** 2] = val


def arc(img, cx, cy, r, a0, a1, w, val=True):
    d = np.sqrt((XX - cx) ** 2 + (YY - cy) ** 2)
    ang = np.degrees(np.arctan2(YY - cy, XX - cx)) % 360
    a0, a1 = a0 % 360, a1 % 360
    inside = (ang >= a0) & (ang <= a1) if a0 <= a1 else (ang >= a0) | (ang <= a1)
    img[(np.abs(d - r) <= w / 2) & inside] = val


class Face:
    EYE_L, EYE_R, EYE_Y = 36, 92, 26

    def __init__(self, seed=4):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.next_blink = 2.0
        self.blink_t = -1.0
        self.mood = "idle"
        self.hold = 0.0
        self.look = 0.0       # -1 .. 1 (eyes left/right)
        self.lookv = 0.0      # -1 .. 1 (eyes up/down)

    def choose(self, info, dt):
        """info: angle, rate (deg/s), v_cmd, yaw_cmd, fallen, low_batt, pushed."""
        self.hold = max(0.0, self.hold - dt)
        if info["fallen"]:
            return "dizzy"
        if info["low_batt"]:
            return "sleepy"
        if abs(info["rate"]) > 90 or info.get("pushed"):
            self.hold = 1.2
            self.mood = "surprised"
            return "surprised"
        if self.hold > 0:
            return self.mood
        if abs(info["angle"]) > 12:
            return "worried"
        if info["v_cmd"] > 0.05:
            return "happy"
        if info["v_cmd"] < -0.05:
            return "careful"
        return "idle"

    def update(self, dt, info):
        self.t += dt
        mood = self.choose(info, dt)
        # eyes follow turning (screen left = robot's right), lean shifts them
        tgt = 0.0
        if info["yaw_cmd"] > 10:
            tgt = -1.0
        elif info["yaw_cmd"] < -10:
            tgt = 1.0
        self.look += (tgt - self.look) * min(1, dt * 8)
        tv = max(-1.0, min(1.0, info["angle"] / 15))
        self.lookv += (tv - self.lookv) * min(1, dt * 8)
        # blinking
        if self.t > self.next_blink and mood in ("idle", "happy", "careful", "worried"):
            self.blink_t = 0.0
            self.next_blink = self.t + self.rng.uniform(2.0, 5.0)
        blink = 1.0
        if self.blink_t >= 0:
            self.blink_t += dt
            ph = self.blink_t / 0.16
            blink = abs(1 - 2 * ph) if ph < 1 else 1.0
            if ph >= 1:
                self.blink_t = -1
        return self.draw(mood, blink), mood

    def draw(self, mood, blink=1.0):
        img = np.zeros((H, W), dtype=bool)
        lx, rx, ey = self.EYE_L, self.EYE_R, self.EYE_Y
        ox = self.look * 10
        oy = self.lookv * 5
        if mood == "dizzy":
            for cx in (lx, rx):
                tline(img, cx - 11, ey - 11, cx + 11, ey + 11, 5)
                tline(img, cx - 11, ey + 11, cx + 11, ey - 11, 5)
            # wobbly mouth
            for i in range(5):
                x0 = 46 + i * 8
                tline(img, x0, 52 + (i % 2) * 4, x0 + 8, 52 + ((i + 1) % 2) * 4, 3)
            return img
        if mood == "sleepy":
            for cx in (lx, rx):
                rbox(img, cx - 14, ey + 2, cx + 14, ey + 8, 3)
            tline(img, 56, 52, 72, 52, 3)
            # battery icon
            rbox(img, 104, 46, 122, 58, 2)
            rbox(img, 106, 48, 120, 56, 1, False)
            rbox(img, 107, 49, 109, 55, 0)
            rbox(img, 122, 49, 124, 55, 0)
            return img
        if mood == "surprised":
            for cx in (lx, rx):
                ellipse(img, cx, ey, 15, 17)
                ellipse(img, cx, ey, 9, 11, False)
                ellipse(img, cx, ey, 5, 6)
            ellipse(img, 64, 53, 6, 7)
            ellipse(img, 64, 53, 3, 4, False)
            return img
        if mood == "happy":
            for cx in (lx, rx):
                arc(img, cx + ox, ey + 8 + oy, 13, 200, 340, 6)
            arc(img, 64, 40, 14, 25, 155, 4)
            return img
        # idle / careful / worried: rounded-rect eyes
        h = 30 * blink
        if mood == "careful":
            h = 22 * blink
        for i, cx in enumerate((lx, rx)):
            x0, x1 = cx - 13 + ox, cx + 13 + ox
            y0, y1 = ey - h / 2 + oy, ey + h / 2 + oy
            if h < 3:
                tline(img, x0, ey + oy, x1, ey + oy, 3)
                continue
            rbox(img, x0, y0, x1, y1, 7)
            if mood == "worried":
                # slanted brow cuts the inner top corner
                s = 1 if i == 0 else -1
                xi = x1 if i == 0 else x0
                for k in range(14):
                    tline(img, xi - s * 26, y0 - 4 - k, xi, y0 + 10 - k, 2, False)
        if mood == "worried":
            for i in range(4):
                tline(img, 50 + i * 7, 54 + (i % 2) * 3, 57 + i * 7, 54 + ((i + 1) % 2) * 3, 3)
        elif mood == "careful":
            tline(img, 58, 54, 70, 54, 3)
        else:
            arc(img, 64, 44, 9, 30, 150, 3)
        return img


if __name__ == "__main__":
    f = Face()
    for m in ("idle", "happy", "careful", "worried", "surprised", "dizzy", "sleepy"):
        img = f.draw(m)
        print(m)
        for row in img[::4]:
            print("".join("#" if v else "." for v in row[::2]))
