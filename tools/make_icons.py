"""Generates the toolbar icons (pure Python, no dependencies). Run: python tools/make_icons.py"""

import math
import os
import struct
import zlib

ROOT = os.path.join(os.path.dirname(__file__), "..", "resources")


def png(path, w, h, rgba):
    raw = b"".join(b"\x00" + bytes(rgba[y * w * 4:(y + 1) * w * 4]) for y in range(h))

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay or 1)))
    return math.hypot(p[0] - a[0] - ax * t, p[1] - a[1] - ay * t)


def render(size, strokes):
    """strokes: [(polyline in 0..1 coords, width in 0..1, rgb)]"""
    buf = [0] * (size * size * 4)
    for poly, width, rgb in strokes:
        pts = [(x * size, y * size) for x, y in poly]
        half = max(width * size / 2, 0.6)
        for y in range(size):
            for x in range(size):
                p = (x + 0.5, y + 0.5)
                d = min(seg_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))
                a = max(0.0, min(1.0, half + 0.5 - d))
                if a <= 0:
                    continue
                i = (y * size + x) * 4
                old = buf[i + 3] / 255
                na = a + old * (1 - a)
                for k in range(3):
                    buf[i + k] = int((rgb[k] * a + buf[i + k] * old * (1 - a)) / na) if na else 0
                buf[i + 3] = int(na * 255)
    return buf


def bezier(ctrl, n=40):
    def pt(t):
        pts = list(ctrl)
        while len(pts) > 1:
            pts = [((1 - t) * a[0] + t * b[0], (1 - t) * a[1] + t * b[1]) for a, b in zip(pts, pts[1:])]
        return pts[0]
    return [pt(i / n) for i in range(n + 1)]


GREY, BLUE, PINK = (110, 110, 110), (30, 110, 220), (220, 50, 150)
# corner at top-right; legs go left and down
CURVE = bezier([(0.45, 0.15), (0.65, 0.15), (0.85, 0.15), (0.85, 0.35), (0.85, 0.55)])


def curve_icon():
    return [
        ([(0.08, 0.15), (0.45, 0.15)], 0.07, GREY),
        ([(0.85, 0.55), (0.85, 0.92)], 0.07, GREY),
        ([(0.45, 0.15), (0.85, 0.15), (0.85, 0.55)], 0.025, (170, 170, 170)),
        (CURVE, 0.08, BLUE),
    ]


def comb_icon():
    c = bezier([(0.1, 0.8), (0.3, 0.1), (0.7, 0.1), (0.9, 0.8)], 12)
    strokes = []
    env = []
    for i in range(1, len(c) - 1):
        (x0, y0), (x1, y1) = c[i - 1], c[i + 1]
        tx, ty = x1 - x0, y1 - y0
        n = math.hypot(tx, ty)
        nx, ny = ty / n, -tx / n  # outward (up/away from the arch centre)
        k = 0.12 + 0.12 * math.sin(math.pi * i / (len(c) - 1))
        tip = (c[i][0] + nx * k, c[i][1] + ny * k)
        strokes.append(([c[i], tip], 0.03, PINK))
        env.append(tip)
    strokes.append((env, 0.04, PINK))
    strokes.append((c, 0.08, BLUE))
    return strokes


def fillet_icon():
    # sharp corner (light) with a G3 blend and a small arc marker
    return [
        ([(0.08, 0.15), (0.85, 0.15), (0.85, 0.92)], 0.03, (180, 180, 180)),
        ([(0.08, 0.15), (0.30, 0.15)], 0.07, GREY),
        ([(0.85, 0.70), (0.85, 0.92)], 0.07, GREY),
        (bezier([(0.30, 0.15), (0.50, 0.15), (0.70, 0.15), (0.85, 0.15), (0.85, 0.30), (0.85, 0.50), (0.85, 0.70)]),
         0.08, BLUE),
        (bezier([(0.18, 0.55), (0.18, 0.35), (0.38, 0.35)], 10), 0.05, (90, 160, 90)),
        ([(0.18, 0.55), (0.18, 0.80)], 0.05, (90, 160, 90)),
    ]


def smooth_icon():
    # lumpy curve (light) -> smooth curve (blue)
    lumpy = [(0.08, 0.75), (0.25, 0.45), (0.38, 0.62), (0.52, 0.30), (0.66, 0.48), (0.80, 0.22), (0.92, 0.30)]
    return [
        (lumpy, 0.04, (190, 190, 190)),
        (bezier([(0.08, 0.75), (0.35, 0.55), (0.60, 0.35), (0.92, 0.30)]), 0.08, BLUE),
    ]


def sketch_polygrid_icon():
    # outline with a 3 x 3 grid of square pockets
    strokes = [([(0.06, 0.06), (0.94, 0.06), (0.94, 0.94), (0.06, 0.94), (0.06, 0.06)], 0.05, GREY)]
    for i in range(3):
        for j in range(3):
            x, y = 0.17 + 0.235 * i, 0.17 + 0.235 * j
            s = 0.19
            strokes.append(([(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)], 0.035, BLUE))
    return strokes


def polygrid_icon():
    # a block (top face and two sides, in perspective) with hexagonal pockets cut into its top
    top = [(0.10, 0.40), (0.55, 0.18), (0.92, 0.38), (0.48, 0.62), (0.10, 0.40)]
    strokes = [(top, 0.045, GREY),
               ([(0.10, 0.40), (0.10, 0.62), (0.48, 0.86), (0.92, 0.60), (0.92, 0.38)], 0.045, GREY),
               ([(0.48, 0.62), (0.48, 0.86)], 0.045, GREY)]
    for cx, cy in ((0.33, 0.40), (0.52, 0.30), (0.52, 0.50), (0.71, 0.40)):
        hexa = [(cx + 0.075 * math.cos(math.radians(a)), cy + 0.045 * math.sin(math.radians(a))) for a in range(0, 361, 60)]
        strokes.append((hexa, 0.035, BLUE))
    return strokes


def screw_boss_icon():
    # a boss standing on a plate, with the hole in its top
    def ellipse(cx, cy, rx, ry, a0=0, a1=360):
        return [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a))) for a in range(a0, a1 + 1, 15)]
    return [([(0.04, 0.80), (0.30, 0.64), (0.96, 0.80), (0.70, 0.96), (0.04, 0.80)], 0.045, GREY),
            ([(0.28, 0.24), (0.28, 0.74)], 0.06, GREY),
            ([(0.72, 0.24), (0.72, 0.74)], 0.06, GREY),
            (ellipse(0.5, 0.74, 0.22, 0.08, 0, 180), 0.06, GREY),
            (ellipse(0.5, 0.24, 0.22, 0.08), 0.06, GREY),
            (ellipse(0.5, 0.24, 0.09, 0.035), 0.07, BLUE)]


def lip_icon():
    # a wall in section: the bottom half's lip (blue) rising into the top half's groove, with a gap round it
    light = (165, 165, 165)
    return [
        ([(0.25, 0.95), (0.25, 0.62), (0.47, 0.62)], 0.07, GREY),
        ([(0.47, 0.62), (0.50, 0.32), (0.62, 0.32), (0.65, 0.62)], 0.08, BLUE),
        ([(0.65, 0.62), (0.75, 0.62), (0.75, 0.95)], 0.07, GREY),
        ([(0.25, 0.05), (0.25, 0.52), (0.38, 0.52), (0.41, 0.18), (0.71, 0.18), (0.74, 0.52), (0.75, 0.52),
          (0.75, 0.05)], 0.05, light),
    ]


if __name__ == "__main__":
    for name, fn in (("gx_curve", curve_icon), ("gx_comb", comb_icon), ("gx_fillet", fillet_icon),
                     ("gx_smooth", smooth_icon), ("sketch_polygrid", sketch_polygrid_icon),
                     ("polygrid", polygrid_icon), ("screw_boss", screw_boss_icon), ("lip", lip_icon)):
        for s in (16, 32, 64):
            png(os.path.join(ROOT, name, "%dx%d.png" % (s, s)), s, s, render(s, fn()))
    print("icons written to", os.path.abspath(ROOT))
