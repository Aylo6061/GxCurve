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


if __name__ == "__main__":
    for name, fn in (("gx_curve", curve_icon), ("gx_comb", comb_icon)):
        for s in (16, 32, 64):
            png(os.path.join(ROOT, name, "%dx%d.png" % (s, s)), s, s, render(s, fn()))
    print("icons written to", os.path.abspath(ROOT))
