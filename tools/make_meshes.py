"""Meshes for parts that have no CAD file, sized from datasheets and the team's photos of the real robot.
Writes binary STL files (metres) into robot/meshes/:
  ydlidar_x2_base.stl / ydlidar_x2_top.stl   YDLIDAR X2 from its datasheet drawing (96.0 x 60.5 x 50.3 mm),
                                             origin = laser centre; the top (rotor cap) is the navy part
  deck_plate.stl                             one acrylic deck: 300 x 280 x 5 mm, 50 mm round corners (photo estimate)
  xtion.stl                                  ASUS Xtion PRO LIVE: 180 x 35 x 38 mm body with round ends on its stand,
                                             origin = centre of the body, +x = where the camera looks
Usage (from the project folder): pixi run python tools\\make_meshes.py      (Ubuntu: python3 tools/make_meshes.py)
"""
import os
import struct
import sys

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robot", "meshes")


class Mesh:
    def __init__(self):
        self.tris = []

    def quad(self, a, b, c, d):
        self.tris += [(a, b, c), (a, c, d)]

    def prism(self, poly, z0, z1, frame=None):
        """Extrude a convex CCW polygon [(u, v)] from z0 to z1; frame maps (u, v, w) -> xyz."""
        f = frame or (lambda p: np.array(p, dtype=float))
        lo = [f((u, v, z0)) for u, v in poly]
        hi = [f((u, v, z1)) for u, v in poly]
        n = len(poly)
        for i in range(n):
            j = (i + 1) % n
            self.quad(lo[i], lo[j], hi[j], hi[i])
        for i in range(1, n - 1):
            self.tris.append((lo[0], lo[i + 1], lo[i]))
            self.tris.append((hi[0], hi[i], hi[i + 1]))

    def cylinder(self, cx, cy, z0, z1, r0, r1=None, seg=40, frame=None):
        r1 = r0 if r1 is None else r1
        t = np.linspace(0, 2 * np.pi, seg, endpoint=False)
        if r0 == r1:
            self.prism([(cx + r0 * np.cos(a), cy + r0 * np.sin(a)) for a in t], z0, z1, frame)
            return
        f = frame or (lambda p: np.array(p, dtype=float))
        lo = [f((cx + r0 * np.cos(a), cy + r0 * np.sin(a), z0)) for a in t]
        hi = [f((cx + r1 * np.cos(a), cy + r1 * np.sin(a), z1)) for a in t]
        c0, c1 = f((cx, cy, z0)), f((cx, cy, z1))
        for i in range(seg):
            j = (i + 1) % seg
            self.quad(lo[i], lo[j], hi[j], hi[i])
            self.tris += [(c0, lo[j], lo[i]), (c1, hi[i], hi[j])]

    def save(self, name, scale=1.0, offset=(0, 0, 0)):
        path = os.path.join(OUT, name)
        with open(path, "wb") as fh:
            fh.write(name.encode().ljust(80, b" "))
            fh.write(struct.pack("<I", len(self.tris)))
            for t in self.tris:
                a, b, c = ((np.asarray(p, dtype=float) + offset) * scale for p in t)
                n = np.cross(b - a, c - a)
                n = n / (np.linalg.norm(n) or 1)
                fh.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
        pts = np.array([p for t in self.tris for p in t], dtype=float) * scale + np.array(offset) * scale
        print(f"{name:22s} {len(self.tris):5d} triangles, size {np.round((pts.max(0) - pts.min(0)) * 1000, 1)} mm")


def hull(circles, seg=48):
    pts = sorted({(cx + r * np.cos(a), cy + r * np.sin(a)) for cx, cy, r in circles
                  for a in np.linspace(0, 2 * np.pi, seg, endpoint=False)})

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) > 1 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) > 1 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def rounded_rect(w, h, r, seg=12):
    return hull([(sx * (w / 2 - r), sy * (h / 2 - r), r) for sx in (-1, 1) for sy in (-1, 1)], seg * 4)


def ydlidar_x2():
    """mm, z from the base; laser centre is 42.4 mm up (50.3 - 7.9)."""
    base, top = Mesh(), Mesh()
    outline = hull([(0, 0, 31), (-35.3, 0, 4.5), (36, 18, 4.5), (36, -18, 4.5), (43.7, 0, 12.5)])
    base.prism(outline, 15.1, 17.2)                       # lower bracket
    base.prism(outline, 27.4, 29.8)                       # upper bracket / PCB
    for x, y in ((-35.3, 0), (36, 18), (36, -18)):        # legs through the mount holes
        base.cylinder(x, y, 0, 29.8, 2.6, seg=16)
    base.cylinder(43.7, 0, 2.5, 26.5, 8, seg=28)          # motor
    base.cylinder(43.7, 0, 29.8, 32.8, 5, seg=20)         # pulley
    base.cylinder(0, 0, 29.8, 33.5, 6, seg=20)            # rotor spindle
    top.cylinder(0, 0, 33.5, 50.3, 30.25, 29.6, seg=64)   # rotor cap (navy on the real one)
    base.save("ydlidar_x2_base.stl", 0.001, (0, 0, -42.4))
    top.save("ydlidar_x2_top.stl", 0.001, (0, 0, -42.4))


def deck():
    m = Mesh()
    m.prism(rounded_rect(300, 280, 50), -2.5, 2.5)
    m.save("deck_plate.stl", 0.001)


def xtion():
    """Body: stadium 180 x 38 mm (y, z) extruded 35 mm along x, then the hinge neck and the oval foot."""
    m = Mesh()
    side = rounded_rect(180, 38, 18.9, seg=10)              # (u, v) = (y, z)
    m.prism(side, -17.5, 17.5, frame=lambda p: np.array([p[2], p[0], p[1]], dtype=float))
    m.prism(rounded_rect(30, 24, 4), -40, -19, frame=lambda p: np.array([p[0] * 0.6, p[1], p[2]], dtype=float))
    m.prism(rounded_rect(100, 50, 24.9, seg=10), -42.5, -38, frame=lambda p: np.array([p[1] * 0.9, p[0], p[2]], dtype=float))
    m.save("xtion.stl", 0.001)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    ydlidar_x2()
    deck()
    xtion()
    old = os.path.join(OUT, "ydlidar_x2.stl")
    if os.path.exists(old) and "--keep-old" not in sys.argv:
        os.remove(old)
        print("removed ydlidar_x2.stl (now split into base + top)")
