"""YDLIDAR X2 as a binary STL in the URDF "laser" frame (metres, origin = laser centre).
YDLIDAR publishes no CAD for the X2, so the shape follows the datasheet dimension drawing:
96.0 mm long, rotor 60.5 mm, 50.3 mm tall, laser centre 7.9 mm under the top (same model as the team website).
Usage: py -I make_x2_stl.py out.stl
"""
import struct
import sys

import numpy as np

MM = 0.001
BOTTOM = -42.4  # mm, laser centre is 50.3 - 7.9 = 42.4 mm above the base
tris = []


def add_quad(a, b, c, d):
    tris.extend([(a, b, c), (a, c, d)])


def ring(cx, cy, r, z, seg):
    t = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    return [np.array([cx + r * np.cos(a), cy + r * np.sin(a), z]) for a in t]


def frustum(cx, cy, z0, z1, r0, r1, seg=40):
    lo, hi = ring(cx, cy, r0, z0, seg), ring(cx, cy, r1, z1, seg)
    c0, c1 = np.array([cx, cy, z0]), np.array([cx, cy, z1])
    for i in range(seg):
        j = (i + 1) % seg
        add_quad(lo[i], lo[j], hi[j], hi[i])
        tris.append((c0, lo[j], lo[i]))
        tris.append((c1, hi[i], hi[j]))


def prism(poly, z0, z1):
    """Extrude a convex CCW polygon (list of (x, y))."""
    lo = [np.array([x, y, z0]) for x, y in poly]
    hi = [np.array([x, y, z1]) for x, y in poly]
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        add_quad(lo[i], lo[j], hi[j], hi[i])
    for i in range(1, n - 1):
        tris.append((lo[0], lo[i + 1], lo[i]))
        tris.append((hi[0], hi[i], hi[i + 1]))


def x_cylinder(x0, x1, y, z, r, seg=24):
    """Short cylinder along +x (laser windows on the turret face)."""
    start = len(tris)
    frustum(0, 0, x0, x1, r, r, seg)
    for k in range(start, len(tris)):  # map (u, v, w) -> (w, u + y, v + z): axis z -> x
        tris[k] = tuple(np.array([p[2], p[0] + y, p[1] + z]) for p in tris[k])


def hull(circles):
    pts = []
    for cx, cy, r in circles:
        for a in np.linspace(0, 2 * np.pi, 48, endpoint=False):
            pts.append((cx + r * np.cos(a), cy + r * np.sin(a)))
    pts = sorted(set(pts))

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


# everything below in mm, z from the base (BOTTOM is applied at the end)
outline = hull([(0, 0, 31), (-35.3, 0, 4.5), (36, 18, 4.5), (36, -18, 4.5), (43.7, 0, 12.5)])
prism(outline, 15.1, 17.2)                                   # lower bracket
prism(outline, 27.4, 29.8)                                   # upper bracket / PCB
for x, y in ((-35.3, 0), (36, 18), (36, -18)):               # three legs through the mount holes
    frustum(x, y, 0, 29.8, 2.6, 2.6, 16)
frustum(43.7, 0, 2.5, 26.5, 8, 8, 28)                        # motor
frustum(43.7, 0, 29.8, 32.8, 5, 5, 20)                       # pulley
frustum(0, 0, 29.8, 33.5, 6, 6, 20)                          # rotor spindle
frustum(0, 0, 33.5, 50.3, 30.25, 29.6, 64)                   # turret
for dy, r in ((-18.8, 3.4), (13, 5.2)):                      # emitter + receiver windows, 7.9 mm under the top
    x = np.sqrt(30.3 ** 2 - dy ** 2)
    x_cylinder(x - 1.5, x + 0.8, dy, 42.4, r)

with open(sys.argv[1], "wb") as f:
    f.write(b"YDLIDAR X2 from datasheet drawing (laser frame, metres)".ljust(80, b" "))
    f.write(struct.pack("<I", len(tris)))
    for t in tris:
        a, b, c = (np.array([p[0], p[1], p[2] + BOTTOM]) * MM for p in t)
        n = np.cross(b - a, c - a)
        n = n / (np.linalg.norm(n) or 1)
        f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
print("triangles", len(tris))
