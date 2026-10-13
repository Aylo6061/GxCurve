"""Tests for the polygrid geometry. Run: python -m unittest discover tests"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import polygrid as w  # noqa: E402


def rect(x0, y0, x1, y1):
    return [("line", (x0, y0), (x1, y0)), ("line", (x1, y0), (x1, y1)),
            ("line", (x1, y1), (x0, y1)), ("line", (x0, y1), (x0, y0))]


def circle(c, r):
    return [("arc", c, r, 0.0, 2 * math.pi)]


def rounded_rect(x0, y0, x1, y1, r):
    """Rectangle with corner fillets, given scrambled (as Fusion may hand profile curves over)."""
    P = math.pi
    prims = [
        ("line", (x0 + r, y0), (x1 - r, y0)), ("arc", (x1 - r, y0 + r), r, -P / 2, P / 2),
        ("line", (x1, y0 + r), (x1, y1 - r)), ("arc", (x1 - r, y1 - r), r, 0, P / 2),
        ("line", (x1 - r, y1), (x0 + r, y1)), ("arc", (x0 + r, y1 - r), r, P / 2, P / 2),
        ("line", (x0, y1 - r), (x0, y0 + r)), ("arc", (x0 + r, y0 + r), r, P, P / 2),
    ]
    prims = [w.prim_reverse(p) if i % 3 == 0 else p for i, p in enumerate(prims)]
    return prims[3:] + prims[:3]


def loops_area(loops):
    return sum(w.poly_area(p) for p, _ in loops)


def prim_pts(prims):
    out = []
    for p in prims:
        out.extend(w.prim_polyline(p, 64)[:-1])
    return out


def pocket_polys(res):
    return [[prim_pts(lp) for lp in pk["loops"]] for pk in res["pockets"]]


def inside_pockets(res, p):
    """Is p inside any pocket (inside its outer loop and outside its holes)?"""
    for loops in pocket_polys(res):
        if w.point_in_poly(p, loops[0]) and not any(w.point_in_poly(p, h) for h in loops[1:]):
            return True
    return False


def dist_to_area_boundary(area, p):
    loops = w.prepare_area(area)
    return min(w.seg_dist(p, a, b) for lp in loops for poly in [w.loop_polyline(lp, 1e-6)]
               for a, b in zip(poly, poly[1:] + poly[:1]))


class Prepare(unittest.TestCase):
    def test_orients_and_chains(self):
        loops = w.prepare_area([circle((5, 5), 1), rounded_rect(0, 0, 10, 8, 1)])
        self.assertEqual(len(loops), 2)
        self.assertGreater(w.poly_area(w.loop_polyline(loops[0])), 0)  # outer CCW
        self.assertLess(w.poly_area(w.loop_polyline(loops[1])), 0)     # hole CW
        lp = loops[0]
        for a, b in zip(lp, lp[1:] + lp[:1]):
            self.assertLess(w.dist(w.prim_end(a), w.prim_start(b)), 1e-9)

    def test_open_loop_rejected(self):
        with self.assertRaises(ValueError):
            w.prepare_area([[("line", (0, 0), (1, 0)), ("line", (1, 0), (1, 1))]])

    def test_interior_point(self):
        area = [rect(0, 0, 10, 4), circle((5, 2), 1.5)]
        loops = w.prepare_area(area)
        p = w.interior_point(loops)
        self.assertTrue(w.area_contains(loops, p))
        self.assertFalse(w.area_contains(loops, (5, 2)))


class Outline(unittest.TestCase):
    def test_chain_and_nest(self):
        # a scrambled pile: two separate squares, one with a round hole that has an island in it
        r1, r2 = rect(0, 0, 10, 10), rect(20, 0, 25, 5)
        pile = [r1[2], w.prim_reverse(r2[0]), circle((5, 5), 3)[0], r1[0], r2[2], w.prim_reverse(r1[3]),
                r2[1], circle((5, 5), 1)[0], r1[1], r2[3]]
        loops = w.chain_loops(pile)
        self.assertEqual(sorted(len(lp) for lp in loops), [1, 1, 4, 4])
        areas = w.areas_from_loops(loops)
        self.assertEqual(len(areas), 3)  # big square with its hole, the island, the small square
        sizes = sorted(sum(abs(w.poly_area(w.loop_polyline(lp))) * (1 if k == 0 else -1)
                           for k, lp in enumerate(w.prepare_area(a))) for a in areas)
        self.assertAlmostEqual(sizes[0], math.pi, delta=5e-3)  # chords of the circles
        self.assertAlmostEqual(sizes[1], 25.0, places=9)
        self.assertAlmostEqual(sizes[2], 100 - 9 * math.pi, delta=5e-3)

    def test_open_outline(self):
        with self.assertRaises(ValueError):
            w.chain_loops(rect(0, 0, 4, 4)[:3])

    def test_pocket_finder(self):
        res = w.build([[rect(0, 0, 10, 10), circle((5, 5), 0.3)]], w.SQUARE, 3.0, 0.4, 0.2, min_partial=0.0)
        inside = w.pocket_finder(res)
        self.assertTrue(inside((5.0, 6.0)))      # in the middle pocket
        self.assertFalse(inside((5.0, 5.0)))     # in its boss (the hole)
        self.assertFalse(inside((0.1, 5.0)))     # in the border
        self.assertFalse(inside((1.8, 5.0)))     # on a rib (x 1.6 to 2.0)
        in_area = w.area_finder(res)
        self.assertTrue(in_area((1.8, 5.0)) and in_area((0.1, 5.0)))
        self.assertFalse(in_area((5.0, 5.0)) or in_area((10.5, 5.0)))  # in the hole; outside


class Erode(unittest.TestCase):
    def test_rectangle(self):
        E = w.erode(w.prepare_area([rect(0, 0, 10, 6)]), 1.0)
        self.assertEqual(len(E), 1)
        self.assertAlmostEqual(loops_area(E), 8 * 4, places=9)
        x0, y0, x1, y1 = w.bbox(E[0][0])
        for a, b in zip((x0, y0, x1, y1), (1, 1, 9, 5)):
            self.assertAlmostEqual(a, b, places=12)

    def test_hole_grows(self):
        E = w.erode(w.prepare_area([rect(0, 0, 10, 10), circle((5, 5), 1)]), 0.5)
        self.assertEqual(len(E), 2)
        hole = [lp for lp in E if w.poly_area(lp[0]) < 0][0]
        for p in hole[0]:
            self.assertAlmostEqual(w.dist(p, (5, 5)), 1.5, places=9)
        self.assertTrue(all(t[0] == "A" and abs(t[3] - 1.5) < 1e-12 for t in hole[1]))

    def test_reflex_corner_rounds(self):
        L = [("line", (0, 0), (6, 0)), ("line", (6, 0), (6, 2)), ("line", (6, 2), (2, 2)),
             ("line", (2, 2), (2, 6)), ("line", (2, 6), (0, 6)), ("line", (0, 6), (0, 0))]
        E = w.erode(w.prepare_area([L]), 0.5)
        self.assertEqual(len(E), 1)
        arcs = [t for t in E[0][1] if t[0] == "A"]
        self.assertTrue(arcs)
        self.assertTrue(all(w.dist(t[2], (2, 2)) < 1e-12 and abs(t[3] - 0.5) < 1e-12 for t in arcs))
        # every point of the shrunk outline is exactly 0.5 from the original boundary
        for p in E[0][0]:
            self.assertAlmostEqual(dist_to_area_boundary([L], p), 0.5, delta=2e-4)

    def test_neck_splits(self):
        # two 4x4 squares joined by a 0.6-wide neck: a 0.5 border closes the neck
        poly = [(0, 0), (4, 0), (4, 1.7), (6, 1.7), (6, 0), (10, 0), (10, 4), (6, 4), (6, 2.3), (4, 2.3), (4, 4), (0, 4)]
        area = [("line", a, b) for a, b in zip(poly, poly[1:] + poly[:1])]
        E = w.erode(w.prepare_area([area]), 0.5)
        self.assertEqual(len(E), 2)
        # each 3x3 square has a small bump toward the neck, between radius-0.5 arcs round the neck's corners
        bump = 2 * (0.15 - (0.3 / 2 * 0.4 + 0.25 / 2 * math.asin(0.6)))
        self.assertAlmostEqual(loops_area(E), 2 * (3 * 3 + bump), delta=2e-4)  # chords of the arcs

    def test_too_thick_removes_everything(self):
        self.assertEqual(w.erode(w.prepare_area([rect(0, 0, 2, 2)]), 1.2), [])

    def test_rounded_rect_and_scale(self):
        area = rounded_rect(-3, -2, 5, 4, 1.5)
        E = w.erode(w.prepare_area([area]), 0.4)
        self.assertEqual(len(E), 1)
        # rounded rect shrunk: straight sides 0.4 in, corner radius 1.1
        expect = (7.2 * 5.2) - (4 - math.pi) * 1.1 ** 2
        self.assertAlmostEqual(loops_area(E), expect, delta=1e-3)
        radii = sorted({round(t[3], 9) for t in E[0][1] if t[0] == "A"})
        self.assertEqual(radii, [1.1])

    def test_spline_boundary(self):
        # an ellipse given as a dense polyline ("curve")
        pts = [(4 * math.cos(t), 2.5 * math.sin(t)) for t in [2 * math.pi * i / 400 for i in range(401)]]
        E = w.erode(w.prepare_area([[("curve", pts)]]), 0.3)
        self.assertEqual(len(E), 1)
        for p in E[0][0][::25]:
            d = min(w.seg_dist(p, a, b) for a, b in zip(pts, pts[1:]))
            self.assertAlmostEqual(d, 0.3, delta=1e-6)


class Lattice(unittest.TestCase):
    def check_gaps(self, cells, rib):
        """Facing edges of neighbouring pockets are parallel and exactly `rib` apart."""
        edges = []
        for c in cells:
            for a, b in zip(c, c[1:] + c[:1]):
                edges.append((a, b))
        found = 0
        for i, (a, b) in enumerate(edges):
            d = w.unit(w.sub(b, a))
            for c, e in edges[i + 1:]:
                if abs(w.cross(d, w.unit(w.sub(e, c)))) > 1e-9:
                    continue
                sep = abs(w.cross(d, w.sub(c, a)))
                # overlapping along the edge direction?
                t0, t1 = sorted((w.dot(w.sub(c, a), d), w.dot(w.sub(e, a), d)))
                if t1 <= 1e-9 or t0 >= w.dist(a, b) - 1e-9 or sep > 2 * rib:
                    continue
                self.assertAlmostEqual(sep, rib, places=9)
                found += 1
        self.assertGreater(found, 0)

    def test_rib_gaps(self):
        b = (-3, -3, 3, 3)
        for shape, dims in ((w.SQUARE, 0.8), (w.HEXAGON, 0.7), (w.TRIANGLE, 0.5), (w.RECTANGLE, (2.0, 1.0))):
            for rot in (False, True):
                cells = w.lattice(shape, dims, 0.25, b, rot)
                self.check_gaps(cells[:60], 0.25)
                for c in cells:
                    self.assertGreater(w.poly_area(c), 0)

    def test_pocket_size(self):
        for shape in (w.TRIANGLE, w.SQUARE, w.HEXAGON):
            r = w.pocket_inradius(shape, w.SIDE, 1.0)
            cell = w.lattice(shape, r, 0.1, (-1, -1, 1, 1))[0]
            self.assertAlmostEqual(w.dist(cell[0], cell[1]), 1.0, places=12)
            self.assertAlmostEqual(w.pocket_side(shape, r), 1.0, places=12)
            r2 = w.pocket_inradius(shape, w.INSCRIBED, 1.0)
            self.assertAlmostEqual(r2, 0.5)

    def test_symmetry(self):
        b = (-4, -4, 4, 4)
        for shape, dims in ((w.SQUARE, 0.6), (w.HEXAGON, 0.6), (w.TRIANGLE, 0.45), (w.RECTANGLE, (1.4, 0.7))):
            for rot in (False, True):
                vs, hs = w.allowed_phases(shape, rot)
                for vp in vs:
                    for hp in hs:
                        cells = w.lattice(shape, dims, 0.2, b, rot, vp, hp)
                        cents = {(round(sum(p[0] for p in c) / len(c), 9) + 0.0, round(sum(p[1] for p in c) / len(c), 9) + 0.0)
                                 for c in cells}
                        inner = {c for c in cents if abs(c[0]) < 3 and abs(c[1]) < 3}
                        for x, y in inner:
                            self.assertIn((round(-x, 9) + 0.0, y), cents, (shape, rot, vp, hp))
                            self.assertIn((x, round(-y, 9) + 0.0), cents, (shape, rot, vp, hp))
                        on_v = any(abs(x) < 1e-9 for x, _ in inner)
                        on_h = any(abs(y) < 1e-9 for _, y in inner)
                        if shape in (w.SQUARE, w.RECTANGLE):
                            self.assertEqual(on_v, vp == w.CELL)
                            self.assertEqual(on_h, hp == w.CELL)

    def test_grid_frame(self):
        o, a = w.grid_frame((3, 3), vline=((1, 0), (1, 5)), hline=((0, 2), (8, 2)))
        self.assertEqual((o, a), ((1.0, 2.0), 0.0))
        o, a = w.grid_frame((3, 3), vline=((1, 5), (1, 0)))
        self.assertEqual(o, (1.0, 3.0))
        self.assertAlmostEqual(a, 0.0)
        o, a = w.grid_frame((0, 0), hline=((0, 0), (1, 1)))
        self.assertAlmostEqual(a, math.pi / 4)
        with self.assertRaises(ValueError):
            w.grid_frame((0, 0), vline=((0, 0), (0.1, 1)), hline=((0, 0), (1, 0)))


class Build(unittest.TestCase):
    def test_rectangle_square_cells_exact(self):
        # 10 x 10, border 0.5, square pockets 2 with rib 0.5 centred on cells: 9 x 9 interior lattice
        res = w.build([[rect(0, 0, 10, 10)]], w.SQUARE, 1.0, 0.5, 0.5, min_partial=0.0)
        # lattice pitch 2.5 centred at 5: centres at 0, 2.5, 5, 7.5, 10 -> pockets 4..6 etc
        self.assertEqual(res["full"], 9)
        # the ring of pockets around the edge is cut by the border at 0.5 / 9.5
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                for p in prim_pts(lp):
                    self.assertGreaterEqual(min(p[0], p[1]), 0.5 - 1e-9)
                    self.assertLessEqual(max(p[0], p[1]), 9.5 + 1e-9)
        # total open area = inner 9x9 square minus the ribs inside it
        expect = 0
        for cx in (0, 2.5, 5, 7.5, 10):
            for cy in (0, 2.5, 5, 7.5, 10):
                ox = max(0, min(cx + 1, 9.5) - max(cx - 1, 0.5))
                oy = max(0, min(cy + 1, 9.5) - max(cy - 1, 0.5))
                expect += ox * oy
        self.assertAlmostEqual(res["pocket_area"], expect, places=6)

    def test_drop_slivers(self):
        res = w.build([[rect(0, 0, 10, 10)]], w.SQUARE, 1.0, 0.5, 0.5, min_partial=0.2)
        # edge pockets are 0.5 wide (0.25 of a full one) -> kept; corner ones 0.0625 -> dropped
        self.assertEqual(res["dropped"], 4)
        self.assertEqual(res["partial"], 12)

    def test_area_matches_monte_carlo(self):
        area = [rounded_rect(0, 0, 9, 7, 2.0), circle((3, 3.5), 1.2), rect(5.5, 2, 7.5, 5)]
        res = w.build([area], w.HEXAGON, 0.45, 0.15, 0.3, min_partial=0.0)
        rnd = random.Random(1)
        loops = w.prepare_area(area)
        shrunk = w.erode(loops, 0.3)
        cells = res["pockets"]
        self.assertGreater(res["full"], 10)
        self.assertGreater(res["partial"], 10)
        # every pocket point is inside the shrunk area
        for pk in cells:
            for lp in pk["loops"]:
                for p in prim_pts(lp)[::3]:
                    self.assertGreaterEqual(dist_to_area_boundary(area, p), 0.3 - 2e-4)
        # pocket area vs. sampled fraction (pocket = in a lattice pocket and in the shrunk area)
        hits, n = 0, 4000
        for _ in range(n):
            p = (rnd.uniform(0, 9), rnd.uniform(0, 7))
            hits += inside_pockets(res, p)
        self.assertAlmostEqual(hits / n * 63, res["pocket_area"], delta=1.0)
        self.assertAlmostEqual(res["pocket_area"], sum(pk["area"] for pk in cells), places=9)
        self.assertLess(res["pocket_area"], loops_area(shrunk))

    def test_hole_inside_pocket(self):
        # a small hole sits inside one big pocket: the pocket gets a hole loop (boss) around it
        res = w.build([[rect(0, 0, 10, 10), circle((5, 5), 0.3)]], w.SQUARE, 3.0, 0.4, 0.2, min_partial=0.0)
        boss = [pk for pk in res["pockets"] if len(pk["loops"]) == 2]
        self.assertEqual(len(boss), 1)
        self.assertEqual(boss[0]["loops"][1], [("circle", (5, 5), 0.5)])
        self.assertAlmostEqual(boss[0]["area"], 36 - math.pi * 0.25, delta=2e-3)

    def test_arc_corners_exact(self):
        # pocket corners cut by a curved border are exactly on the shrunk circle
        res = w.build([[circle((0, 0), 5)]], w.SQUARE, 0.5, 0.2, 0.5, min_partial=0.0)
        n_arcs = 0
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                for p in lp:
                    if p[0] == "arc3":
                        n_arcs += 1
                        for q in (p[1], p[2], p[3]):
                            self.assertAlmostEqual(w.dist(q, (0, 0)), 4.5, places=9)
                for a, b in zip(lp, lp[1:] + lp[:1]):  # closed, head to tail
                    end = a[3] if a[0] == "arc3" else a[2] if a[0] == "seg" else None
                    start = b[1]
                    if end is not None and b[0] in ("seg", "arc3"):
                        self.assertLess(w.dist(end, start), 1e-12)
        self.assertGreater(n_arcs, 10)

    def test_symmetric_result(self):
        area = [rounded_rect(-6, -4, 6, 4, 1.0), circle((-3, 0), 0.8), circle((3, 0), 0.8)]
        for shape, dims in ((w.HEXAGON, 0.5), (w.TRIANGLE, 0.35), (w.SQUARE, 0.6)):
            vs, hs = w.allowed_phases(shape)
            res = w.build([area], shape, dims, 0.2, 0.3, origin=(0, 0), vphase=vs[-1], hphase=hs[-1], min_partial=0.15)
            cents = sorted((round(sum(p[0] for p in pts) / len(pts), 6), round(sum(p[1] for p in pts) / len(pts), 6))
                           for pts in (prim_pts(pk["loops"][0]) for pk in res["pockets"]))
            mirrored = sorted((round(-x, 6) + 0.0, y) for x, y in cents)
            self.assertEqual([(x + 0.0, y) for x, y in cents], mirrored, shape)

    def test_tilted_grid(self):
        area = [rect(0, 0, 6, 6)]
        res = w.build([area], w.SQUARE, 0.5, 0.2, 0.2, hline=((0, 0), (6, 2)))
        d = w.unit((6, 2))
        for pk in res["pockets"]:
            if pk["full"]:
                lp = pk["loops"][0]
                e = w.unit(w.sub(lp[0][2], lp[0][1]))
                self.assertAlmostEqual(abs(w.cross(e, d)) * abs(w.dot(e, d)), 0.0, places=9)

    def test_no_border(self):
        res = w.build([[circle((0, 0), 3)]], w.TRIANGLE, 0.3, 0.1, 0.0, min_partial=0.0)
        self.assertAlmostEqual(res["pocket_area"] / (math.pi * 9), res["cell_area"] / w.lattice_cell_area(w.TRIANGLE, 0.3, 0.1), delta=0.03)
        on_edge = [p for pk in res["pockets"] for lp in pk["loops"] for p in lp if p[0] == "arc3"]
        self.assertTrue(on_edge)
        self.assertTrue(all(abs(w.dist(p[2], (0, 0)) - 3) < 1e-9 for p in on_edge))

    def test_seeds_and_lines(self):
        a1, a2 = [rect(0, 0, 4, 4), circle((2, 2), 1)], [rect(5, 0, 9, 4), circle((7, 2), 1)]
        res = w.build([a1, a2], w.SQUARE, 0.4, 0.1, 0.2, vline=((4.5, -1), (4.5, 5)))
        self.assertEqual(len(res["seeds"]), 2)
        self.assertTrue(w.area_contains(w.prepare_area(a1), res["seeds"][0]))
        self.assertTrue(w.area_contains(w.prepare_area(a2), res["seeds"][1]))
        cents = sorted((round(sum(p[0] for p in q) / len(q), 6), round(sum(p[1] for p in q) / len(q), 6))
                       for q in (prim_pts(pk["loops"][0]) for pk in res["pockets"]))
        self.assertEqual(cents, sorted((round(9 - x, 6), y) for x, y in cents))  # mirrored about x = 4.5

    def test_too_many_cells(self):
        with self.assertRaises(ValueError):
            w.build([[rect(0, 0, 100, 100)]], w.SQUARE, 0.05, 0.02, 0.1)

    def test_bad_values(self):
        for kw in (dict(rib=0), dict(border=-1)):
            args = dict(shape=w.SQUARE, dims=1.0, rib=0.2, border=0.2)
            args.update(kw)
            with self.assertRaises(ValueError):
                w.build([[rect(0, 0, 5, 5)]], **args)


def ends(p):
    """(start, end) of a drawing primitive."""
    if p[0] == "seg":
        return p[1], p[2]
    if p[0] == "arc3":
        return p[1], p[3]
    return p[1][0], p[1][-1]


def tangents(p):
    """Unit tangents at the start and end of a drawing primitive (direction of travel)."""
    if p[0] == "seg":
        d = w.unit(w.sub(p[2], p[1]))
        return d, d
    if p[0] == "arc3":
        c, sw = p[4], p[5]
        rot = (lambda v: (-v[1], v[0])) if sw > 0 else (lambda v: (v[1], -v[0]))
        return w.unit(rot(w.sub(p[1], c))), w.unit(rot(w.sub(p[3], c)))
    s = w.g.Spline(p[1], p[2])
    return w.unit(s.d(0.0, 1)), w.unit(s.d(1.0, 1))


def joints(lp):
    """(previous primitive, next primitive) at each joint of a closed loop of primitives."""
    return [(lp[i - 1], lp[i]) for i in range(len(lp))]


class Corners(unittest.TestCase):
    AREA = [rect(0, 0, 10, 7)]

    def check_closed_and_smooth(self, res, smooth=True):
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                if len(lp) == 1:
                    continue
                for p, q in joints(lp):
                    self.assertLess(w.dist(ends(p)[1], ends(q)[0]), 1e-9)
                    if smooth:  # no corners left (every corner of these pockets points outwards)
                        self.assertAlmostEqual(w.cross(tangents(p)[1], tangents(q)[0]), 0.0, delta=1e-6)

    def test_arc_full_pockets(self):
        r = 0.2
        res = w.build([self.AREA], w.SQUARE, 0.6, 0.2, 0.3, corner=r, corner_style="arc")
        full = [pk for pk in res["pockets"] if pk["full"]]
        self.assertTrue(full)
        for pk in full:
            lp = pk["loops"][0]
            arcs = [p for p in lp if p[0] == "arc3"]
            self.assertEqual(len(arcs), 4)
            for p in arcs:
                self.assertAlmostEqual(w.dist(p[1], p[4]), r, places=12)
        self.assertAlmostEqual(res["cell_area"], 1.2 ** 2 - (4 - math.pi) * r * r, delta=1e-5)
        self.check_closed_and_smooth(res)

    def test_arc_cut_pockets(self):
        res = w.build([self.AREA], w.SQUARE, 0.6, 0.2, 0.3, corner=0.2, min_partial=0.0)
        self.assertGreater(res["partial"], 10)
        self.check_closed_and_smooth(res)
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                for p in lp:
                    if p[0] == "arc3":
                        self.assertAlmostEqual(w.dist(p[1], p[4]), 0.2, places=9)
                    for q in w.prim_polyline(p):  # still inside the border
                        self.assertTrue(0.3 - 1e-9 <= q[0] <= 9.7 + 1e-9 and 0.3 - 1e-9 <= q[1] <= 6.7 + 1e-9)

    def test_arc_on_curved_border(self):
        # pockets cut by a circular border: their corners there are radius-r arcs tangent to it
        res = w.build([[circle((0, 0), 5)]], w.SQUARE, 0.5, 0.2, 0.5, corner=0.15, min_partial=0.0)
        tangent_to_border = 0
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                for p in lp:
                    if p[0] == "arc3" and abs(w.dist(p[4], (0, 0)) - (4.5 - 0.15)) < 1e-6:
                        tangent_to_border += 1
        self.assertGreater(tangent_to_border, 10)
        self.check_closed_and_smooth(res)

    def test_inward_corner_stays_sharp(self):
        # no border: the L's inside corner is a corner of a pocket, pointing into it; it stays sharp
        L = [("line", a, b) for a, b in zip([(0, 0), (6, 0), (6, 2.5), (2.5, 2.5), (2.5, 6), (0, 6)],
                                            [(6, 0), (6, 2.5), (2.5, 2.5), (2.5, 6), (0, 6), (0, 0)])]
        res = w.build([[L]], w.SQUARE, 1.0, 0.2, 0.0, origin=(2.0, 2.0), corner=0.2, min_partial=0.0)
        hits = [pk for pk in res["pockets"] for lp in pk["loops"] for p in lp
                if p[0] == "seg" and w.dist(p[2], (2.5, 2.5)) < 1e-9]
        self.assertEqual(len(hits), 1)
        self.check_closed_and_smooth(res, smooth=False)

    def test_g3_g4_full_pockets(self):
        for style, count, G in (("g3", 7, 3), ("g4", 9, 4)):
            for shape, dims in ((w.SQUARE, 0.6), (w.HEXAGON, 0.5), (w.TRIANGLE, 0.4), (w.RECTANGLE, (1.4, 0.8))):
                res = w.build([self.AREA], shape, dims, 0.2, 0.3, corner=0.12, corner_style=style)
                full = [pk for pk in res["pockets"] if pk["full"]]
                self.assertTrue(full, (style, shape))
                for p in full[0]["loops"][0]:
                    if p[0] == "cspline":
                        self.assertEqual(len(p[1]), count)
                        self.assertEqual(w.g.end_continuity_to_line(p[1], 5), G)
                        self.assertEqual(w.g.end_continuity_to_line(p[1], 5, from_end=True), G)
                self.check_closed_and_smooth(res)
                self.assertEqual(res["arc_corners"], 0)

    def test_g3_on_curved_border(self):
        # where a cut pocket meets the round border, its G3 corner matches the border's curvature
        Re = 4.5
        res = w.build([[circle((0, 0), 5)]], w.SQUARE, 0.5, 0.2, 0.5, corner=0.12, corner_style="g3", min_partial=0.0)
        self.check_closed_and_smooth(res)
        checked = 0
        for pk in res["pockets"]:
            for lp in pk["loops"]:
                for p in lp:
                    if p[0] != "cspline":
                        continue
                    s = w.g.Spline(p[1], p[2])
                    for t, q in ((0.0, p[1][0]), (1.0, p[1][-1])):
                        if abs(w.dist(q, (0, 0)) - Re) < 1e-9:
                            self.assertAlmostEqual(s.curvature(t), 1.0 / Re, delta=1e-6)
                            self.assertAlmostEqual(s.curvature_rate(t), 0.0, delta=1e-5)
                            checked += 1
                        else:  # the other end is on a straight pocket edge
                            self.assertAlmostEqual(s.curvature(t), 0.0, delta=1e-6)
                            self.assertAlmostEqual(s.curvature_rate(t), 0.0, delta=1e-5)
        self.assertGreater(checked, 10)

    def test_g3_tight_corners_fall_back(self):
        # thin cut pockets: some G3 corners can't fit and shrink or stay arcs; loops stay closed
        res = w.build([[rounded_rect(0, 0, 10, 7, 1.5)]], w.RECTANGLE, (1.6, 0.8), 0.2, 0.25, corner=0.2,
                      corner_style="g3", hline=((0, 1), (10, 3)), min_partial=0.0)
        self.assertGreater(res["arc_corners"] + res["smaller_corners"], 0)
        self.check_closed_and_smooth(res)

    def test_radius_too_big(self):
        with self.assertRaises(ValueError):
            w.build([self.AREA], w.SQUARE, 0.5, 0.2, 0.3, corner=0.51)
        w.build([self.AREA], w.SQUARE, 0.5, 0.2, 0.3, corner=0.5)  # round pockets: allowed
        lim = w.max_corner(w.SQUARE, 0.5, "g3")
        self.assertAlmostEqual(lim, 0.5 / 1.452488, places=5)
        with self.assertRaises(ValueError):
            w.build([self.AREA], w.SQUARE, 0.5, 0.2, 0.3, corner=lim * 1.01, corner_style="g3")

    def test_corner_table_matches_fillet_tool(self):
        # the table reproduces GX G3 Fillet's layout at a tabulated angle (computing it takes ~1 s)
        L, fr = w.corner_shape(math.pi / 2, 7)
        lay = w.g.fillet_layout(math.pi / 2, 1.0, 7)
        self.assertAlmostEqual(L, lay["L"], delta=1e-5)
        for a, b in zip(fr, lay["fractions"]):
            self.assertAlmostEqual(a, b, delta=1e-5)

    def test_ribs_edges_skip_the_outline(self):
        # with no border, pocket stretches on the area's outline aren't in "edges"
        res = w.build([self.AREA], w.SQUARE, 0.6, 0.2, 0.0, origin=(0.3, 0.3), corner=0.1, min_partial=0.0)
        cut = [pk for pk in res["pockets"] if not pk["full"]]
        self.assertTrue(cut)
        for pk in cut:
            n_all = sum(len(lp) for lp in pk["loops"])
            n_own = sum(len(lp) for lp in pk["edges"])
            self.assertLess(n_own, n_all)


class Bottom(unittest.TestCase):
    def test_profile_is_g3_to_wall_and_floor(self):
        for style, G in (("g3", 3), ("g4", 4)):
            ctrl = w.bottom_profile(0.2, style)
            L = w.bottom_reach(0.2, style)
            self.assertAlmostEqual(ctrl[0][0], L)       # floor contact on +x
            self.assertAlmostEqual(ctrl[-1][1], L)      # wall contact on +y
            self.assertEqual(w.g.end_continuity_to_line(ctrl, 5), G)
            self.assertEqual(w.g.end_continuity_to_line(ctrl, 5, from_end=True), G)
        self.assertAlmostEqual(w.bottom_reach(0.2, "g3"), 0.2 * 1.452488, places=5)
        self.assertEqual(w.bottom_reach(0.2, "radius"), 0.2)

    def test_blend_loops(self):
        res = w.build([[rect(0, 0, 10, 10), circle((5, 5), 0.3)]], w.SQUARE, 3.0, 0.4, 0.2, min_partial=0.0)
        n_loops = sum(len(pk["loops"]) for pk in res["pockets"])
        cut = w.blend_loops(res, "cut_holes")
        self.assertEqual(len(cut), n_loops)  # every outline, the boss's too
        self.assertTrue(all(side == 1 for _, _, _, side in cut))
        ext = w.blend_loops(res, "extrude_ribs")
        self.assertEqual(len(ext), len(res["pockets"]))  # outer outlines only
        self.assertTrue(all(side == -1 for _, _, _, side in w.blend_loops(res, "cut_ribs")))

    def test_shape_groups(self):
        # square pockets in a rectangle: all whole pockets are one shape; the cut ones along each
        # straight edge are another per edge (and per corner)
        res = w.build([[rect(0, 0, 10, 7)]], w.SQUARE, 0.6, 0.2, 0.3, origin=(0.3, 0.3), min_partial=0.0)
        loops = [pk["loops"][0] for pk in res["pockets"]]
        groups = w.shape_groups(loops)
        sizes = sorted(len(m) for _, m in groups)
        self.assertEqual(sum(sizes), len(loops))
        self.assertEqual(sizes[-1], res["full"])
        self.assertLess(len(groups), 10)
        for seed, members in groups:
            for k, (dx, dy) in members:  # each member is its seed moved by (dx, dy)
                a = [q for p in loops[seed] for q in w._key_points(p)]
                b = [q for p in loops[k] for q in w._key_points(p)]
                self.assertAlmostEqual(min(q[0] for q in b) - min(q[0] for q in a), dx, places=9)
                self.assertAlmostEqual(min(q[1] for q in b) - min(q[1] for q in a), dy, places=9)
        # mirror images aren't the same shape
        tri = [("seg", (0, 0), (1, 0), True), ("seg", (1, 0), (0, 1), True), ("seg", (0, 1), (0, 0), True)]
        mir = [("seg", (0, 0), (1, 0), True), ("seg", (1, 0), (1, 1), True), ("seg", (1, 1), (0, 0), True)]
        self.assertEqual(len(w.shape_groups([tri, mir])), 2)

    def test_loop_distance(self):
        sq = [("seg", (0, 0), (2, 0), True), ("seg", (2, 0), (2, 2), True), ("seg", (2, 2), (0, 2), True),
              ("seg", (0, 2), (0, 0), True)]
        self.assertAlmostEqual(w.loop_distance(sq, (1, 0.5)), 0.5)
        self.assertAlmostEqual(w.loop_distance(sq, (3, 1)), 1.0)


if __name__ == "__main__":
    unittest.main()
