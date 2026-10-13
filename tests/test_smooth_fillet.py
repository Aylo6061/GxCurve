"""Tests for fairing (GX Smooth) and the G3 fillet layout. Run: python -m unittest discover tests"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import gx_math as g  # noqa: E402


def corner(count, fr, apex=g.APEX_CORNER):
    m, apex_mode, n, p = g.corner_style(count, "cubic", apex)
    uA, uB = (1.0, 0.0), g.normalize((0.3, 1.0))
    return g.corner_control_points((0.0, 0.0), uA, uB, [5 * f for f in fr], [4 * f for f in fr], apex_mode, 0.4), p


class Gram(unittest.TestCase):
    def test_matches_numeric_energy(self):
        ctrl = [(0, 0), (1, 2), (3, 3), (4, 1), (6, 2), (7, 0), (9, 1)]
        s = g.Spline(ctrl, 5)
        for order in (1, 2, 3):
            G = g.gram_matrix(len(ctrl), 5, None, order)
            e_quad = sum(G[i][j] * g.dot(ctrl[i], ctrl[j]) for i in range(7) for j in range(7))
            N = 20000
            e_num = sum(g.dot(s.d((k + 0.5) / N, order), s.d((k + 0.5) / N, order)) for k in range(N)) / N
            self.assertAlmostEqual(e_quad / e_num, 1.0, places=4, msg=order)

    def test_solve(self):
        A = [[4.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]]
        x = g.solve(A, [1.0, 2.0, 3.0])
        for row, b in zip(A, [1.0, 2.0, 3.0]):
            self.assertAlmostEqual(sum(a * xi for a, xi in zip(row, x)), b)


class Smooth(unittest.TestCase):
    def test_corner_keeps_continuity_and_ends(self):
        for count, G in ((7, 3), (9, 4), (5, 2)):
            ctrl, p = corner(count, [1.0, 0.95, 0.3, 0.25][: (count - 1) // 2])  # deliberately lumpy
            for order in (g.ENERGY_VELOCITY, g.ENERGY_ACCEL, g.ENERGY_JERK):
                new, warn = g.fair_spline(ctrl, p, order=order, amount=1.0)
                with self.subTest(count=count, order=order):
                    self.assertEqual(new[0], ctrl[0])
                    self.assertEqual(new[-1], ctrl[-1])
                    self.assertEqual(new[(count - 1) // 2], ctrl[(count - 1) // 2])  # corner apex fixed
                    if not warn:
                        self.assertEqual(g.end_continuity_to_line(new, p), G)
                        self.assertEqual(g.end_continuity_to_line(new, p, from_end=True), G)

    def test_energy_goes_down(self):
        ctrl, p = corner(9, [1.0, 0.95, 0.3, 0.25])
        for order in (g.ENERGY_VELOCITY, g.ENERGY_ACCEL, g.ENERGY_JERK):
            G = g.gram_matrix(len(ctrl), p, None, order)
            E = lambda c: sum(G[i][j] * g.dot(c[i], c[j]) for i in range(len(c)) for j in range(len(c)))
            new, _ = g.fair_spline(ctrl, p, order=order, amount=1.0)
            self.assertLess(E(new), E(ctrl), order)

    def test_bending_then_curvature_improve_fairness(self):
        # note: parametric jerk (ENERGY_JERK) is NOT used for smoothing -- it measures change per
        # parameter step, not per arc length, and makes the comb worse on corner curves.
        ctrl, p = corner(9, [1.0, 0.95, 0.3, 0.25])
        f0 = g.fairness(g.Spline(ctrl, p))
        bend, warn = g.fair_spline(ctrl, p, amount=1.0)
        self.assertEqual(warn, "")
        f1 = g.fairness(g.Spline(bend, p))
        self.assertLess(f1, 0.5 * f0)
        curv, warn = g.fair_spline(ctrl, p, amount=1.0, method=g.METHOD_CURVATURE)
        self.assertEqual(warn, "")
        s = g.Spline(curv, p)
        self.assertLess(g.fairness(s), f1)
        self.assertLess(g.max_curvature(s, 4000), 2.0)  # no hidden cusp
        self.assertEqual(g.end_continuity_to_line(curv, p), 4)

    def test_tension_shortens(self):
        ctrl, p = corner(7, [1.0, 0.7, 0.3])
        lens = []
        for tension in (0.0, 0.1):
            new, _ = g.fair_spline(ctrl, p, amount=1.0, tension=tension)
            s = g.Spline(new, p)
            lens.append(sum(g.length(g.sub(s.point((i + 1) / 400), s.point(i / 400))) for i in range(400)))
        self.assertLess(lens[1], lens[0])

    def test_amount_zero_is_identity(self):
        random.seed(1)
        ctrl = [(i, random.uniform(-1, 1)) for i in range(7)]
        new, _ = g.fair_spline(ctrl, 5, amount=0.0, keep_ends=False)
        for a, b in zip(ctrl, new):
            self.assertAlmostEqual(a[0], b[0], places=2)
            self.assertAlmostEqual(a[1], b[1], places=2)

    def test_symmetric_stays_symmetric(self):
        m, apex_mode, n, p = g.corner_style(7)
        uA, uB = (1.0, 0.0), (0.0, 1.0)
        ctrl = g.corner_control_points((0.0, 0.0), uA, uB, [4, 3.5, 0.5], [4, 3.5, 0.5], apex_mode)
        new, _ = g.fair_spline(ctrl, p, amount=0.8)
        for i in range(n):
            a, b = new[i], new[n - 1 - i]
            self.assertAlmostEqual(a[0], b[1], places=9)
            self.assertAlmostEqual(a[1], b[0], places=9)

    def test_free_curve_with_knots(self):
        ctrl = [(0, 0), (1, 1), (2, -1), (3, 2), (4, -2), (5, 1), (6, 0)]
        knots = [0] * 6 + [0.3] + [1] * 6  # non-uniform
        new, _ = g.fair_spline(ctrl, 5, knots=knots, amount=0.7, keep_ends=False)
        self.assertLess(g.fairness(g.Spline(new, 5, knots)), g.fairness(g.Spline(ctrl, 5, knots)))
        self.assertEqual((new[0], new[-1]), (ctrl[0], ctrl[-1]))


class Fillet(unittest.TestCase):
    def test_profile_distance(self):
        phi = math.radians(90)  # legs +x and +y, R=1: tangent points (1,0),(0,1), centre (1,1)
        self.assertAlmostEqual(g.fillet_profile_distance((5.0, 0.0), phi), 0.0)
        c = (1 - math.cos(math.pi / 4), 1 - math.sin(math.pi / 4))  # arc midpoint
        self.assertAlmostEqual(g.fillet_profile_distance(c, phi), 0.0)
        self.assertAlmostEqual(g.fillet_profile_distance((0.0, 0.0), phi), math.sqrt(2) - 1)

    def test_layout(self):
        for deg in (60, 90, 135):
            phi = math.radians(deg)
            for count, G in ((7, 3), (9, 4)):
                r = g.fillet_layout(phi, 2.0, count)
                with self.subTest(deg=deg, count=count):
                    self.assertTrue(g.check_distances(r["dist"]))
                    self.assertLess(r["max_gap"], 0.06 * 2.0)
                    self.assertLess(r["min_radius"], 2.0)       # a G3 curve has to bend tighter than R
                    self.assertGreater(r["L"], r["arc_tangent"])  # ...and start further out
                    uA, uB = (1.0, 0.0), (math.cos(phi), math.sin(phi))
                    m, apex_mode, n, p = g.corner_style(count)
                    ctrl = g.corner_control_points((0, 0), uA, uB, r["dist"], r["dist"], apex_mode)
                    self.assertEqual(g.end_continuity_to_line(ctrl, p), G)

    def test_match_modes(self):
        phi, R = math.radians(90), 3.0
        d = g.fillet_layout(phi, R, 7, g.FILLET_DEPTH)
        self.assertAlmostEqual(d["depth"], d["arc_depth"], places=9)
        t = g.fillet_layout(phi, R, 7, g.FILLET_TANGENT)
        self.assertAlmostEqual(t["L"], t["arc_tangent"], places=9)
        mr = g.fillet_layout(phi, R, 7, g.FILLET_MINRADIUS)
        self.assertAlmostEqual(mr["min_radius"], R, places=6)
        uA, uB = (1.0, 0.0), (0.0, 1.0)
        ctrl = g.corner_control_points((0, 0), uA, uB, mr["dist"], mr["dist"], g.APEX_CORNER)
        self.assertAlmostEqual(1 / g.max_curvature(g.Spline(ctrl, 5), 4000), R, places=3)

    def test_smoothest_is_smoother_than_emulate(self):
        # compare comb acceleration at the same leg length (it scales as 1/size^5)
        for deg in (60, 90, 120):
            phi = math.radians(deg)
            for count in (7, 9):
                e = g.fillet_layout(phi, 1.0, count, emulate=True)
                s = g.fillet_layout(phi, 1.0, count, emulate=False)
                with self.subTest(deg=deg, count=count):
                    self.assertLess(s["comb_accel"] * s["L"] ** 5, e["comb_accel"] * e["L"] ** 5)
                    self.assertTrue(g.check_distances(s["dist"]))

    def test_default_fillet_comb_eases_in(self):
        # regression: minimising integral (dk/ds)^2 gave a comb that ramped up linearly right from
        # the ends (15% of peak curvature just 5% of the way along). G3 should ease in.
        for count in (7, 9):
            phi = math.radians(90)
            lay = g.fillet_layout(phi, 1.0, count)
            m, apex, n, deg = g.corner_style(count)
            s = g.Spline(g.corner_control_points((0, 0), (1.0, 0.0), (0.0, 1.0), lay["dist"], lay["dist"], apex), deg)
            N = 2000
            pts = [s.point(i / N) for i in range(N + 1)]
            S = [0.0]
            for a, b in zip(pts, pts[1:]):
                S.append(S[-1] + math.dist(a, b))
            K = [abs(s.curvature(i / N)) for i in range(N + 1)]
            i5 = min(range(N + 1), key=lambda j: abs(S[j] - 0.05 * S[-1]))
            self.assertLess(K[i5] / max(K), 0.05, count)

    def test_scales_with_radius(self):
        a = g.fillet_layout(math.radians(80), 1.0, 7)
        b = g.fillet_layout(math.radians(80), 2.5, 7)
        for x, y in zip(a["dist"], b["dist"]):
            self.assertAlmostEqual(y, 2.5 * x, places=9)


if __name__ == "__main__":
    unittest.main()
