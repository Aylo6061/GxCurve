"""Numerical checks of the continuity claims. Run: python -m unittest discover tests"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import gx_math as g  # noqa: E402

CORNER = (1.0, 2.0)
UA = g.normalize((1.0, 0.2))
UB = g.normalize((-0.3, 1.0))
L = 5.0


def corner_spline(count, five_type="cubic", apex=g.APEX_CORNER, tension=0.5, LB=None, apex_dist=0.0):
    m, apex_mode, n, p = g.corner_style(count, five_type, apex)
    fr = g.tension_fractions(m, tension)
    dA = [L * f for f in fr]
    dB = [(LB or L) * f for f in fr]
    ctrl = g.corner_control_points(CORNER, UA, UB, dA, dB, apex_mode, apex_dist)
    assert len(ctrl) == n
    return g.Spline(ctrl, p)


def rate_order(s, at_start=True):
    """Ratio |dk/ds(2h)| / |dk/ds(h)| near an end: ~2 means G3 (rate ~ s), ~4 means G4."""
    h = 1e-5
    f = (lambda t: s.curvature_rate(t)) if at_start else (lambda t: s.curvature_rate(1 - t))
    return abs(f(2 * h)) / abs(f(h))


class CornerContinuity(unittest.TestCase):
    CASES = [
        # count, five_type, apex, expected G at ends
        (5, "cubic", g.APEX_CORNER, 2),
        (5, "quintic", g.APEX_CORNER, 2),
        (7, "cubic", g.APEX_CORNER, 3),
        (7, "cubic", g.APEX_FREE, 2),
        (9, "cubic", g.APEX_CORNER, 4),
        (9, "cubic", g.APEX_FREE, 3),
    ]

    def test_expected_table(self):
        for count, ft, apex, G in self.CASES:
            self.assertEqual(g.expected_corner_continuity(count, ft, apex), G, (count, ft, apex))

    def test_ends(self):
        for count, ft, apex, G in self.CASES:
            for tension in (0.2, 0.5, 0.8):
                s = corner_spline(count, ft, apex, tension, LB=3.0, apex_dist=0.7)
                with self.subTest(count=count, five=ft, apex=apex, tension=tension):
                    # G1: tangent along the legs
                    self.assertAlmostEqual(abs(g.cross(g.normalize(s.d(0, 1)), UA)), 0, places=9)
                    self.assertAlmostEqual(abs(g.cross(g.normalize(s.d(1, 1)), UB)), 0, places=9)
                    # G2: zero curvature
                    self.assertAlmostEqual(s.curvature(0), 0, places=9)
                    self.assertAlmostEqual(s.curvature(1), 0, places=9)
                    rate0, rate1 = abs(s.curvature_rate(0)), abs(s.curvature_rate(1))
                    if G >= 3:
                        self.assertAlmostEqual(rate0, 0, places=9)
                        self.assertAlmostEqual(rate1, 0, places=9)
                    else:
                        self.assertGreater(max(rate0, rate1), 1e-6, "should NOT be G3")
                    if G >= 4:
                        self.assertAlmostEqual(rate_order(s, True), 4, delta=0.1)
                        self.assertAlmostEqual(rate_order(s, False), 4, delta=0.1)
                    elif G == 3:
                        self.assertAlmostEqual(rate_order(s, True), 2, delta=0.1)
                    self.assertEqual(g.end_continuity_to_line(s.ctrl, s.p), G)
                    self.assertEqual(g.end_continuity_to_line(s.ctrl, s.p, from_end=True), G)

    def test_interior_curvature_continuous(self):
        for count, ft, apex, _ in self.CASES:
            s = corner_spline(count, ft, apex, 0.5, LB=3.0, apex_dist=0.7)
            inner = sorted(set(k for k in s.knots if 0 < k < 1))
            for k in inner:
                with self.subTest(count=count, five=ft, apex=apex, knot=k):
                    a, b = s.curvature(k - 1e-7), s.curvature(k + 1e-7)
                    self.assertAlmostEqual(a, b, delta=1e-4 * max(1, abs(a)))

    def test_symmetric_is_mirror(self):
        bis = g.bisector(UA, UB)
        for count, ft, apex, _ in self.CASES:
            s = corner_spline(count, ft, apex, 0.6, apex_dist=0.5)
            for t in (0.0, 0.1, 0.33, 0.5):
                p, q = s.point(t), s.point(1 - t)
                # reflect p about the bisector through the corner
                v = g.sub(p, CORNER)
                r = g.sub(g.mul(bis, 2 * g.dot(v, bis)), v)
                mp = g.add(CORNER, r)
                with self.subTest(count=count, five=ft, apex=apex, t=t):
                    self.assertAlmostEqual(mp[0], q[0], places=9)
                    self.assertAlmostEqual(mp[1], q[1], places=9)

    def test_quintic_five_equals_elevated_quartic(self):
        # the 6-point option is the degree-elevated 5-point Bezier (A A C B B)
        fr = [1.0, 0.5]
        P = g.corner_control_points(CORNER, UA, UB, [L * f for f in fr], [L * f for f in fr], g.APEX_CORNER)
        Q = [P[0]] + [g.add(g.mul(P[i - 1], i / 5), g.mul(P[i], 1 - i / 5)) for i in range(1, 5)] + [P[4]]
        dist = [g.length(g.sub(q, CORNER)) for q in Q[:3]]
        self.assertTrue(all(abs(a - b * L) < 1e-9 for a, b in zip(dist, [1.0, 0.6, 0.2])))

    def test_tension(self):
        self.assertEqual(g.tension_fractions(3, 0.5), [1.0, 2 / 3, 1 / 3])
        lo, hi = g.tension_fractions(4, 0.1), g.tension_fractions(4, 0.9)
        self.assertTrue(all(a > b for a, b in zip(lo[1:], hi[1:])))
        self.assertTrue(g.check_distances([5, 3, 1]))
        self.assertFalse(g.check_distances([5, 5, 1]))
        self.assertFalse(g.check_distances([5, 3, 0]))


class Freeform(unittest.TestCase):
    def test_free_ends(self):
        for n in (5, 7, 9):
            ctrl, p, gs, ge, w = g.freeform_control_points((0, 0), (10, 0), n)
            self.assertEqual(len(ctrl), n)
            self.assertEqual((ctrl[0], ctrl[-1]), ((0, 0), (10, 0)))
            self.assertIsNone(gs)

    def test_one_line(self):
        for n, G in ((5, 2), (7, 3), (9, 3)):
            ctrl, p, gs, ge, w = g.freeform_control_points((0, 0), (10, 5), n, dir_start=(1, 0))
            self.assertEqual(gs, G)
            s = g.Spline(ctrl, p)
            self.assertAlmostEqual(s.curvature(0), 0, places=9)

    def test_two_lines_meeting(self):
        # rays from (0,0) going +x and from (10,10) going -y meet at (10,0)
        for n, G in ((5, 2), (7, 3), (9, 3)):
            ctrl, p, gs, ge, w = g.freeform_control_points((0, 0), (10, 10), n, (1, 0), (0, -1))
            self.assertEqual((gs, ge, w), (G, G, ""), n)

    def test_two_lines_not_meeting(self):
        # parallel, same direction: rays never meet
        ctrl, p, gs, ge, w = g.freeform_control_points((0, 0), (10, 3), 5, (1, 0), (1, 0))
        self.assertEqual((gs, ge), (1, 1))
        self.assertTrue(w)
        ctrl, p, gs, ge, w = g.freeform_control_points((0, 0), (10, 3), 9, (1, 0), (1, 0))
        self.assertEqual((gs, ge, w), (3, 3, ""))


if __name__ == "__main__":
    unittest.main()
