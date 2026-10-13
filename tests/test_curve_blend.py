"""G2/G3 blends into conics (circle, ellipse, parabola, hyperbola). Run: python -m unittest discover tests"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import gx_math as g  # noqa: E402


def cases():
    """(name, A, uX, sA, B, vX, sB, R): curves crossing at A(uX) = B(vX); sA/sB = kept direction."""
    out = []
    circle = g.ellipse_curve((0, 0), 5, 5)
    out.append(("circle-line", circle, math.atan2(4, 3), +1, g.line_curve((3, 0), (0, 1)), 4.0, +1, 0.8))
    ell = g.ellipse_curve((0, 0), 6, 3, 0.3)
    line = g.line_curve((-10, 1.0), (1, 0.05))
    hit = None
    for u0 in (0.5, 1.0, 1.5):
        hit = g.find_intersection(ell, u0, line, 15.0)
        if hit:
            break
    out.append(("ellipse-line", ell, hit[0], +1, line, hit[1], +1, 0.6))
    par = g.parabola_curve((0, 0), 1.0)  # y = x^2/4
    out.append(("parabola-line", par, 2.0, +1, g.line_curve((0, 1), (1, 0)), 2.0, -1, 0.5))
    hyp = g.hyperbola_curve((0, 0), 1.0, 1.0)
    u = math.asinh(math.sqrt(3))  # (2, sqrt 3)
    out.append(("hyperbola-line", hyp, u, +1, g.line_curve((2, 0), (0, 1)), math.sqrt(3), -1, 0.4))
    e1, e2 = g.ellipse_curve((0, 0), 4, 2), g.ellipse_curve((3, 0), 3, 2.5)
    hit = g.find_intersection(e1, 1.0, e2, 2.0)
    out.append(("ellipse-ellipse", e1, hit[0], -1, e2, hit[1], -1, 0.5))
    return out


def check_contact(tc, blend_spline, t, frame, G):
    """blend at parameter t (0 or 1) vs the curve frame (frame T = direction the blend travels)."""
    p = blend_spline.point(t)
    tc.assertLess(g.length(g.sub(p, frame["P"])), 1e-9)
    d1 = g.normalize(blend_spline.d(t, 1))
    tc.assertAlmostEqual(g.cross(d1, frame["T"]), 0.0, places=8)
    tc.assertGreater(g.dot(d1, frame["T"]), 0)
    tc.assertAlmostEqual(blend_spline.curvature(t), frame["k"], places=7)
    if G >= 3:
        tc.assertAlmostEqual(blend_spline.curvature_rate(t), frame["dk"], delta=1e-6 * max(1.0, abs(frame["dk"])))


class CurveBlend(unittest.TestCase):
    def test_curves_and_intersections(self):
        for name, A, uX, sA, B, vX, sB, R in cases():
            self.assertLess(g.length(g.sub(A.point(uX), B.point(vX))), 1e-7, name)

    def test_conic_frames(self):
        # circle radius 5 has curvature 1/5 and zero rate; ellipse vertex curvature a/b^2
        c = g.ellipse_curve((0, 0), 5, 5).frame(0.3, +1)
        self.assertAlmostEqual(c["k"], 0.2)
        self.assertAlmostEqual(c["dk"], 0.0)
        e = g.ellipse_curve((0, 0), 6, 3).frame(0.0, +1)
        self.assertAlmostEqual(e["k"], 6 / 9)
        self.assertAlmostEqual(g.ellipse_curve((0, 0), 6, 3).frame(0.0, -1)["k"], -6 / 9)
        p = g.parabola_curve((0, 0), 1.0).frame(0.0, +1)
        self.assertAlmostEqual(p["k"], 0.5)  # 1/(2f)

    def test_g3_into_conics(self):
        for name, A, uX, sA, B, vX, sB, R in cases():
            for count, ft, G in ((7, "cubic", 3), (5, "quintic", 2)):
                r = g.curve_fillet(A, uX, sA, B, vX, sB, R, count, emulate=False, five_type=ft)
                s = g.Spline(r["ctrl"], r["degree"])
                with self.subTest(case=name, count=count, five=ft):
                    check_contact(self, s, 0.0, r["FA"], G)
                    # far end: blend arrives travelling in B's kept direction
                    check_contact(self, s, 1.0, B.frame(r["ub"], sB), G)
                    self.assertLess(r["max_gap"], 0.2 * R)
                    self.assertGreater(r["min_radius"], 0.2 * R)

    def test_emulate_hugs_arc(self):
        name, A, uX, sA, B, vX, sB, R = cases()[1]  # ellipse-line
        smooth = g.curve_fillet(A, uX, sA, B, vX, sB, R, 7, emulate=False)
        emu = g.curve_fillet(A, uX, sA, B, vX, sB, R, 7, emulate=True)
        self.assertLess(emu["max_gap"], smooth["max_gap"])
        s = g.Spline(emu["ctrl"], emu["degree"])
        check_contact(self, s, 0.0, emu["FA"], 3)

    def test_nine_points(self):
        name, A, uX, sA, B, vX, sB, R = cases()[0]
        r = g.curve_fillet(A, uX, sA, B, vX, sB, R, 9)
        s = g.Spline(r["ctrl"], r["degree"])
        self.assertEqual(len(r["ctrl"]), 9)
        check_contact(self, s, 0.0, r["FA"], 3)
        check_contact(self, s, 1.0, B.frame(r["ub"], sB), 3)

    def test_lines_match_straight_fillet(self):
        # two lines through the engine should land close to the existing straight-line fillet
        A, B = g.line_curve((0, 0), (1, 0)), g.line_curve((0, 0), (0, 1))
        r = g.curve_fillet(A, 0.0, +1, B, 0.0, +1, 1.0, 7)
        lay = g.fillet_layout(math.pi / 2, 1.0, 7)
        self.assertAlmostEqual(r["ua"], lay["L"], places=6)
        self.assertAlmostEqual(r["ub"], lay["L"], places=6)
        self.assertLess(abs(r["max_gap"] - lay["max_gap"]), 0.01)
        self.assertEqual(g.end_continuity_to_line(r["ctrl"], 5), 3)

    def test_conic_segment_is_exact(self):
        for cur, a, b in ((g.ellipse_curve((1, 2), 5, 2, 0.4), 0.2, 1.3), (g.hyperbola_curve((0, 0), 1, 2), -0.4, 0.9),
                          (g.ellipse_curve((0, 0), 3, 3), 0.3, 1.9), (g.parabola_curve((0, 0), 1.0), -1.0, 2.5)):
            kind, p0, p1, apex, rho = g.conic_segment(cur, a, b)
            self.assertEqual(kind, "conic")
            W = rho / (1 - rho)
            for i in range(21):
                t = i / 20
                b0, b1, b2 = (1 - t) ** 2, 2 * t * (1 - t) * W, t * t
                s = b0 + b1 + b2
                q = ((b0 * p0[0] + b1 * apex[0] + b2 * p1[0]) / s, (b0 * p0[1] + b1 * apex[1] + b2 * p1[1]) / s)
                u = cur.closest(q, a + (b - a) * t)
                self.assertLess(g.length(g.sub(q, cur.point(u))), 1e-9)
        self.assertEqual(g.conic_segment(g.line_curve((0, 0), (1, 1)), 0, 2)[0], "line")

    def test_too_big_radius(self):
        arcA = g.Curve2D(g.ellipse_curve((0, 0), 5, 5).fn, 0.6, 1.2)  # short arc
        with self.assertRaises(ValueError):
            g.curve_fillet(arcA, math.atan2(4, 3), +1, g.line_curve((3, 0), (0, 1)), 4.0, +1, 3.0)


if __name__ == "__main__":
    unittest.main()
