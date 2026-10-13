"""Lip geometry (lib/lip.py): cross-sections, reveals, size checks and chaining edges. No Fusion needed.
Run: python -m unittest discover tests
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import lip as L  # noqa: E402


def line_at(p, q, v):
    """u where the line through p and q crosses height v."""
    return p[0] + (q[0] - p[0]) * (v - p[1]) / (q[1] - p[1])


def dist_to_line(p, a, b):
    return abs((b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1])) / math.dist(a, b)


class Sections(unittest.TestCase):
    def test_lip_without_draft_is_a_rectangle(self):
        self.assertEqual(L.lip_section(0.25, 0.5, 0.75, 0.0), [(0.25, 0.0), (0.75, 0.0), (0.75, 0.75), (0.25, 0.75)])

    def test_draft_leans_both_sides_in(self):
        a = math.radians(3)
        lip = L.lip_section(0.0, 0.2, 0.3, a)
        self.assertAlmostEqual(math.atan2(lip[3][0] - lip[0][0], lip[3][1] - lip[0][1]), a)   # near side leans in
        self.assertAlmostEqual(math.atan2(lip[1][0] - lip[2][0], lip[2][1] - lip[1][1]), a)   # far side too
        self.assertAlmostEqual(L.top_width(lip), 0.2 - 2 * 0.3 * math.tan(a))

    def test_groove_keeps_the_gaps_square_to_the_drafted_sides(self):
        a, near, far, top, below = math.radians(5), 0.01, 0.03, 0.02, 0.05
        lip = L.lip_section(0.05, 0.2, 0.3, a)
        g = L.groove_section(0.05, 0.2, 0.3, a, near, far, top, below)
        self.assertAlmostEqual(dist_to_line(g[0], lip[0], lip[3]), near)
        self.assertAlmostEqual(dist_to_line(g[3], lip[0], lip[3]), near)
        self.assertAlmostEqual(dist_to_line(g[1], lip[1], lip[2]), far)
        self.assertAlmostEqual(dist_to_line(g[2], lip[1], lip[2]), far)
        self.assertAlmostEqual(g[2][1], 0.3 + top)      # the top gap
        self.assertAlmostEqual(g[0][1], -below)         # reaches below the parting face
        # the groove's sides are parallel to the lip's
        self.assertAlmostEqual(line_at(g[0], g[3], 0.3) - line_at(g[0], g[3], 0.0), lip[3][0] - lip[0][0])

    def test_no_gaps_means_the_groove_fits_the_lip(self):
        lip = L.lip_section(0.0, 0.2, 0.3, math.radians(2))
        g = L.groove_section(0.0, 0.2, 0.3, math.radians(2), 0.0, 0.0, 0.0, 0.1)
        self.assertAlmostEqual(line_at(g[0], g[3], 0.0), lip[0][0])
        self.assertAlmostEqual(line_at(g[1], g[2], 0.0), lip[1][0])
        self.assertAlmostEqual(g[3][0], lip[3][0])
        self.assertAlmostEqual(g[2][0], lip[2][0])

    def test_sections_run_counterclockwise(self):
        for sec in (L.lip_section(0.0, 0.2, 0.3, 0.05), L.groove_section(0.0, 0.2, 0.3, 0.05, 0.01, 0.01, 0.01, 0.1)):
            area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(sec, sec[1:] + sec[:1])) / 2
            self.assertGreater(area, 0)

    def test_describe(self):
        top, wide, deep = L.describe(0.0, 0.2, 0.3, 0.0, 0.01, 0.02, 0.05)
        self.assertAlmostEqual(top, 0.2)
        self.assertAlmostEqual(wide, 0.23)
        self.assertAlmostEqual(deep, 0.35)


class Reveals(unittest.TestCase):
    def test_inner_and_outer_follow_the_side_the_face_runs_to(self):
        # edge on the parting face's outer boundary: the face runs inward, so the far side is the inner one
        self.assertEqual(L.gaps(L.INNER, 0.1, inner_far=True), (0.0, 0.1))
        self.assertEqual(L.gaps(L.OUTER, 0.1, inner_far=True), (0.1, 0.0))
        # edge on a hole in the parting face: the near side is the inner one
        self.assertEqual(L.gaps(L.INNER, 0.1, inner_far=False), (0.1, 0.0))
        self.assertEqual(L.gaps(L.OUTER, 0.1, inner_far=False), (0.0, 0.1))
        self.assertEqual(L.gaps(L.BOTH, 0.1, inner_far=True), (0.1, 0.1))
        self.assertEqual(L.gaps(L.NONE, 0.1, inner_far=True), (0.0, 0.0))


class Errors(unittest.TestCase):
    def test_good_sizes(self):
        self.assertEqual(L.errors(0.0, 0.1, 0.2, math.radians(1), 0.01, 0.0), [])

    def test_bad_sizes(self):
        self.assertTrue(L.errors(0.0, 0.0, 0.2, 0.0, 0.0, 0.0))
        self.assertTrue(L.errors(0.0, 0.1, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(L.errors(-0.1, 0.1, 0.2, 0.0, 0.0, 0.0))
        self.assertTrue(L.errors(0.0, 0.1, 0.2, 0.0, -0.01, 0.0))
        self.assertTrue(L.errors(0.0, 0.1, 0.2, math.radians(50), 0.0, 0.0))

    def test_too_much_draft(self):
        errs = L.errors(0.0, 0.1, 0.2, math.radians(20), 0.0, 0.0)
        self.assertEqual(len(errs), 1)
        self.assertIn("draft is too steep", errs[0])
        self.assertIn("%.2f°" % math.degrees(math.atan(0.25)), errs[0])


class Chains(unittest.TestCase):
    SQUARE = [((0, 0, 0), (1, 0, 0)), ((1, 0, 0), (1, 1, 0)), ((1, 1, 0), (0, 1, 0)), ((0, 1, 0), (0, 0, 0))]

    def ordered_points(self, ends, ch):
        out = []
        for i, rev in ch["edges"]:
            a, b = ends[i]
            out.append((b, a) if rev else (a, b))
        return out

    def check_connected(self, ends, ch):
        segs = self.ordered_points(ends, ch)
        for (_, b), (a, _) in zip(segs, segs[1:]):
            self.assertLess(math.dist(a, b), 1e-9)
        if ch["closed"]:
            self.assertLess(math.dist(segs[-1][1], segs[0][0]), 1e-9)

    def test_loop_in_any_order_and_direction(self):
        ends = [self.SQUARE[2], self.SQUARE[0][::-1], self.SQUARE[3], self.SQUARE[1][::-1]]
        chs = L.chains(ends)
        self.assertEqual(len(chs), 1)
        self.assertTrue(chs[0]["closed"])
        self.assertEqual(sorted(i for i, _ in chs[0]["edges"]), [0, 1, 2, 3])
        self.check_connected(ends, chs[0])

    def test_open_chain_starts_at_a_free_end(self):
        ends = [self.SQUARE[1], self.SQUARE[0][::-1], self.SQUARE[2]]
        chs = L.chains(ends)
        self.assertEqual(len(chs), 1)
        self.assertFalse(chs[0]["closed"])
        self.check_connected(ends, chs[0])
        start = self.ordered_points(ends, chs[0])[0][0]
        self.assertIn(start, ((0, 0, 0), (0, 1, 0)))

    def test_two_loops_and_a_closed_edge(self):
        hole = [tuple((x + 5, y, z) for x, y, z in seg) for seg in [((0, 0, 0), (1, 0, 0))]]
        ends = self.SQUARE + [((8, 0, 0), (8, 0, 0))] + hole
        chs = L.chains(ends)
        self.assertEqual(len(chs), 3)
        self.assertEqual(sorted(len(c["edges"]) for c in chs), [1, 1, 4])
        self.assertEqual([c["closed"] for c in chs].count(True), 2)  # the square and the circle

    def test_endpoints_within_tolerance_join(self):
        ends = [((0, 0, 0), (1, 0, 0)), ((1 + 1e-8, 0, 0), (2, 0, 0))]
        self.assertEqual(len(L.chains(ends, tol=1e-6)), 1)

    def test_branching_is_an_error(self):
        with self.assertRaises(ValueError):
            L.chains(self.SQUARE + [((0, 0, 0), (-1, 0, 0))])


class Frames(unittest.TestCase):
    def test_frame_map(self):
        f = L.frame_map((1.0, 2.0, 3.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self.assertEqual(f((0.5, 0.25)), (1.0, 2.5, 3.25))


if __name__ == "__main__":
    unittest.main()
