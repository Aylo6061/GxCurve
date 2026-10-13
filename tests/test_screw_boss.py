"""Tests for the screw boss sizes and checks. Run: python -m unittest discover tests"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))
import screw_boss as sb  # noqa: E402


class Sizes(unittest.TestCase):
    def test_tap_drills(self):
        self.assertEqual(sb.SIZES, ("M2", "M2.5", "M3", "M4", "M5", "M6", "M8", "M10"))
        self.assertEqual(sb.TAP_DRILL, {"M2": 1.6, "M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2, "M6": 5.0, "M8": 6.8,
                                        "M10": 8.5})
        for s in sb.SIZES:
            self.assertEqual(sb.preset_hole(sb.TAP, s), sb.TAP_DRILL[s])
            # within a standard drill of the size less the pitch
            self.assertLessEqual(abs(sb.TAP_DRILL[s] - (sb.NOMINAL[s] - sb.PITCH[s])), 0.05)

    def test_self_tapping(self):
        self.assertEqual([sb.preset_hole(sb.SELF_TAP, s) for s in sb.SIZES], [1.6, 2.0, 2.4, 3.2, 4.0, 4.8, 6.4, 8.0])
        self.assertEqual(sb.suggested_od(sb.SELF_TAP, "M3", 2.4), 6.0)

    def test_inserts(self):
        for s in sb.SIZES:
            rows = sb.INSERTS[s]
            self.assertTrue(rows)
            self.assertIn(sb.TYPICAL[s], [r["part"] for r in rows])
            for r in rows:
                self.assertTrue(r["part"].startswith("94459A"))
                # the drill makes a hole bigger than the thread and no bigger than McMaster's largest
                self.assertGreater(r["hole"], sb.NOMINAL[s])
                self.assertLessEqual(r["hole"], r["max_hole"])
        m3 = sb.insert("M3")
        self.assertEqual((m3["part"], m3["length"], m3["hole"]), ("94459A140", 5.7, 3.99))
        self.assertEqual(sb.insert("M3", "94459A130")["length"], 4.3)
        self.assertEqual(sb.insert("M3", "not-a-part")["part"], "94459A140")
        self.assertEqual(sb.preset_hole(sb.INSERT, "M3"), 3.99)
        self.assertEqual(sb.preset_hole(sb.INSERT, "M3", "94459A771"), 4.76)
        self.assertIsNone(sb.preset_hole(sb.CUSTOM, "M3"))

    def test_insert_depth(self):
        # the insert's length plus two pitches, at least 1 mm
        self.assertEqual(sb.insert_depth("M3"), 6.7)
        self.assertEqual(sb.insert_depth("M2"), 5.0)
        self.assertEqual(sb.insert_depth("M4"), 7.8)
        self.assertEqual(sb.insert_depth("M6", "94459A220"), 14.7)
        self.assertEqual(sb.insert_depth("M8"), 15.2)
        self.assertEqual(sb.insert_depth("M10"), 15.7)

    def test_describe(self):
        self.assertEqual(sb.describe(sb.TAP, "M4"), "tap drill for M4 × 0.7")
        self.assertIn("94459A140", sb.describe(sb.INSERT, "M3"))
        self.assertIn("0.8", sb.describe(sb.SELF_TAP, "M3"))
        self.assertEqual(sb.describe(sb.CUSTOM, "M3"), "custom diameter")

    def test_boss_od(self):
        self.assertEqual(sb.boss_od(2.5, sb.WALL, 2.0), 6.5)
        self.assertEqual(sb.boss_od(2.5, sb.OD, 7.0), 7.0)


class Placement(unittest.TestCase):
    def test_merge_points(self):
        self.assertEqual(sb.merge_points([(0, 0), (1, 0), (0, 0), (1, 1e-9)]), [0, 1])

    def test_overlapping(self):
        pts = [(0, 0), (0.5, 0), (2, 0)]
        self.assertEqual(sb.overlapping(pts, 0.6), [(0, 1)])
        self.assertEqual(sb.overlapping(pts, 0.5), [])  # just touching

    def test_group_levels(self):
        self.assertEqual(sb.group_levels([1.0, 0.5, 1.0 + 1e-7, 0.5]), [[1, 3], [0, 2]])
        self.assertEqual(sb.group_levels([2.0]), [[0]])


if __name__ == "__main__":
    unittest.main()
