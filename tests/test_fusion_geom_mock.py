"""Runs fusion_geom's corner builder against fake Fusion objects to check constraint
bookkeeping (which points get which constraints). Run: python -m unittest discover tests
"""

import os
import sys
import unittest
import unittest.mock as mock
from collections import Counter

# Import GX_tool as a package from the folder above it. Inside the project folder,
# GX_tool.py would otherwise shadow the package name.
_proj = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != _proj]
sys.path.insert(0, os.path.dirname(_proj))
if getattr(sys.modules.get("GX_tool"), "__path__", None) is None:
    sys.modules.pop("GX_tool", None)

for _n in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(_n, mock.MagicMock())
sys.modules["adsk"].core = sys.modules["adsk.core"]
sys.modules["adsk"].fusion = sys.modules["adsk.fusion"]

import adsk.core  # noqa: E402
import adsk.fusion  # noqa: E402

from GX_tool.lib import futil  # noqa: E402
from GX_tool.lib import fusion_geom as fg  # noqa: E402

futil.log = futil.debug = lambda msg: None  # keep test runs out of logs/gx_tool.log
fg.log = fg.debug = futil.log
from GX_tool.lib import gx_math as g  # noqa: E402


class Pt:
    def __init__(self, x, y):
        self.x, self.y, self.z = x, y, 0.0


class SP:  # sketch point
    def __init__(self, x, y):
        self.geometry = Pt(x, y)


class Line:
    def __init__(self, sketch, a, b):
        self.parentSketch = sketch
        self.startSketchPoint, self.endSketchPoint = SP(*a), SP(*b)
        self.isConstruction = False


class Sketch:
    def __init__(self):
        self.log = []
        self.isComputeDeferred = False
        gc, dims = mock.MagicMock(), mock.MagicMock()
        gc.addCoincident.side_effect = lambda a, b: self.log.append(("coin", a, b))
        gc.addSymmetry.side_effect = lambda a, b, ax: self.log.append(("sym", a, b, ax))
        dims.addDistanceDimension.side_effect = lambda a, b, o, t: self.log.append(("dim", a, b))
        self.geometricConstraints, self.sketchDimensions = gc, dims
        self.sketchPoints = mock.MagicMock()
        self.sketchPoints.add.side_effect = lambda p: SP(0, 0)
        self.sketchCurves = mock.MagicMock()
        self.sketchCurves.sketchLines.addByTwoPoints.side_effect = lambda a, b: self._line(a, b)
        self.sketchCurves.sketchControlPointSplines.add.side_effect = self._spline

    def _line(self, a, b):
        ln = mock.MagicMock()
        self.log.append(("line", a, b))
        return ln

    def _spline(self, pts, deg):
        sp = mock.MagicMock()
        sp.controlPoints = [SP(0, 0) for _ in pts]
        self.spline_pts = sp.controlPoints
        return sp

    def modelToSketchSpace(self, p):
        return p


adsk.fusion.SketchPoint.cast = staticmethod(lambda x: x)


class CornerBuild(unittest.TestCase):
    def geo(self):
        sk = Sketch()
        # L-corner: line A along x toward origin, line B along y toward origin (sharing no point)
        la = Line(sk, (10, 0), (0, 0))
        lb = Line(sk, (0, 0), (0, 8))
        return sk, fg.CornerGeo(la, None, lb, None)

    def test_geometry(self):
        sk, geo = self.geo()
        self.assertEqual(geo.corner, (0.0, 0.0))
        self.assertEqual((geo.uA, geo.uB), ((1.0, 0.0), (0.0, 1.0)))
        self.assertEqual((geo.availA, geo.availB), (10.0, 8.0))

    def test_constraint_counts(self):
        for count, five, apex in ((5, "cubic", "corner"), (5, "quintic", "corner"), (7, "", "corner"),
                                  (7, "", "free"), (9, "", "corner"), (9, "", "free")):
            for sym in (True, False):
                for trim in (True, False):
                    sk, geo = self.geo()
                    m, apex_mode, n, _ = g.corner_style(count, five, apex)
                    fr = g.tension_fractions(m)
                    P = dict(count=count, five_type=five, apex=apex, symmetric=sym, trim=trim, dims=True,
                             distA=[4 * f for f in fr], distB=[3 * f for f in fr], apex_dist=0.5, comb=False)
                    spline, failed = fg.build_corner(geo, P, final=True)
                    with self.subTest(count=count, five=five, apex=apex, sym=sym, trim=trim):
                        self.assertEqual(failed, [])
                        self.assertEqual(len(sk.spline_pts), n)
                        kinds = Counter(e[0] for e in sk.log)
                        coin = 2 + m + (1 if apex_mode == g.APEX_CORNER else 0)
                        coin += 1 if sym else m
                        coin += 1 if (sym and apex_mode == g.APEX_FREE) else 0
                        self.assertEqual(kinds["coin"], coin)
                        self.assertEqual(kinds["sym"], m if sym else 0)
                        dims = m if sym else 2 * m
                        dims += 1 if (sym and apex_mode == g.APEX_FREE) else 0
                        self.assertEqual(kinds["dim"], dims)
                        self.assertEqual(kinds["line"], (2 if trim else 0) + (1 if sym else 0))
                        self.assertEqual(geo.lineA.isConstruction, trim)
                        # symmetry pairs A[i] with its mirror B[i]
                        pts = sk.spline_pts
                        for e in sk.log:
                            if e[0] == "sym":
                                self.assertEqual(pts.index(e[1]) + pts.index(e[2]), n - 1)
                        self.assertFalse(sk.isComputeDeferred)


if __name__ == "__main__":
    unittest.main()
