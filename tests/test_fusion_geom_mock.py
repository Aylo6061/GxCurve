"""Runs fusion_geom's corner builder against fake Fusion objects to check constraint
bookkeeping (which points get which constraints). Run: python -m unittest discover tests
"""

import math
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


class MovablePt:
    _n = 0

    def __init__(self, x, y):
        MovablePt._n += 1
        self.entityToken = "pt%d" % MovablePt._n
        self.geometry = Pt(x, y)

    def move(self, v):
        self.geometry = Pt(self.geometry.x + v.x, self.geometry.y + v.y)


class Dim:
    """Distance dimension that, like Fusion's solver, slides entityTwo along the corner->point ray."""

    def __init__(self, e1, e2):
        self.entityOne, self.entityTwo, self.isDriving = e1, e2, True
        dim = self

        class Param:
            @property
            def value(self):
                return math.hypot(e2.geometry.x - e1.geometry.x, e2.geometry.y - e1.geometry.y)

            @value.setter
            def value(self, v):
                d = self.value
                ux, uy = (e2.geometry.x - e1.geometry.x) / d, (e2.geometry.y - e1.geometry.y) / d
                e2.geometry = Pt(e1.geometry.x + ux * v, e1.geometry.y + uy * v)
                dim.set_count += 1

        self.parameter = Param()
        self.set_count = 0


class ApplyPoints(unittest.TestCase):
    def test_dimension_or_move(self):
        corner = MovablePt(0, 0)
        pts = [MovablePt(4, 0), MovablePt(2, 0), MovablePt(0, 0), MovablePt(0, 3)]
        d1 = Dim(corner, pts[1])
        sk = mock.MagicMock()
        sk.sketchDimensions.count = 1
        sk.sketchDimensions.item.side_effect = lambda i: d1
        adsk.core.Vector3D.create = staticmethod(lambda x, y, z: Pt(x, y))
        target = [(4, 0), (2.5, 0), (0, 0), (0.5, 3)]
        resid = fg.apply_control_points(sk, pts, target)
        self.assertLess(resid, 1e-9)
        self.assertEqual(d1.set_count, 1)                      # point 1 went through its dimension
        self.assertAlmostEqual(pts[3].geometry.x, 0.5)          # point 3 was dragged


class V3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class RectFace:
    """Planar rectangle: origin + a*U + b*W for a, b in [0, 1]; fixed outward normal."""

    def __init__(self, origin, U, W, normal):
        self.o, self.U, self.W, self.nrm = origin, U, W, normal
        self.geometry = "plane"
        face = self

        class Ev:
            def getNormalAtPoint(self, p):
                return True, V3(*face.nrm)

            def getParameterAtPoint(self, p):
                r = [p.x - face.o[0], p.y - face.o[1], p.z - face.o[2]]
                a = sum(r[i] * face.U[i] for i in range(3)) / sum(u * u for u in face.U)
                b = sum(r[i] * face.W[i] for i in range(3)) / sum(w * w for w in face.W)
                return True, (a, b)

            def isParameterOnFace(self, uv):
                return -1e-9 <= uv[0] <= 1 + 1e-9 and -1e-9 <= uv[1] <= 1 + 1e-9

            def getPointAtParameter(self, uv):
                a, b = uv
                return True, V3(*[face.o[i] + a * face.U[i] + b * face.W[i] for i in range(3)])

        self.evaluator = Ev()


class FakeEdge:
    def __init__(self, p0, p1, faces):
        self.geometry = "line"
        self.startVertex, self.endVertex = mock.Mock(geometry=V3(*p0)), mock.Mock(geometry=V3(*p1))
        self.faces = mock.Mock(count=len(faces), item=lambda i: faces[i])
        self.body = "body"


class EdgeAnalysis(unittest.TestCase):
    def setUp(self):
        adsk.core.Line3D.cast = staticmethod(lambda x: x == "line")
        adsk.core.Plane.cast = staticmethod(lambda x: x == "plane")
        adsk.core.Point3D.create = staticmethod(lambda x, y, z: V3(x, y, z))
        from GX_tool.commands import gx_edge_fillet
        self.ef = gx_edge_fillet

    def test_box_edge_is_convex(self):
        # unit box, edge along y at x=1, z=1: top face (normal +z) and side face (normal +x)
        top = RectFace((0, 0, 1), (1, 0, 0), (0, 1, 0), (0, 0, 1))
        side = RectFace((1, 0, 0), (0, 0, 1), (0, 1, 0), (1, 0, 0))
        geo = self.ef.EdgeGeo(FakeEdge((1, 0, 1), (1, 1, 1), [top, side]))
        self.assertTrue(geo.convex)
        self.assertAlmostEqual(math.degrees(geo.phi), 90)
        self.assertEqual(tuple(round(c, 9) for c in geo.v[0]), (-1, 0, 0))   # into the top face
        self.assertEqual(tuple(round(c, 9) for c in geo.v[1]), (0, 0, -1))   # down the side

    def test_inside_corner_is_concave(self):
        # L-shaped step: floor z=0 for x in [0,1] (normal +z), wall x=0 for z in [0,1] (normal +x)
        floor = RectFace((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))
        wall = RectFace((0, 0, 0), (0, 0, 1), (0, 1, 0), (1, 0, 0))
        geo = self.ef.EdgeGeo(FakeEdge((0, 0, 0), (0, 1, 0), [floor, wall]))
        self.assertFalse(geo.convex)
        self.assertAlmostEqual(math.degrees(geo.phi), 90)

    def test_obtuse_edge(self):
        # chamfer-like 135 deg edge: top z=1 (x<=1) meets a 45 deg slope going down to +x
        top = RectFace((0, 0, 1), (1, 0, 0), (0, 1, 0), (0, 0, 1))
        s = math.sqrt(0.5)
        slope = RectFace((1, 0, 1), (1, 0, -1), (0, 1, 0), (s, 0, s))
        geo = self.ef.EdgeGeo(FakeEdge((1, 0, 1), (1, 1, 1), [top, slope]))
        self.assertTrue(geo.convex)
        self.assertAlmostEqual(math.degrees(geo.phi), 135)


class P2:
    def __init__(self, x, y):
        self.x, self.y = x, y


class CylTopFace:
    """Cylindrical face, axis along y through (0, *, -4), radius 5: points (5 sin a, y, -4 + 5 cos a)
    for a in [-asin(3/5), asin(3/5)], y in [0, 1]. Parameters (u, v) = (a, y)."""
    R0, A = 5.0, math.asin(3 / 5)

    def __init__(self):
        self.geometry = "cyl"
        face = self

        def pt(a, y):
            return V3(face.R0 * math.sin(a), y, -4 + face.R0 * math.cos(a))

        class Curve:  # iso-curve along u (the circle across the edge) at fixed y
            def __init__(self, y):
                c = self
                self.y = y

                class Ev:
                    def getParameterExtents(self):
                        return True, -math.pi, math.pi

                    def getParameterAtPoint(self, p):
                        return True, math.atan2(p.x, p.z + 4)

                    def getPointAtParameter(self, a):
                        return True, pt(a, c.y)

                    def getFirstDerivative(self, a):
                        return True, V3(face.R0 * math.cos(a), 0, -face.R0 * math.sin(a))

                    def getSecondDerivative(self, a):
                        return True, V3(-face.R0 * math.sin(a), 0, -face.R0 * math.cos(a))

                    def getThirdDerivative(self, a):
                        return True, V3(-face.R0 * math.cos(a), 0, face.R0 * math.sin(a))
                self.evaluator = Ev()

        class Line:  # iso-curve along v (a straight generator) at fixed a
            def __init__(self, a):
                class Ev:
                    def getParameterAtPoint(self, p):
                        return True, p.y

                    def getPointAtParameter(self, y):
                        return True, pt(a, y)

                    def getFirstDerivative(self, y):
                        return True, V3(0, 1, 0)
                self.evaluator = Ev()

        class Ev:
            def parametricRange(self):
                return mock.Mock(minPoint=P2(-face.A, 0), maxPoint=P2(face.A, 1))

            def getNormalAtParameter(self, uv):
                return True, V3(math.sin(uv.x), 0, math.cos(uv.x))

            def getNormalAtPoint(self, p):
                a = math.atan2(p.x, p.z + 4)
                return True, V3(math.sin(a), 0, math.cos(a))

            def getParameterAtPoint(self, p):
                return True, P2(math.atan2(p.x, p.z + 4), p.y)

            def isParameterOnFace(self, uv):
                return -face.A - 1e-9 <= uv.x <= face.A + 1e-9 and -1e-9 <= uv.y <= 1 + 1e-9

            def getPointAtParameter(self, uv):
                return True, pt(uv.x, uv.y)

            def getIsoCurve(self, par, is_u):
                # is_u: curve runs along u (the angle) at v = par
                c = Curve(par) if is_u else Line(par)
                return mock.Mock(count=1, item=lambda i: c)
        self.evaluator = Ev()


class CurvedEdge(unittest.TestCase):
    def setUp(self):
        adsk.core.Line3D.cast = staticmethod(lambda x: x == "line")
        adsk.core.Plane.cast = staticmethod(lambda x: x == "plane")
        adsk.core.Circle3D.cast = staticmethod(lambda x: None)
        adsk.core.Ellipse3D.cast = staticmethod(lambda x: None)
        adsk.core.Point3D.create = staticmethod(lambda x, y, z: V3(x, y, z))
        adsk.core.Point2D.create = staticmethod(lambda x, y: P2(x, y))
        from GX_tool.commands import gx_edge_fillet
        self.ef = gx_edge_fillet

    def test_cylinder_top_meets_flat_side(self):
        top = CylTopFace()
        # flat side x = 3, z from -2 to 0 (outward normal +x)
        side = RectFace((3, 0, -2), (0, 0, 2), (0, 1, 0), (1, 0, 0))
        geo = self.ef.EdgeGeo(FakeEdge((3, 0, 0), (3, 1, 0), [top, side]))
        self.assertFalse(geo.planar)
        self.assertTrue(geo.convex)
        (A, ua, sa), (B, ub, sb) = geo.sections
        self.assertAlmostEqual(abs(A.frame(ua, sa)["k"]), 1 / 5, places=9)  # the cylinder's curvature
        self.assertAlmostEqual(B.frame(ub, sb)["k"], 0.0, places=12)       # the flat side
        res = geo.curve_fillet(0.4, 7, g.FILLET_BEST, False)
        s = g.Spline(res["ctrl"], res["degree"])
        FA = res["FA"]
        self.assertAlmostEqual(s.curvature(0), FA["k"], places=7)        # G2 into the cylinder
        self.assertAlmostEqual(s.curvature_rate(0), FA["dk"], places=6)  # G3
        self.assertAlmostEqual(s.curvature(1), 0.0, places=7)
        # the contact points really are on the faces
        ca, cb = geo.from2d(res["ctrl"][0]), geo.from2d(res["ctrl"][-1])
        self.assertAlmostEqual(math.hypot(ca[0], ca[2] + 4), 5.0, places=9)
        self.assertAlmostEqual(cb[0], 3.0, places=9)
        self.assertTrue(geo.on_face(top, (ca[0], 0.5, ca[2])))
        self.assertTrue(geo.on_face(side, (cb[0], 0.5, cb[2])))
        # and the profile pieces: the cylinder part is an exact conic arc
        seg = g.conic_segment(A, ua, res["ua"])
        self.assertEqual(seg[0], "conic")
        self.assertEqual(g.conic_segment(B, ub, res["ub"])[0], "line")


class CircleEv:
    """CurveEvaluator3D for a circle in the sketch's xy plane, parameter = angle."""

    def __init__(self, c, r):
        self.c, self.r = c, r

    def getParameterExtents(self):
        return True, 0.0, 2 * math.pi

    def getParameterAtPoint(self, p):
        return True, math.atan2(p.y - self.c[1], p.x - self.c[0]) % (2 * math.pi)

    def getPointAtParameter(self, a):
        return True, V3(self.c[0] + self.r * math.cos(a), self.c[1] + self.r * math.sin(a), 0)

    def getFirstDerivative(self, a):
        return True, V3(-self.r * math.sin(a), self.r * math.cos(a), 0)

    def getSecondDerivative(self, a):
        return True, V3(-self.r * math.cos(a), -self.r * math.sin(a), 0)

    def getThirdDerivative(self, a):
        return True, V3(self.r * math.sin(a), -self.r * math.cos(a), 0)


class SketchCurveCorner(unittest.TestCase):
    def test_circle_and_line(self):
        sk = mock.Mock()
        sk.modelToSketchSpace = lambda p: p
        circle = mock.Mock(parentSketch=sk, geometry=mock.Mock(evaluator=CircleEv((0, 0), 5)))
        line = mock.Mock(parentSketch=sk, startSketchPoint=SP(3, -6), endSketchPoint=SP(3, 6))
        adsk.fusion.SketchLine.cast = staticmethod(lambda x: x if x is line else None)
        adsk.fusion.SketchCircle.cast = staticmethod(lambda x: x if x is circle else None)
        adsk.fusion.SketchEllipse.cast = staticmethod(lambda x: None)
        adsk.core.Point3D.create = staticmethod(lambda x, y, z: V3(x, y, z))
        # clicks: the circle above-left of (3, 4), the line above it
        cg = fg.CurveCornerGeo(circle, V3(5 * math.cos(1.3), 5 * math.sin(1.3), 0), line, V3(3, 5.5, 0))
        self.assertAlmostEqual(cg.X[0], 3.0, places=9)
        self.assertAlmostEqual(cg.X[1], 4.0, places=9)       # the upper crossing, near the clicks
        self.assertEqual((cg.sA, cg.sB), (1, 1))               # counter-clockwise on the circle, up the line
        res = cg.fillet(0.6, 7, g.FILLET_BEST, False)
        s = g.Spline(res["ctrl"], res["degree"])
        self.assertAlmostEqual(abs(s.curvature(0)), 0.2, places=7)
        self.assertAlmostEqual(s.curvature(1), 0.0, places=7)


if __name__ == "__main__":
    unittest.main()
