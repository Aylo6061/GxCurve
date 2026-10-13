"""Fake Fusion objects for driving the polygrid commands without Fusion (see the *_mock tests).

Covers what the commands touch: sketches (curves, shared end points, profiles worked out from the
drawn loops), a timeline (marker, rolling, groups), extrude features, command inputs and the
dialog's events. Fusion rules the commands rely on are enforced: a sketch on a face only gives its
face with the marker just before it, and an extrude's inputs can only change with the marker just
before it.
"""

import math
import os
import re
import sys
import types
import unittest.mock as mock

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

from GX_tool.lib import comb_graphics, futil  # noqa: E402
from GX_tool.lib import fusion_geom as fg  # noqa: E402
from GX_tool.lib import polygrid as W  # noqa: E402
from GX_tool.commands import gx_comb  # noqa: E402

futil.log = futil.debug = lambda msg: None
fg.log = fg.debug = futil.log


# --------------------------------------------------------------------------- geometry
class Pt:
    objectType = "adsk::core::Point3D"

    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distanceTo(self, o):
        return math.sqrt((self.x - o.x) ** 2 + (self.y - o.y) ** 2 + (self.z - o.z) ** 2)

    def copy(self):
        return Pt(self.x, self.y, self.z)

    def translateBy(self, v):
        self.x, self.y, self.z = self.x + v.x, self.y + v.y, self.z + v.z


def xy(p):
    return (p.x, p.y)


class Collection(list):
    def add(self, x):
        self.append(x)


class Items:
    def __init__(self, items):
        self.items = list(items)

    @property
    def count(self):
        return len(self.items)

    def item(self, k):
        return self.items[k]


class TYPES:
    Line3DCurveType, Arc3DCurveType, Circle3DCurveType, NurbsCurve3DCurveType = 1, 2, 3, 4


class Eval:
    def __init__(self, fn, t0=0.0, t1=1.0):
        self.fn, self.t0, self.t1 = fn, t0, t1

    def getParameterExtents(self):
        return True, self.t0, self.t1

    def getPointAtParameter(self, t):
        return True, Pt(*self.fn(t))

    def getStrokes(self, t0, t1, tol):
        return True, [Pt(*self.fn(t0 + (t1 - t0) * i / 64)) for i in range(65)]


class Curve3D:
    """Curve geometry, from a polygrid primitive."""

    def __init__(self, prim):
        k = prim[0]
        if k == "line":
            self.curveType = TYPES.Line3DCurveType
            self.startPoint, self.endPoint = Pt(*prim[1]), Pt(*prim[2])
        elif k == "arc" and abs(prim[4]) >= 2 * math.pi - 1e-9:
            self.curveType = TYPES.Circle3DCurveType
            self.center, self.radius = Pt(*prim[1]), prim[2]
        elif k == "arc":
            c, r, a0, sw = prim[1:]
            self.curveType = TYPES.Arc3DCurveType
            self.center, self.radius = Pt(*c), r
            lo = a0 if sw > 0 else a0 + sw  # Fusion arcs run CCW
            fn = lambda t: (c[0] + r * math.cos(lo + abs(sw) * t), c[1] + r * math.sin(lo + abs(sw) * t))
            self.evaluator = Eval(fn)
            self.startPoint, self.endPoint = Pt(*fn(0)), Pt(*fn(1))
        else:
            pts = prim[1]
            self.curveType = TYPES.NurbsCurve3DCurveType
            self.evaluator = Eval(lambda t: pts[min(len(pts) - 1, int(round(t * (len(pts) - 1))))])


def box_of(pts):
    x0, y0, x1, y1 = W.bbox(pts)
    return mock.Mock(minPoint=Pt(x0, y0), maxPoint=Pt(x1, y1))


# --------------------------------------------------------------------------- entities
class Attributes:
    def __init__(self):
        self.d = {}

    def add(self, group, name, value):
        self.d[(group, name)] = mock.Mock(value=value)
        return self.d[(group, name)]

    def itemByName(self, group, name):
        return self.d.get((group, name))


TOKENS = {}


class Entity:
    isValid = True
    _n = 0

    def __init__(self):
        Entity._n += 1
        self.entityToken = "tok%d" % Entity._n
        TOKENS[self.entityToken] = self
        self.attributes = Attributes()

    def deleteMe(self):
        self.isValid = False


class ProfileCurve:
    def __init__(self, prim):
        self.geometry = Curve3D(prim)

    @property
    def sketchEntity(self):
        raise RuntimeError("not modelled")  # fusion_geom falls back to sketch space


class Profile(Entity):
    """A profile of the area's sketch, from polygrid loops."""
    objectType = "adsk::fusion::Profile"

    def __init__(self, sketch, loops):
        super().__init__()
        self.parentSketch = sketch
        self.loops = loops
        self.profileLoops = Items([mock.Mock(profileCurves=Items([ProfileCurve(p) for p in lp])) for lp in loops])
        self.boundingBox = box_of([q for lp in W.prepare_area(loops) for q in W.loop_polyline(lp)])


class GridProfile(Entity):
    """A profile of a drawn grid sketch; pt is a point inside it (see Sketch.profiles)."""
    objectType = "adsk::fusion::Profile"

    def __init__(self, sketch, pt):
        super().__init__()
        self.parentSketch, self.pt = sketch, pt


class Face(Entity):
    objectType = "adsk::fusion::BRepFace"

    def __init__(self, **kw):
        super().__init__()
        self.__dict__.update(kw)


class Box:
    def __init__(self, lo, hi):
        self.lo, self.hi = tuple(lo), tuple(hi)
        self.minPoint, self.maxPoint = Pt(*self.lo), Pt(*self.hi)

    def intersects(self, o):
        return all(self.lo[k] <= o.hi[k] and o.lo[k] <= self.hi[k] for k in range(3))

    def moved(self, v):
        return Box([a + b for a, b in zip(self.lo, v)], [a + b for a, b in zip(self.hi, v)])


class Body(Entity):
    objectType = "adsk::fusion::BRepBody"

    def __init__(self, box=None):
        super().__init__()
        self.boundingBox = box or Box((-100, -100, -100), (100, 100, 100))


def v3(p):
    return (p.x, p.y, p.z)


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit3(a):
    n = math.sqrt(_dot3(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class PolyEval:
    """An evaluator over a model-space polyline, parameter 0..1."""

    def __init__(self, pts):
        self.pts = pts

    def getParameterExtents(self):
        return True, 0.0, 1.0

    def getPointAtParameter(self, t):
        k = min(int(t * (len(self.pts) - 1)), len(self.pts) - 2)
        f = t * (len(self.pts) - 1) - k
        a, b = self.pts[k], self.pts[k + 1]
        return True, Pt(*[a[i] + (b[i] - a[i]) * f for i in range(3)])

    def getFirstDerivative(self, t):
        k = min(int(t * (len(self.pts) - 1)), len(self.pts) - 2)
        return True, Pt(*_sub3(self.pts[k + 1], self.pts[k]))


class SketchPoint(Entity):
    objectType = "adsk::fusion::SketchPoint"

    def __init__(self, sketch, p):
        super().__init__()
        self.parentSketch, self.geometry = sketch, Pt(p.x, p.y, p.z)
        sketch.points.append(self)

    def merge(self, other):
        for c in self.parentSketch.curves:
            if c.startSketchPoint is other:
                c.startSketchPoint = self
            if c.endSketchPoint is other:
                c.endSketchPoint = self
        other.isValid = False
        self.parentSketch.merges += 1
        return True


class SketchCurve(Entity):
    objectType = "adsk::fusion::SketchCurve"

    def __init__(self, sketch, kind, a=None, b=None, **kw):
        super().__init__()
        self.parentSketch, self.kind, self.isFixed = sketch, kind, False
        self.startSketchPoint = a if a is None or isinstance(a, SketchPoint) else SketchPoint(sketch, a)
        self.endSketchPoint = b if b is None or isinstance(b, SketchPoint) else SketchPoint(sketch, b)
        self.__dict__.update(kw)
        sketch.curves.append(self)

    def polyline(self):
        """Points from the start point to the end point (closed curves: around)."""
        if self.kind == "circle":
            c, r = xy(self.center), self.radius
            return [(c[0] + r * math.cos(2 * math.pi * i / 48), c[1] + r * math.sin(2 * math.pi * i / 48)) for i in range(48)]
        if self.kind == "line":
            return [xy(self.startSketchPoint.geometry), xy(self.endSketchPoint.geometry)]
        if self.kind == "arc":
            return [xy(self.startSketchPoint.geometry), xy(self.mid), xy(self.endSketchPoint.geometry)]
        pts = [xy(q) for q in (self.ctrl if self.kind == "cspline" else self.fit)]
        if xy(self.startSketchPoint.geometry) != pts[0]:
            pts.reverse()
        return pts

    def point_on(self):
        """A point on the curve, away from its ends (like BRepEdge.pointOnEdge)."""
        pts = self.polyline()
        if self.kind == "line":
            return ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
        if self.kind == "cspline":
            return W.g.Spline(pts, self.ctrl and 5).point(0.5)
        return pts[len(pts) // 2]  # arcs: their middle point; circles, fitted splines: a point they pass

    def model_points(self, z=0.0):
        sk = self.parentSketch
        return [v3(sk.sketchToModelSpace(Pt(q[0], q[1], z))) for q in self.polyline()]

    @property
    def worldGeometry(self):
        return mock.Mock(evaluator=PolyEval(self.model_points()))


class Curves:
    def __init__(self, sketch, arcs_take_points):
        sk = sketch
        self.sketch = sketch
        self.sketchLines = mock.Mock()
        self.sketchLines.addByTwoPoints.side_effect = lambda a, b: SketchCurve(sk, "line", a, b)
        self.sketchArcs = mock.Mock()

        def arc(a, m, b):
            if not arcs_take_points and (isinstance(a, SketchPoint) or isinstance(b, SketchPoint)):
                raise RuntimeError("Point3D expected")
            return SketchCurve(sk, "arc", b, a, mid=m)  # Fusion may store an arc the other way round
        self.sketchArcs.addByThreePoints.side_effect = arc
        self.sketchCircles = mock.Mock()
        self.sketchCircles.addByCenterRadius.side_effect = lambda c, r: SketchCurve(
            sk, "circle", center=c.geometry if isinstance(c, SketchPoint) else c, radius=r, centerSketchPoint=c)
        self.sketchControlPointSplines = mock.Mock()
        self.sketchControlPointSplines.add.side_effect = \
            lambda pts, deg: SketchCurve(sk, "cspline", pts[0], pts[-1], ctrl=list(pts))
        self.sketchFittedSplines = mock.Mock()
        self.sketchFittedSplines.add.side_effect = \
            lambda coll: SketchCurve(sk, "spline", coll[0], coll[-1], fit=list(coll))

    @property
    def count(self):
        return len([c for c in self.sketch.curves if c.isValid])

    def item(self, k):
        return [c for c in self.sketch.curves if c.isValid][k]


class Dimensions:
    """A sketch's dimensions (diameter dimensions only). Like Fusion, a dimension goes when its
    curve is deleted."""

    def __init__(self):
        self.made = []
        self.addDiameterDimension = mock.Mock(side_effect=self._diameter)

    def _diameter(self, curve, text_point, driving=True):
        d = types.SimpleNamespace(kind="diameter", entity=curve, parameter=types.SimpleNamespace(
            expression="%.12g cm" % (2 * curve.radius), value=2 * curve.radius))
        self.made.append(d)
        return d

    def _live(self):
        return [d for d in self.made if d.entity.isValid]

    @property
    def count(self):
        return len(self._live())

    def item(self, k):
        return self._live()[k]


class Sketch(Entity):
    objectType = "adsk::fusion::Sketch"

    def __init__(self, name, comp, arcs_take_points=True, offset=(0.0, 0.0), frame=None):
        super().__init__()
        # sketch space -> model: origin + x X + y Y + z Z
        self.frame = frame or ((offset[0], offset[1], 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self.name, self.parentComponent, self.isVisible = name, comp, True
        self.curves, self.points, self.merges = [], [], 0
        self.sketchCurves = Curves(self, arcs_take_points)
        self.isComputeDeferred = False
        self.nativeObject = None
        self.on_face = False
        self.given_profiles = None
        comp.timeline.insert(self)
        self.timelineObject = TimelineObject(comp.timeline, self)
        self.offset = offset  # sketch space = model space shifted by offset
        self.originPoint = SketchPoint(self, Pt(0, 0))
        self.geometricConstraints = mock.Mock()
        self.sketchDimensions = Dimensions()

    @property
    def referencePlane(self):
        if not self.on_face:
            return "plane"
        tl = self.parentComponent.timeline
        if tl.markerPosition != tl.items.index(self):
            raise RuntimeError("3 : referencePlane is a BRefFace - need to roll timeline back before sketch")
        if getattr(self, "face", None) is None:
            self.face = Face()
        return self.face

    @property
    def profiles(self):
        """Given profiles, or worked out from the drawn curves: chains of curves sharing end
        points (and closed curves) are loops, and each loop's inside, less the loops directly
        inside it, is a profile."""
        if self.given_profiles is not None:
            return Items(self.given_profiles)
        curves = [c for c in self.curves if c.isValid]
        loops = [c.polyline() for c in curves if c.kind == "circle" or getattr(c, "isClosed", False)]
        chain = [c for c in curves if not (c.kind == "circle" or getattr(c, "isClosed", False))]
        at_pt = {}
        for c in chain:
            for sp in (c.startSketchPoint, c.endSketchPoint):
                at_pt.setdefault(id(sp), []).append(c)
        used = set()
        for c0 in chain:
            if id(c0) in used:
                continue
            pts, c, at = [], c0, c0.startSketchPoint
            while c is not None:
                used.add(id(c))
                seg = c.polyline()
                if c.startSketchPoint is not at:
                    seg.reverse()
                pts.extend(seg[:-1])
                at = c.endSketchPoint if c.startSketchPoint is at else c.startSketchPoint
                c = next((d for d in at_pt.get(id(at), []) if id(d) not in used), None)
            loops.append(pts)
        # like Fusion: every loop bounds a region (its inside, less the loops directly inside it)
        loops = [pts for pts in loops if len(pts) >= 3]
        sizes = [abs(W.poly_area(pts)) for pts in loops]
        parent = []
        for i, pts in enumerate(loops):
            around = [j for j in range(len(loops)) if j != i and sizes[j] > sizes[i] and W.point_in_poly(pts[0], loops[j])]
            parent.append(min(around, key=lambda j: sizes[j]) if around else None)
        out = []
        for i, pts in enumerate(loops):
            area = [[("curve", q + [q[0]])] for q in [pts] + [loops[k] for k in range(len(loops)) if parent[k] == i]]
            out.append(GridProfile(self, W.interior_point(W.prepare_area(area))))
        return Items(out)

    @property
    def sketchPoints(self):
        return Items([p for p in self.points if p.isValid])

    def sketchToModelSpace(self, p):
        o, X, Y, Z = self.frame
        return Pt(*[o[i] + X[i] * p.x + Y[i] * p.y + Z[i] * p.z for i in range(3)])

    def modelToSketchSpace(self, p):
        o, X, Y, Z = self.frame
        d = _sub3(v3(p), o)
        return Pt(_dot3(d, X), _dot3(d, Y), _dot3(d, Z))

    def chain(self, curve):
        """The curves joined end to end with this one."""
        out, todo = [], [curve]
        while todo:
            c = todo.pop()
            if any(c is x for x in out):
                continue
            out.append(c)
            for d in self.curves:
                if d.isValid and d.startSketchPoint is not None and not any(d is x for x in out) and \
                        {id(d.startSketchPoint), id(d.endSketchPoint)} & {id(c.startSketchPoint), id(c.endSketchPoint)}:
                    todo.append(d)
        return out


class FakeLine(Entity):
    objectType = "adsk::fusion::SketchLine"

    def __init__(self, a, b):
        super().__init__()
        self.worldGeometry = mock.Mock(startPoint=Pt(*a), endPoint=Pt(*b))


# --------------------------------------------------------------------------- timeline and features
class Timeline:
    def __init__(self):
        self.items = ["Extrude1"]
        self.markerPosition = 1
        self.groups = []
        tl = self
        self.timelineGroups = mock.Mock()

        def add(i0, i1):
            g = Group(tl, i0, i1)
            tl.groups.append(g)
            return g
        self.timelineGroups.add.side_effect = add

    @property
    def count(self):
        return len(self.items)

    def moveToEnd(self):
        self.markerPosition = len(self.items)

    def insert(self, obj):
        self.items.insert(self.markerPosition, obj)
        self.markerPosition += 1

    def remove(self, obj):
        i = self.items.index(obj)
        self.items.pop(i)
        if self.markerPosition > i:
            self.markerPosition -= 1


class TimelineObject:
    def __init__(self, tl, obj):
        self.tl, self.obj = tl, obj

    @property
    def entity(self):
        return self.obj

    @property
    def index(self):
        return self.tl.items.index(self.obj)

    @property
    def parentGroup(self):
        return next((g for g in self.tl.groups if self.obj in g.members), None)

    def rollTo(self, rollBefore):
        i = self.tl.items.index(self.obj)
        self.tl.markerPosition = i if rollBefore else i + 1


class Group:
    objectType = "adsk::fusion::TimelineGroup"

    def __init__(self, tl, i0, i1):
        self.tl, self.members, self.name = tl, tl.items[i0:i1 + 1], ""

    @property
    def count(self):
        return len(self.members)

    def item(self, k):
        return TimelineObject(self.tl, self.members[k])

    def deleteMe(self, delete_contents):
        assert delete_contents is False
        self.tl.groups.remove(self)
        return True


class FeatureOperations:
    CutFeatureOperation, JoinFeatureOperation, NewBodyFeatureOperation = "cut", "join", "new"


class ExtentDirections:
    PositiveExtentDirection, NegativeExtentDirection = "pos", "neg"


class DesignTypes:
    ParametricDesignType, DirectDesignType = "parametric", "direct"


class DistExt:
    def __init__(self, expr):
        self.distance = mock.Mock(expression=expr)


class AllExt:
    pass


class ToExt:
    def __init__(self, ent, chained, offset):
        self.entity, self.offset = ent, mock.Mock(expression=offset)


class ExtrudeInput:
    def __init__(self, profiles, op):
        self.profiles, self.operation = list(profiles), op
        self.extent = self.direction = None
        self.participantBodies = []

    def setOneSideExtent(self, extent, direction):
        self.extent, self.direction = extent, direction


class Extrude(Entity):
    objectType = "adsk::fusion::ExtrudeFeature"

    def __init__(self, comp, inp):
        super().__init__()
        self.tl = comp.timeline
        self.name = ""
        self._profile, self.operation = inp.profiles, inp.operation
        self.extentOne, self.direction = inp.extent, inp.direction
        self.participantBodies = list(inp.participantBodies)
        self.edits = 0
        comp.timeline.insert(self)
        self.timelineObject = TimelineObject(comp.timeline, self)

    def _check(self):
        if self.tl.markerPosition != self.tl.items.index(self):
            raise RuntimeError("the timeline marker must be just before the feature to edit it")

    @property
    def profile(self):
        return self._profile

    @profile.setter
    def profile(self, coll):
        self._check()
        self._profile = list(coll)
        self.edits += 1

    def setOneSideExtent(self, extent, direction):
        self._check()
        self.extentOne, self.direction = extent, direction

    def deleteMe(self):
        super().deleteMe()
        self.tl.remove(self)

    def _level(self):
        e = self.extentOne
        if isinstance(e, AllExt):
            return None
        d = _parse(e.distance.expression) if isinstance(e, DistExt) else 0.3
        return d if self.direction == "pos" else -d

    def _edges(self, z):
        sk = self._profile[0].parentSketch
        out = []
        for c in sk.curves:
            if c.isValid:
                out.append(Edge(sk.sketchToModelSpace(Pt(*c.point_on(), z)), c))
        return out

    @property
    def endFaces(self):
        z = self._level()
        if z is None:
            return Items([])
        sk = self._profile[0].parentSketch
        return Items([Face(edges=Items(self._edges(z)), pointOnFace=sk.sketchToModelSpace(Pt(0, 0, z)))])

    @property
    def sideFaces(self):
        z = self._level() or 0.5
        walls = [Edge(Pt(e.pointOnEdge.x, e.pointOnEdge.y, z / 2), e.curve) for e in self._edges(0.0)]
        return Items([Face(edges=Items(self._edges(0.0) + walls))])


class Edge(Entity):
    objectType = "adsk::fusion::BRepEdge"

    def __init__(self, point, curve):
        super().__init__()
        self.pointOnEdge, self.curve = point, curve


class PlaneInput:
    def setByDistanceOnPath(self, ent, distance):
        self.ent = ent


class Plane(Entity):
    """A construction plane across a path's start: its sketches have x across the path (in the
    path's plane), y along the model's z, and the path going out of them."""
    objectType = "adsk::fusion::ConstructionPlane"

    def __init__(self, comp, pin):
        super().__init__()
        ev = pin.ent.worldGeometry.evaluator
        _, p = ev.getPointAtParameter(0.0)
        _, d = ev.getFirstDerivative(0.0)
        t = _unit3(v3(d))
        self.frame = (v3(p), _unit3(_cross3((0.0, 0.0, 1.0), t)), (0.0, 0.0, 1.0), t)
        self.path, self.name, self.isLightBulbOn = pin.ent, "", True
        comp.timeline.insert(self)
        self.timelineObject = TimelineObject(comp.timeline, self)

    def deleteMe(self):
        super().deleteMe()
        self.timelineObject.tl.remove(self)


class TimelineFeature(Entity):
    """Any other feature: in the timeline, with a name."""

    def __init__(self, comp, **kw):
        super().__init__()
        self.name = ""
        self.__dict__.update(kw)
        comp.timeline.insert(self)
        self.timelineObject = TimelineObject(comp.timeline, self)

    def deleteMe(self):
        super().deleteMe()
        self.timelineObject.tl.remove(self)


class FakePath:
    """A sweep path: exactly the curves given (a collection), or, chaining from one curve, what
    comp.chain_from(curve) returns (all its connected curves by default)."""

    def __init__(self, comp, ents, chain):
        if isinstance(ents, list):
            self.entities = list(ents)
        else:
            self.entities = comp.chain_from(ents) if chain else [ents]

    @property
    def count(self):
        return len(self.entities)

    @property
    def isClosed(self):
        ends = {}
        for c in self.entities:
            if c.startSketchPoint is None:  # a closed curve on its own (circle, closed spline)
                return len(self.entities) == 1
            for p in (c.startSketchPoint, c.endSketchPoint):
                ends[id(p)] = ends.get(id(p), 0) + 1
        return bool(ends) and all(n == 2 for n in ends.values())


class Sweep(TimelineFeature):
    objectType = "adsk::fusion::SweepFeature"

    def __init__(self, comp, si):
        ents = si.path.entities
        ent = ents[0]
        if comp.sweep_fails(ent):
            raise RuntimeError("sweep failed")
        psk = si.profile.parentSketch
        ring = [q for c in ents for q in c.model_points()]  # exactly what the path covers
        cross = [v3(psk.sketchToModelSpace(Pt(*q, 0.0))) for c in psk.curves if c.isValid for q in c.polyline()]
        o = psk.frame[0]
        reach = max(math.hypot(q[0] - o[0], q[1] - o[1]) for q in cross)
        lo = (min(q[0] for q in ring) - reach, min(q[1] for q in ring) - reach, min(q[2] for q in cross))
        hi = (max(q[0] for q in ring) + reach, max(q[1] for q in ring) + reach, max(q[2] for q in cross))
        super().__init__(comp, input=si, bodies=Items([Body(Box(lo, hi))]))


class TBM:
    def __init__(self):
        self.copies = 0

    def copy(self, body):
        self.copies += 1
        return Body(body.boundingBox)

    def transform(self, body, m):
        body.boundingBox = body.boundingBox.moved(m.translation)
        return True


class FilletInput:
    def __init__(self):
        self.sets = []
        self.isRollingBallCorner = False
        fi = self
        self.edgeSetInputs = mock.Mock()

        def add(coll, vi, chain):
            es = types.SimpleNamespace(edges=list(coll), radius=vi, continuity=None)
            fi.sets.append(es)
            return es
        self.edgeSetInputs.addConstantRadiusEdgeSet.side_effect = add


class Features:
    def __init__(self, comp):
        self.made = []
        f = self
        self.extrudeFeatures = mock.Mock()
        self.extrudeFeatures.createInput.side_effect = lambda coll, op: ExtrudeInput(coll, op)

        def add(inp):
            e = Extrude(comp, inp)
            f.made.append(e)
            return e
        self.extrudeFeatures.add.side_effect = add
        self.createPath = lambda ents, chain: FakePath(comp, ents, chain)
        self.sweepFeatures = mock.Mock()
        self.sweepFeatures.createInput.side_effect = \
            lambda prof, path, op: types.SimpleNamespace(profile=prof, path=path, operation=op, orientation=None)
        self.sweepFeatures.add.side_effect = lambda si: f.made.append(Sweep(comp, si)) or f.made[-1]
        self.baseFeatures = mock.Mock()
        self.baseFeatures.add.side_effect = \
            lambda: f.made.append(TimelineFeature(comp, bodies=Items([]), startEdit=lambda: True,
                                                  finishEdit=lambda: True, kind="base")) or f.made[-1]
        self.combineFeatures = mock.Mock()
        self.combineFeatures.createInput.side_effect = lambda target, coll: types.SimpleNamespace(
            target=target, tools=list(coll), operation=None, isKeepToolBodies=True)
        self.combineFeatures.add.side_effect = \
            lambda ci: f.made.append(TimelineFeature(comp, input=ci, kind="combine")) or f.made[-1]
        self.filletFeatures = mock.Mock()
        self.filletFeatures.createInput.side_effect = FilletInput
        self.filletFeatures.add.side_effect = \
            lambda fi: f.made.append(TimelineFeature(comp, input=fi, kind="fillet")) or f.made[-1]

    def of(self, cls, kind=None):
        return [x for x in self.made if isinstance(x, cls) and x.isValid and (kind is None or getattr(x, "kind", None) == kind)]


class Component:
    def __init__(self, arcs_take_points):
        self.made = []
        self.timeline = Timeline()
        self.features = Features(self)
        comp = self

        def add(plane):
            sk = Sketch("Sketch%d" % (len(comp.made) + 2), comp, arcs_take_points, offset=(0.0, 0.0),
                        frame=getattr(plane, "frame", None))
            (comp.profile_sketches if isinstance(plane, Plane) else comp.made).append(sk)
            sk.plane_from = plane
            return sk
        self.profile_sketches = []
        self.sketches = mock.Mock()
        self.sketches.addWithoutEdges.side_effect = add
        self.sketches.add.side_effect = add
        self.constructionPlanes = mock.Mock()
        self.constructionPlanes.createInput.side_effect = PlaneInput
        self.constructionPlanes.add.side_effect = lambda pin: Plane(comp, pin)
        self.bRepBodies = mock.Mock()
        self.bRepBodies.add.side_effect = lambda body, bf: bf.bodies.items.append(body) or body
        self.sweep_fails = lambda ent: False
        # chaining from one curve (createPath(curve, True)); tests can make it stop short, the way
        # real Fusion did with polygrid outlines (it swept just one corner)
        self.chain_from = lambda ent: ent.parentSketch.chain(ent)


def _cast(cls):
    return lambda x: x if isinstance(x, cls) else None


def fake_fusion(stack):
    """Point the mocked adsk modules at these fakes, for one test (other test files share them)."""
    for obj, name, val in (
            (adsk.fusion.Profile, "cast", _cast((Profile, GridProfile))), (adsk.fusion.Sketch, "cast", _cast(Sketch)),
            (adsk.fusion.SketchCurve, "cast", _cast(SketchCurve)), (adsk.fusion.SketchPoint, "cast", _cast(SketchPoint)),
            (adsk.fusion.SketchLine, "cast", _cast(FakeLine)), (adsk.fusion.ExtrudeFeature, "cast", _cast(Extrude)),
            (adsk.fusion.TimelineGroup, "cast", _cast(Group)), (adsk.fusion, "FeatureOperations", FeatureOperations),
            (adsk.fusion, "ExtentDirections", ExtentDirections), (adsk.fusion, "DesignTypes", DesignTypes),
            (adsk.fusion.DistanceExtentDefinition, "create", DistExt),
            (adsk.fusion.DistanceExtentDefinition, "cast", _cast(DistExt)),
            (adsk.fusion.ThroughAllExtentDefinition, "create", AllExt),
            (adsk.fusion.ThroughAllExtentDefinition, "cast", _cast(AllExt)),
            (adsk.fusion.ToEntityExtentDefinition, "create", ToExt),
            (adsk.fusion.ToEntityExtentDefinition, "cast", _cast(ToExt)),
            (adsk.core, "Curve3DTypes", TYPES), (adsk.core.Point3D, "create", lambda x, y, z: Pt(x, y, z)),
            (adsk.core.ObjectCollection, "create", Collection), (adsk.core.ValueInput, "createByString", lambda s: s),
            (adsk.fusion.TemporaryBRepManager, "get", TBM),
            (adsk.core.Matrix3D, "create", lambda: types.SimpleNamespace(translation=None)),
            (adsk.core.Vector3D, "create", lambda x, y, z: (x, y, z)),
            (adsk.fusion, "SweepOrientationTypes", types.SimpleNamespace(PerpendicularOrientationType="perp")),
            (adsk.fusion, "SurfaceContinuityTypes", types.SimpleNamespace(CurvatureSurfaceContinuityType="G2")),
            (fg, "_arc_by_points", [True])):
        stack.enter_context(mock.patch.object(obj, name, val))
    real_point = fg.profile_point
    stack.enter_context(mock.patch.object(
        fg, "profile_point", lambda p: p.pt if isinstance(p, GridProfile) else real_point(p)))


# --------------------------------------------------------------------------- command inputs
def _parse(expr):
    m = re.fullmatch(r"\s*([-+0-9.eE]+)\s*(mm|cm)?\s*", expr)
    if not m:
        return None
    return float(m.group(1)) * (0.1 if m.group(2) == "mm" else 1.0)


class Inp:
    objectType = "adsk::core::CommandInput"

    def __init__(self, reg, id_, name):
        self.id, self.name, self.isVisible, self.tooltip = id_, name, True, ""
        reg[id_] = self


class ValueInp(Inp):
    objectType = "adsk::core::ValueCommandInput"

    def __init__(self, reg, id_, name, expr):
        super().__init__(reg, id_, name)
        self.expression = expr

    @property
    def isValidExpression(self):
        return _parse(self.expression) is not None

    @property
    def value(self):
        return _parse(self.expression)

    @value.setter
    def value(self, v):
        self.expression = "%.12g cm" % v


class Sel(Inp):
    objectType = "adsk::core::SelectionCommandInput"

    def __init__(self, reg, id_, name):
        super().__init__(reg, id_, name)
        self.sel = []

    def addSelectionFilter(self, f):
        pass

    def setSelectionLimits(self, a, b=0):
        pass

    @property
    def selectionCount(self):
        return len(self.sel)

    def selection(self, k):
        return mock.Mock(entity=self.sel[k])

    def addSelection(self, e):
        self.sel.append(e)
        return True

    def clearSelection(self):
        self.sel = []


class ListItem:
    def __init__(self, dd, name, sel):
        self.dd, self.name, self._sel = dd, name, sel

    @property
    def isSelected(self):
        return self._sel

    @isSelected.setter
    def isSelected(self, v):
        if v:
            for it in self.dd.items:
                it._sel = False
        self._sel = v


class DD(Inp):
    objectType = "adsk::core::DropDownCommandInput"

    def __init__(self, reg, id_, name):
        super().__init__(reg, id_, name)
        self.items = []
        dd = self

        class LI:
            def add(self, name, sel, icon):
                dd.items.append(ListItem(dd, name, sel))

            @property
            def count(self):
                return len(dd.items)

            def item(self, k):
                return dd.items[k]

            def clear(self):
                dd.items.clear()
        self.listItems = LI()

    @property
    def selectedItem(self):
        return next((it for it in self.items if it._sel), None)

    def pick(self, name):
        next(it for it in self.items if it.name == name).isSelected = True


class Inputs:
    def __init__(self, reg=None):
        self.reg = {} if reg is None else reg

    def itemById(self, id_):
        return self.reg.get(id_)

    def addSelectionInput(self, id_, name, tip):
        return Sel(self.reg, id_, name)

    def addTextBoxCommandInput(self, id_, name, text, rows, ro):
        t = Inp(self.reg, id_, name)
        t.formattedText = text
        return t

    def addDropDownCommandInput(self, id_, name, style):
        return DD(self.reg, id_, name)

    def addBoolValueInput(self, id_, name, check, icon, value):
        b = Inp(self.reg, id_, name)
        b.value = value
        return b

    def addValueInput(self, id_, name, units, value):
        return ValueInp(self.reg, id_, name, value)

    def addIntegerSliderCommandInput(self, id_, name, lo, hi, two):
        s = Inp(self.reg, id_, name)
        s.valueOne = lo
        return s

    def addGroupCommandInput(self, id_, name):
        g = Inp(self.reg, id_, name)
        g.children = Inputs(self.reg)
        return g


# --------------------------------------------------------------------------- the harness
P = math.pi
ROUNDED = [("line", (1, 0), (9, 0)), ("arc", (9, 1), 1, -P / 2, P / 2), ("line", (10, 1), (10, 6)),
           ("line", (10, 6), (0, 6)), ("line", (0, 6), (0, 1)), ("arc", (1, 1), 1, P, P / 2)]
HOLE = [("arc", (3, 3), 1.0, 0.0, 2 * P)]
SPLINE_HOLE = [("curve", [(7 + 0.9 * math.cos(t), 3 + 0.6 * math.sin(t)) for t in [2 * P * i / 80 for i in range(81)]])]


class Harness:
    """One fake design with an area sketch ("Area": a rounded rectangle with a round hole and a
    spline hole) and a command module (sketch_polygrid or polygrid) to drive."""

    def __init__(self, stack, mod, arcs_take_points=True, offset=(0.0, 0.0)):
        self.mod = mod
        self.comp = Component(arcs_take_points)
        self.src = Sketch("Area", self.comp, arcs_take_points, offset)
        self.profile = Profile(self.src, [ROUNDED, HOLE, SPLINE_HOLE])
        self.src.given_profiles = [Profile(self.src, [[("line", (20, 0), (21, 0)), ("line", (21, 0), (21, 1)),
                                                        ("line", (21, 1), (20, 1)), ("line", (20, 1), (20, 0))]]),
                                   self.profile]
        # the same area as outline curves in the area's sketch
        self.outline = [SketchCurve(self.src, "outline", geometry=Curve3D(p),
                                    boundingBox=box_of([xy(Curve3D(p).startPoint), xy(Curve3D(p).endPoint)])
                                    if p[0] == "line" else box_of([(0, 0), (10, 6)]))
                        for p in ROUNDED + HOLE]
        self.body = Body()
        self.settings, self.graphics, self.handlers = {}, {}, []
        self.design = mock.Mock()
        self.design.timeline = self.comp.timeline
        self.design.designType = DesignTypes.ParametricDesignType
        self.comp.timeline.items.append("Extrude2")  # a feature after the area's sketch
        self.comp.timeline.moveToEnd()
        self.design.unitsManager.defaultLengthUnits = "mm"
        self.design.unitsManager.formatInternalValue.side_effect = lambda v, u, s: "%.4g mm" % (v * 10)
        self.design.findEntityByToken.side_effect = lambda t: [TOKENS[t]] if t in TOKENS else []
        for obj, name, val in (
                (adsk.fusion.Design, "cast", lambda x: self.design),
                (gx_comb, "get_setting", lambda name, default: self.settings.get(name, default)),
                (gx_comb, "set_setting", lambda name, value: self.settings.__setitem__(name, value)),
                (comb_graphics, "draw_segments", lambda key, coords, rgb, w: self.graphics.__setitem__(key, coords)),
                (comb_graphics, "clear", lambda key: self.graphics.pop(key, None)),
                (futil, "add_handler", lambda event, cb, cls: self.handlers.append(cb))):
            stack.enter_context(mock.patch.object(obj, name, val))

    @property
    def dlg(self):
        return self.mod._dlg

    def open(self, preselected=()):
        futil.ui.activeSelections.count = len(preselected)
        futil.ui.activeSelections.item.side_effect = lambda k: mock.Mock(entity=preselected[k])
        self.inputs = Inputs()
        args = mock.Mock()
        args.command.commandInputs = self.inputs
        self.mod._on_created(args)
        self.activate()
        return self.inputs

    def _args(self):
        args = mock.Mock()
        args.command.commandInputs = self.inputs
        args.firingEvent.sender.commandInputs = self.inputs
        return args

    def activate(self):
        self.activate_args = self._args()
        self.dlg.on_activate(self.activate_args)

    def changed(self, id_):
        args = self._args()
        args.input = self.inputs.itemById(id_)
        self.dlg.on_input_changed(args)

    def validate(self):
        args = self._args()
        self.dlg.on_validate(args)
        return args.areInputsValid

    def preview(self):
        self.dlg.on_preview(self._args())

    def execute(self):
        args = self._args()
        self.dlg.on_execute(args)
        args.terminationReason = "done"
        self.dlg.on_destroy(args)

    def cancel(self):
        args = self._args()
        args.terminationReason = "cancelled"
        self.dlg.on_destroy(args)


def check_closed(test, sk):
    """Every curve end is shared by exactly two curves (all loops closed, ends joined)."""
    uses = {}
    for c in sk.curves:
        if not c.isValid or c.kind == "circle" or getattr(c, "isClosed", False):
            continue
        for sp in (c.startSketchPoint, c.endSketchPoint):
            test.assertTrue(sp.isValid)
            uses[id(sp)] = uses.get(id(sp), 0) + 1
    test.assertTrue(uses)
    test.assertEqual(set(uses.values()), {2})
