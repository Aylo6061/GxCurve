"""The Lip command against fake Fusion objects: a shelled box split at z = 2 into a bottom body (the lip
body) and a top body (the groove body). Checks the cross-sections' placement, the inner / outer reveal,
the features made (two sweeps and two Combines per run, grouped), and the dialog's checks. No Fusion
needed. Run: python -m unittest discover tests
"""

import contextlib
import json
import math
import os
import sys
import types
import unittest
import unittest.mock as mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_fakes as F  # noqa: E402

import adsk.core  # noqa: E402
import adsk.fusion  # noqa: E402

from GX_tool.commands import gx_comb  # noqa: E402
from GX_tool.commands import lip as cmd  # noqa: E402
from GX_tool.lib import futil  # noqa: E402

INSIDE, ON, OUTSIDE = 0, 1, 2
X, Y, H, T, Z = 6.0, 4.0, 2.0, 0.2, 3.0  # box size, parting height, wall, top
EPS = 1e-9


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return tuple(x / n for x in a)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# --------------------------------------------------------------------------- B-rep
class PlaneGeo:
    def __init__(self, origin, normal):
        self.origin, self.normal = F.Pt(*origin), F.Pt(*normal)


class FaceEval:
    def __init__(self, face):
        self.face = face

    def getNormalAtPoint(self, p):
        return True, F.Pt(*self.face.reported_normal)

    def getParameterAtPoint(self, p):
        o, n = F.v3(self.face.geometry.origin), F.v3(self.face.geometry.normal)
        d = sum(a * b for a, b in zip(sub(F.v3(p), o), n))
        return True, F.Pt(*[a - d * b for a, b in zip(F.v3(p), n)])  # "parameters": the projected point

    def isParameterOnFace(self, uv):
        return self.face.contains(F.v3(uv))

    def getPointAtParameter(self, uv):
        return True, uv


class BFace(F.Entity):
    """A flat face: a box region of its plane (less a hole box), with loops of edges."""
    objectType = "adsk::fusion::BRepFace"

    def __init__(self, origin, normal, lo, hi, hole=None, flip_normal=False):
        super().__init__()
        self.geometry = PlaneGeo(origin, normal)
        self.reported_normal = tuple(-c for c in normal) if flip_normal else normal  # not always outward in Fusion
        self.lo, self.hi, self.hole = lo, hi, hole
        self.evaluator = FaceEval(self)
        self.loops = F.Items([])

    def contains(self, p):
        if not all(self.lo[k] - EPS <= p[k] <= self.hi[k] + EPS for k in range(3)):
            return False
        return not (self.hole and all(self.hole[0][k] < p[k] < self.hole[1][k] for k in range(3)))


class Loop:
    def __init__(self, edges, outer):
        self.edges, self.isOuter = F.Items(edges), outer


class EdgeEval:
    def __init__(self, a, b):
        self.a, self.b = a, b

    def getParameterExtents(self):
        return True, 0.0, 1.0

    def getPointAtParameter(self, t):
        return True, F.Pt(*[x + (y - x) * t for x, y in zip(self.a, self.b)])

    def getFirstDerivative(self, t):
        return True, F.Pt(*sub(self.b, self.a))


class BEdge(F.Entity):
    objectType = "adsk::fusion::BRepEdge"

    def __init__(self, a, b, body):
        super().__init__()
        self.a, self.b, self.body = a, b, body
        self.evaluator = EdgeEval(a, b)
        self.faces = F.Items([])


class BBody(F.Entity):
    objectType = "adsk::fusion::BRepBody"

    def __init__(self, name, comp, solid, box):
        super().__init__()
        self.name, self.parentComponent, self.solid = name, comp, solid
        self.boundingBox = F.Box(*box)

    def pointContainment(self, p):
        return INSIDE if self.solid(F.v3(p)) else OUTSIDE


def ring(z0, z1, cav0, cav1):
    """Inside a box wall: the outer box between heights z0 and z1, less the cavity between cav0 and cav1."""
    def solid(p):
        x, y, z = p
        if not (0 < x < X and 0 < y < Y and z0 < z < z1):
            return False
        return not (T < x < X - T and T < y < Y - T and cav0 < z < cav1)
    return solid


def rect(z, a, b):
    (x0, y0), (x1, y1) = a, b
    return [((x0, y0, z), (x1, y0, z)), ((x1, y0, z), (x1, y1, z)), ((x1, y1, z), (x0, y1, z)), ((x0, y1, z), (x0, y0, z))]


def rim(body, up, flip_normal=False):
    """The body's parting face at z = H (outward normal up: +1 for the bottom body, -1 for the top) and
    the edges around it, each with its wall face. Returns (face, outer edges, inner edges)."""
    face = BFace((0, 0, H), (0, 0, up), (0, 0, H), (X, Y, H), hole=((T, T, H - 1), (X - T, Y - T, H + 1)),
                 flip_normal=flip_normal)
    z0, z1 = (0.0, H) if up > 0 else (H, Z)
    loops = []
    for (a, b), outer in ((((0, 0), (X, Y))), True), ((((T, T), (X - T, Y - T))), False):
        edges = []
        for p, q in rect(H, a, b):
            e = BEdge(p, q, body)
            d = unit(sub(q, p))
            n = cross(d, (0, 0, 1))  # the wall's outward normal: right of the outer loop's direction (CCW)
            if not outer:
                n = tuple(-c for c in n)  # the inner walls face into the cavity
            lo = tuple(min(p[k], q[k]) for k in range(2)) + (z0,)
            hi = tuple(max(p[k], q[k]) for k in range(2)) + (z1,)
            wall = BFace(p, n, lo, hi)
            e.faces = F.Items([wall, face] if len(edges) % 2 else [face, wall])
            edges.append(e)
        loops.append(Loop(edges, outer))
    face.loops = F.Items(loops)
    return face, loops[0].edges.items, loops[1].edges.items


# --------------------------------------------------------------------------- features
class PlaneInput:
    def setByDistanceOnPath(self, ent, d):
        self.ent, self.d = ent, d


class CPlane(F.Entity):
    objectType = "adsk::fusion::ConstructionPlane"

    def __init__(self, comp, pin):
        super().__init__()
        e = pin.ent
        d = 1.0 - pin.d if comp.reversed_distance else pin.d
        p = tuple(x + (y - x) * d for x, y in zip(e.a, e.b))
        t = unit(sub(e.b, e.a))
        xa = unit(cross((0.0, 0.0, 1.0), t)) if abs(t[2]) < 0.9 else (1.0, 0.0, 0.0)
        self.frame = (p, xa, cross(t, xa), t)
        self.geometry = PlaneGeo(p, t)
        self.edge, self.d, self.name, self.isLightBulbOn = e, pin.d, "", True
        comp.timeline.insert(self)
        self.timelineObject = F.TimelineObject(comp.timeline, self)

    def deleteMe(self):
        super().deleteMe()
        self.timelineObject.tl.remove(self)


class Path:
    def __init__(self, ents):
        self.entities = list(ents)

    @property
    def count(self):
        return len(self.entities)

    def item(self, k):
        """Like Fusion's: the path runs from a free end of a chain; an entity running against it is opposed."""
        ents = self.entities
        if len(ents) == 1:
            return types.SimpleNamespace(entity=ents[0], isOpposedToEntity=False)
        nxt = ents[k + 1] if k + 1 < len(ents) else None
        e = ents[k]
        opposed = nxt is not None and not any(math.dist(e.b, q) < 1e-9 for q in (nxt.a, nxt.b))
        return types.SimpleNamespace(entity=e, isOpposedToEntity=opposed)


class Sweep(F.TimelineFeature):
    objectType = "adsk::fusion::SweepFeature"


class Comp:
    def __init__(self):
        self.timeline = F.Timeline()
        self.made, self.sketches_made, self.planes = [], [], []
        self.reversed_distance = False
        self.sweep_fails = False
        comp = self

        def sketch(plane):
            sk = F.Sketch("Sketch%d" % (len(comp.sketches_made) + 1), comp, frame=plane.frame)
            comp.sketches_made.append(sk)
            return sk
        self.sketches = mock.Mock()
        self.sketches.addWithoutEdges.side_effect = sketch
        self.sketches.add.side_effect = sketch

        def plane(pin):
            p = CPlane(comp, pin)
            comp.planes.append(p)
            return p
        self.constructionPlanes = mock.Mock()
        self.constructionPlanes.createInput.side_effect = PlaneInput
        self.constructionPlanes.add.side_effect = plane

        def sweep(si):
            if comp.sweep_fails:
                raise RuntimeError("sweep failed")
            sw = Sweep(comp, input=si, bodies=F.Items([BBody("tool", comp, lambda p: False, ((0, 0, 0), (1, 1, 1)))]))
            comp.made.append(sw)
            return sw

        def combine(ci):
            c = F.TimelineFeature(comp, input=ci, kind="combine")
            comp.made.append(c)
            return c
        self.features = types.SimpleNamespace(
            createPath=lambda coll, chain: Path(coll),
            sweepFeatures=mock.Mock(**{
                "createInput.side_effect": lambda prof, path, op: types.SimpleNamespace(
                    profile=prof, path=path, operation=op, orientation=None),
                "add.side_effect": sweep}),
            combineFeatures=mock.Mock(**{
                "createInput.side_effect": lambda target, coll: types.SimpleNamespace(
                    target=target, tools=list(coll), operation=None, isKeepToolBodies=True),
                "add.side_effect": combine}))

    def of(self, cls, kind=None):
        return [x for x in self.made if isinstance(x, cls) and x.isValid and (kind is None or getattr(x, "kind", None) == kind)]


# --------------------------------------------------------------------------- the dialog's inputs
class AngleInp(F.ValueInp):
    @property
    def isValidExpression(self):
        return self.value is not None

    @property
    def value(self):
        try:
            return math.radians(float(self.expression.replace("deg", "")))
        except ValueError:
            return None


class Inputs(F.Inputs):
    def addValueInput(self, id_, name, units, value):
        return (AngleInp if units == "deg" else F.ValueInp)(self.reg, id_, name, value)


class Harness:
    def __init__(self, stack, flip_bottom_normal=False):
        self.comp = Comp()
        self.bottom = BBody("bottom", self.comp, ring(0.0, H, T, H + 1), ((0, 0, 0), (X, Y, H)))
        self.top = BBody("top", self.comp, ring(H, Z, H - 1, Z - T), ((0, 0, H), (X, Y, Z)))
        self.other = BBody("other", self.comp, lambda p: False, ((10, 10, -11), (11, 11, -10)))  # off below
        self.rim_b, self.outer_b, self.inner_b = rim(self.bottom, +1, flip_normal=flip_bottom_normal)
        self.rim_t, self.outer_t, self.inner_t = rim(self.top, -1)
        self.comp.bRepBodies = F.Items([self.other, self.bottom, self.top])
        self.settings, self.handlers = {}, []
        self.design = mock.Mock()
        self.design.timeline = self.comp.timeline
        self.design.designType = F.DesignTypes.ParametricDesignType
        self.comp.timeline.moveToEnd()
        self.design.unitsManager.defaultLengthUnits = "mm"
        self.design.unitsManager.formatInternalValue.side_effect = lambda v, u, s: "%.4g mm" % (v * 10)
        cast = F._cast
        for obj, name, val in (
                (adsk.fusion.Design, "cast", lambda x: self.design),
                (adsk.fusion.BRepEdge, "cast", cast(BEdge)),
                (adsk.core.Plane, "cast", cast(PlaneGeo)),
                (adsk.core.ValueInput, "createByReal", lambda v: v),
                (adsk.fusion, "PointContainment", types.SimpleNamespace(
                    PointInsidePointContainment=INSIDE, PointOnPointContainment=ON,
                    PointOutsidePointContainment=OUTSIDE)),
                (gx_comb, "get_setting", lambda name, default: self.settings.get(name, default)),
                (gx_comb, "set_setting", lambda name, value: self.settings.__setitem__(name, value)),
                (futil, "add_handler", lambda event, cb, cls: self.handlers.append(cb))):
            stack.enter_context(mock.patch.object(obj, name, val))

    @property
    def dlg(self):
        return cmd._dlg

    def open(self):
        self.inputs = Inputs()
        args = mock.Mock()
        args.command.commandInputs = self.inputs
        cmd._on_created(args)
        return self.inputs

    def set(self, **values):
        for id_, v in values.items():
            inp = self.inputs.itemById(id_)
            if id_ == "reveal":
                inp.pick(v)
            elif id_ == "wholeLoop":
                inp.value = v
            else:
                inp.expression = v
            self.changed(id_)

    def pick(self, id_, *ents):
        for e in ents:
            self.inputs.itemById(id_).addSelection(e)
        self.changed(id_)

    def _args(self):
        args = mock.Mock()
        args.command.commandInputs = self.inputs
        args.firingEvent.sender.commandInputs = self.inputs
        return args

    def changed(self, id_):
        args = self._args()
        args.input = self.inputs.itemById(id_)
        self.dlg.on_input_changed(args)

    def validate(self):
        args = self._args()
        self.dlg.on_validate(args)
        return args.areInputsValid

    def preview(self):
        args = self._args()
        args.isValidResult = False
        self.dlg.on_preview(args)
        return args.isValidResult

    def execute(self):
        args = self._args()
        args.executeFailed = False
        self.dlg.on_execute(args)
        return args

    @property
    def info(self):
        return self.inputs.itemById("info").formattedText


def model_points(sk):
    return [F.v3(sk.sketchToModelSpace(F.Pt(*q, 0.0))) for c in sk.curves if c.isValid for q in c.polyline()]


def span(pts, k):
    return min(p[k] for p in pts), max(p[k] for p in pts)


class Lip(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        F.fake_fusion(self.stack)

    def tearDown(self):
        self.stack.close()

    def harness(self, **kw):
        h = Harness(self.stack, **kw)
        h.open()
        h.set(height="2 mm", width="1 mm", offset="0 mm", draft="0 deg", reveal="Both sides", gap="0.1 mm",
              topGap="0 mm", wholeLoop=True)
        return h

    def made(self, h):
        sweeps = h.comp.of(Sweep)
        combines = h.comp.of(F.TimelineFeature, "combine")
        return sweeps, combines

    def sections(self, h):
        sk_lip, sk_groove = [s for s in h.comp.sketches_made if s.isValid][-2:]
        return model_points(sk_lip), model_points(sk_groove)

    def assertSpan(self, got, lo, hi):
        self.assertAlmostEqual(got[0], lo, places=9)
        self.assertAlmostEqual(got[1], hi, places=9)

    # ------------------------------------------------------------- picking
    def test_picking_an_edge_fills_in_the_bodies(self):
        h = self.harness()
        h.pick("edges", h.outer_b[0])
        self.assertEqual(h.inputs.itemById("lipBody").sel, [h.bottom])
        self.assertEqual(h.inputs.itemById("grooveBody").sel, [h.top])

    def test_picking_an_edge_of_the_top_fills_it_in_as_the_lip_body(self):
        h = self.harness()
        h.pick("edges", h.inner_t[2])
        self.assertEqual(h.inputs.itemById("lipBody").sel, [h.top])
        self.assertEqual(h.inputs.itemById("grooveBody").sel, [h.bottom])

    def test_bodies_already_picked_are_kept(self):
        h = self.harness()
        h.pick("grooveBody", h.bottom)
        h.pick("lipBody", h.top)
        h.pick("edges", h.outer_b[0])
        self.assertEqual(h.inputs.itemById("lipBody").sel, [h.top])
        self.assertEqual(h.inputs.itemById("grooveBody").sel, [h.bottom])

    # ------------------------------------------------------------- the result
    def test_lip_on_the_outer_loop(self):
        h = self.harness()
        h.pick("edges", h.outer_b[0])  # one edge: Whole loop takes all four
        self.assertTrue(h.validate(), h.info)
        self.assertTrue(h.preview(), h.info)
        sweeps, combines = self.made(h)
        self.assertEqual(len(sweeps), 2)
        groove_sw, lip_sw = sweeps
        for sw in sweeps:
            self.assertEqual(sw.input.operation, "new")
            self.assertEqual(sw.input.orientation, "perp")
            self.assertEqual(set(map(id, sw.input.path.entities)), set(map(id, h.outer_b)))
        self.assertEqual([s.name for s in sweeps], ["Lip groove", "Lip"])
        cut, join = combines
        self.assertEqual((cut.input.target, cut.input.operation, cut.input.isKeepToolBodies), (h.top, "cut", False))
        self.assertEqual(cut.input.tools, list(groove_sw.bodies.items))
        self.assertEqual((join.input.target, join.input.operation, join.input.isKeepToolBodies), (h.bottom, "join", False))
        self.assertEqual(join.input.tools, list(lip_sw.bodies.items))
        # the cross-sections, across the middle of the longest edge (y = 0, from x 0 to 6)
        plane = [p for p in h.comp.planes if p.isValid][0]
        self.assertEqual((plane.edge, plane.d), (h.outer_b[0], 0.5))
        lip, groove = self.sections(h)
        for p in lip + groove:
            self.assertAlmostEqual(p[0], 3.0, places=9)
        self.assertSpan(span(lip, 1), 0.0, 0.1)        # flush with the outer wall, 1 mm into the face
        self.assertSpan(span(lip, 2), H, H + 0.2)      # standing 2 mm up from the parting face
        self.assertSpan(span(groove, 1), -0.01, 0.11)  # the reveal on both sides
        self.assertSpan(span(groove, 2), H - 0.05, H + 0.2)
        # everything is grouped, hidden, and remembers its settings
        tl = h.comp.timeline
        self.assertEqual(len(tl.groups), 1)
        self.assertEqual(tl.groups[0].name, "Lip")
        self.assertEqual(len(tl.groups[0].members), 7)   # plane, 2 sketches, 2 sweeps, 2 combines
        self.assertFalse(plane.isLightBulbOn)
        self.assertTrue(all(not s.isVisible for s in h.comp.sketches_made))
        data = json.loads(join.attributes.itemByName("GX_tool", cmd.SETTINGS_ATTR).value)
        self.assertEqual((data["width"], data["reveal"]), ("1 mm", "both"))
        self.assertEqual(json.loads(h.settings[cmd.LAST_KEY]), data)
        self.assertIn("Loop 1: 4 edge(s)", h.info)

    def test_reveal_sides_follow_inside_and_outside(self):
        cases = (("Outer side", (-0.01, 0.1)), ("Inner side", (0.0, 0.11)), ("None", (0.0, 0.1)))
        for reveal, ys in cases:
            with self.subTest(reveal=reveal):
                h = self.harness()
                h.set(reveal=reveal)
                h.pick("edges", h.outer_b[0])
                self.assertTrue(h.preview(), h.info)
                self.assertSpan(span(self.sections(h)[1], 1), *ys)

    def test_lip_on_the_inner_loop_of_the_top_body(self):
        """Edges on a hole in the top body's parting face: the lip rises from the top body down into the
        bottom one, the face runs outward from the edge, and the inner side is the one at the edge."""
        h = self.harness()
        h.set(reveal="Outer side", draft="2 deg", topGap="0.5 mm")
        h.pick("edges", h.inner_t[0])  # y = T, from x = T to X - T
        self.assertEqual(h.inputs.itemById("lipBody").sel, [h.top])
        self.assertTrue(h.preview(), h.info)
        lip, groove = self.sections(h)
        k = 0.2 * math.tan(math.radians(2))
        self.assertSpan(span(lip, 1), T - 0.1, T)                # from the inner wall, outward
        self.assertSpan(span(lip, 2), H - 0.2, H)                # downward, into the bottom body
        # the reveal is on the outer side only (toward y = 0), square to the drafted side
        s = 1 / math.cos(math.radians(2))
        self.assertAlmostEqual(min(p[1] for p in groove), T - 0.1 - 0.01 * s - 0.05 * math.tan(math.radians(2)), places=9)
        self.assertAlmostEqual(max(p[1] for p in groove), T + 0.05 * math.tan(math.radians(2)), places=9)
        self.assertSpan(span(groove, 2), H - 0.25, H + 0.05)
        tops = sorted(p[1] for p in lip if abs(p[2] - (H - 0.2)) < 1e-9)
        self.assertAlmostEqual(tops[-1] - tops[0], 0.1 - 2 * k, places=9)  # the draft narrows the top
        cut, join = self.made(h)[1]
        self.assertEqual((cut.input.target, join.input.target), (h.bottom, h.top))

    def test_offset_moves_the_lip_across_the_face(self):
        h = self.harness()
        h.set(offset="0.5 mm", reveal="None")
        h.pick("edges", h.outer_b[0])
        self.assertTrue(h.preview(), h.info)
        self.assertSpan(span(self.sections(h)[0], 1), 0.05, 0.15)

    def test_normals_pointing_into_the_body_are_turned_round(self):
        h = Harness(self.stack, flip_bottom_normal=True)
        h.open()
        h.pick("edges", h.outer_b[0])
        self.assertTrue(h.preview(), h.info)
        self.assertSpan(span(self.sections(h)[0], 2), H, H + 0.2)

    def test_open_chain_starts_at_its_free_end(self):
        h = self.harness()
        h.set(wholeLoop=False)
        h.pick("edges", h.outer_b[1], h.outer_b[0])
        self.assertTrue(h.preview(), h.info)
        plane = [p for p in h.comp.planes if p.isValid][0]
        self.assertEqual((plane.edge, plane.d), (h.outer_b[1], 1.0))  # Fusion's path starts at its far end
        lip = self.sections(h)[0]
        for p in lip:
            self.assertAlmostEqual(p[1], Y, places=9)    # across the free end, at y = Y
        self.assertSpan(span(lip, 0), X - 0.1, X)        # the face runs inward (-x) from that edge
        self.assertIn("Chain 1: 2 edge(s) (open)", h.info)

    def test_plane_made_again_when_the_distance_runs_the_other_way(self):
        h = self.harness()
        h.comp.reversed_distance = True
        h.set(wholeLoop=False)
        h.pick("edges", h.outer_b[1], h.outer_b[0])
        self.assertTrue(h.preview(), h.info)
        planes = [p for p in h.comp.planes if p.isValid]
        self.assertEqual(len(planes), 1)
        self.assertEqual(len(h.comp.planes), 2)  # the first one went to the wrong end and was deleted
        self.assertAlmostEqual(planes[0].geometry.origin.y, Y, places=9)  # at the free end after all

    def test_two_loops(self):
        h = self.harness()
        h.pick("edges", h.outer_b[0], h.inner_b[3])
        self.assertTrue(h.preview(), h.info)
        sweeps, combines = self.made(h)
        self.assertEqual(len(sweeps), 4)
        self.assertEqual(len(combines), 2)
        self.assertEqual(len(combines[0].input.tools), 2)
        self.assertEqual(len(combines[1].input.tools), 2)

    # ------------------------------------------------------------- problems
    def test_lip_wider_than_the_face_warns(self):
        h = self.harness()
        h.set(width="3 mm")
        h.pick("edges", h.outer_b[0])
        self.assertTrue(h.preview(), h.info)
        self.assertIn("runs off the parting face", h.info)

    def test_quick_checks(self):
        for values, msg in ((dict(width="0 mm"), "width must be above zero"),
                            (dict(draft="30 deg"), "draft is too steep"),
                            (dict(gap="-1 mm"), "can't be negative"),
                            (dict(height="abc"), "isn't a valid value")):
            with self.subTest(values=values):
                h = self.harness()
                h.pick("edges", h.outer_b[0])
                h.set(**values)
                self.assertFalse(h.validate())
                self.assertIn(msg, h.info)
                self.assertTrue(h.preview() is False)
                self.assertEqual(self.made(h), ([], []))

    def test_same_body_twice(self):
        h = self.harness()
        h.pick("lipBody", h.bottom)
        h.pick("grooveBody", h.bottom)
        h.pick("edges", h.outer_b[0])
        self.assertFalse(h.validate())
        self.assertIn("must be different bodies", h.info)

    def test_edge_on_neither_body(self):
        h = self.harness()
        h.pick("lipBody", h.bottom)
        h.pick("grooveBody", h.other)
        h.pick("edges", h.inner_t[0])
        self.assertTrue(h.validate())
        self.assertFalse(h.preview())
        self.assertIn("isn't on the lip body or the groove body", h.info)
        args = h.execute()
        self.assertTrue(args.executeFailed)

    def test_parting_face_must_face_the_other_body(self):
        h = self.harness()
        h.pick("lipBody", h.bottom)
        h.pick("grooveBody", h.other)
        h.pick("edges", h.outer_b[0])
        self.assertFalse(h.preview())
        self.assertIn("faces the other body", h.info)

    def test_failed_sweep(self):
        h = self.harness()
        h.comp.sweep_fails = True
        h.pick("edges", h.outer_b[0])
        self.assertFalse(h.preview())
        self.assertIn("sweep failed", h.info)
        args = h.execute()
        self.assertTrue(args.executeFailed)
        self.assertIn("tighter than the lip reaches", args.executeFailedMessage)

    def test_execute_builds_when_the_preview_did_not(self):
        h = self.harness()
        h.pick("edges", h.outer_b[0])
        args = h.execute()
        self.assertFalse(args.executeFailed)
        self.assertEqual([len(x) for x in self.made(h)], [2, 2])
        self.assertIn(cmd.LAST_KEY, h.settings)

    def test_last_settings_come_back(self):
        h = self.harness()
        h.set(height="3 mm", reveal="Inner side")
        h.pick("edges", h.outer_b[0])
        self.assertTrue(h.preview())
        h.open()
        self.assertEqual(h.inputs.itemById("height").expression, "3 mm")
        self.assertEqual(h.inputs.itemById("reveal").selectedItem.name, "Inner side")


if __name__ == "__main__":
    unittest.main()
