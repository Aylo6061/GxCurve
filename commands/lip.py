"""'Lip' command (Solid > Create): a lip on one body and the groove it fits into on the other, along
the edges where the two bodies meet, as in a lip-and-groove joint between two halves of an enclosure.

Pick:
  - Edges: the edges along the parting line, on either body, where a parting face (the flat face
    where the two bodies meet) runs into a wall. With Whole loop on, one edge stands for the whole
    loop of the parting face it's on.
  - Lip body and Groove body. Picking an edge fills in empty body boxes: the edge's body becomes the
    lip body, and the body sitting on its parting face becomes the groove body.
The lip's cross-section is a trapezoid standing on the parting face: Offset from edge across the
face from the picked edge, Lip width wide where it stands and Lip height tall, its sides leaning in
by the Draft angle. The groove's cross-section is the lip's grown by the reveal: a gap between the
lip and the groove's walls on the lip's inner side, outer side or both (square to the drafted
sides), plus the Top gap above the lip.

Both cross-sections are sketched on a plane across the start of the edges and swept along them
(one sweep each per loop or chain of edges) as new bodies. Then one Combine cuts the grooves from
the groove body and another joins the lips to the lip body; Combine keeps the two bodies apart,
where a joining sweep would merge them wherever the lip touches the groove. All of it is grouped in
the timeline as "Lip". The preview is the real result.

Limits: the parting faces under the picked edges must be flat and in one plane per loop, and both
bodies must be in the same component.
"""

import json
import math
import os

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib import lip as L
from ..lib.futil import ATTR_GROUP, ui
from . import gx_comb
from . import polygrid_dialog as D

CMD_ID = "GXTool_Lip"
PANEL = "SolidCreatePanel"
NAME = "Lip"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "lip")
REVEALS = {"None": L.NONE, "Inner side": L.INNER, "Outer side": L.OUTER, "Both sides": L.BOTH}
VALUES = ("height", "width", "offset", "draft", "gap", "topGap")
DEFAULTS = dict(height="2 mm", width="1 mm", offset="0 mm", draft="0 deg", reveal=L.BOTH, gap="0.1 mm",
                topGap="0 mm", wholeLoop=True)
LAST_KEY = "lipLast"   # design setting: the last settings used
SETTINGS_ATTR = "lip"  # on the lip's Combine: its settings (JSON)
EPS = 1e-3             # cm: how far off an edge or a face the probe points go
BELOW = 0.25           # the groove's cut reaches this share of the lip height below the parting face

_dlg = None  # the open dialog


def _P(a):
    return adsk.core.Point3D.create(a[0], a[1], a[2])


def _v(p):
    return (p.x, p.y, p.z)


def _items(coll):
    return [coll.item(k) for k in range(coll.count)]


def native(e):
    """The entity in its own component (selections in an assembly context are proxies)."""
    try:
        return e.nativeObject or e
    except Exception:
        return e


def _same(a, b):
    try:
        return a == b
    except Exception:
        return a is b


def _on_face(face, p, tol=1e-5):
    ev = face.evaluator
    ok, uv = ev.getParameterAtPoint(_P(p))
    if not ok or not ev.isParameterOnFace(uv):
        return False
    ok, q = ev.getPointAtParameter(uv)  # the projection must be the point itself
    return ok and math.dist(_v(q), p) < tol


def _inside(body, p):
    """Is p inside the body (or on its surface)?"""
    C = adsk.fusion.PointContainment
    return body.pointContainment(_P(p)) in (C.PointInsidePointContainment, C.PointOnPointContainment)


def _center(body):
    bb = body.boundingBox
    return L.mul(L.add(_v(bb.minPoint), _v(bb.maxPoint)), 0.5)


# --------------------------------------------------------------------------- edges
class EdgeInfo:
    """A picked edge and the parting face it bounds: the flat face of its body that faces the other
    body. own: the edge's body; other: the body across the parting face (None: not known yet)."""

    def __init__(self, edge, own, other=None):
        self.edge, self.own = edge, own
        self.ev = edge.evaluator
        _, self.t0, self.t1 = self.ev.getParameterExtents()
        self.a, self.b = self.point(self.t0), self.point(self.t1)
        self.tm = 0.5 * (self.t0 + self.t1)
        self.mid = self.point(self.tm)
        pts = [self.point(self.t0 + (self.t1 - self.t0) * i / 8) for i in range(9)]
        self.length = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
        best = None
        for face, q, n, s in self.probes():
            if other is None:
                score = (0, 0.0)
            else:  # touching the other body beats facing toward it
                toward = L.dot(n, L.sub(_center(other), q))
                score = (2 if _inside(other, L.add(q, L.mul(n, EPS))) else 1 if toward > 0 else 0, toward)
            if best is None or score > best[0]:
                best = (score, face, n, s)
        if best is None or (other is not None and best[0][0] == 0):
            raise ValueError("it doesn't run along a flat face that faces the other body")
        _, self.face, self.n_out, self.s = best

    def point(self, t):
        return _v(self.ev.getPointAtParameter(t)[1])

    def tangent(self, t):
        return L.unit(_v(self.ev.getFirstDerivative(t)[1]))

    def probes(self):
        """(face, point on the face just off the edge's middle, the face's outward normal, the sign of
        n x tangent pointing into the face) for each flat face along the edge."""
        tan = self.tangent(self.tm)
        for face in _items(self.edge.faces):
            if not adsk.core.Plane.cast(face.geometry):
                continue
            ok, nrm = face.evaluator.getNormalAtPoint(_P(self.mid))
            if not ok:
                continue
            n = L.unit(_v(nrm))
            side = L.unit(L.cross(n, tan))
            s = next((s for s in (1, -1) if _on_face(face, L.add(self.mid, L.mul(side, s * EPS)))), None)
            if s is None:
                continue
            q = L.add(self.mid, L.mul(side, s * EPS))
            if _inside(self.own, L.add(q, L.mul(n, EPS))):  # make the normal point out of the body
                n, s = L.mul(n, -1.0), -s
            yield face, q, n, s

    def across(self, t):
        """Unit direction across the parting face, square to the edge at parameter t, into the face."""
        return L.mul(L.unit(L.cross(self.n_out, self.tangent(t))), self.s)

    def loop(self):
        """(the edges of the parting face's loop this edge is on, whether it's the face's outer loop)."""
        for lp in _items(self.face.loops):
            edges = _items(lp.edges)
            if any(_same(e, self.edge) for e in edges):
                return edges, bool(lp.isOuter)
        return [self.edge], True


def touching_body(edge, bodies):
    """The body, of these, that sits on a flat face along the edge (its other side), or None."""
    edge = native(edge)
    own = native(edge.body)
    try:
        probes = list(EdgeInfo(edge, own).probes())
    except Exception as e:
        futil.debug("lip: couldn't probe the edge's faces: %s" % e)
        return None
    for b in bodies:
        if _same(b, own):
            continue
        if any(_inside(b, L.add(q, L.mul(n, EPS))) for _, q, n, _ in probes):
            return b
    return None


class Chain:
    """Picked edges end to end (a loop or an open chain), where the lip runs."""

    def __init__(self, infos, order, closed):
        self.infos = [infos[i] for i, _ in order]
        self.order = [(infos[i], rev) for i, rev in order]
        self.closed = closed
        self.edges = [x.edge for x in self.infos]


# --------------------------------------------------------------------------- the plan
def sizes(inp):
    """The dialog's sizes: lengths in cm, the draft in radians."""
    return dict(offset=inp["offset"].value, width=inp["width"].value, height=inp["height"].value,
                draft=inp["draft"].value, gap=inp["gap"].value if inp["gap"].isVisible else 0.0,
                top=inp["topGap"].value)


class Plan:
    """What the Lip will make, from the dialog: chains of edges with their frames, the cross-sections,
    and what's wrong (errors) or doubtful (warnings)."""

    def __init__(self, edges, lip, groove, cfg, whole_loop):
        self.lip, self.groove, self.cfg = lip, groove, cfg
        self.errors, self.warnings, self.chains = [], [], []
        infos = []
        for k, e in enumerate(edges):
            try:
                infos.append(self.info(e))
            except ValueError as ex:
                self.errors.append("Edge %d: %s." % (k + 1, ex))
        if self.errors:
            return
        if whole_loop:
            for x in list(infos):
                for e in x.loop()[0]:
                    if not any(_same(e, y.edge) for y in infos):
                        try:
                            infos.append(self.info(e))
                        except ValueError as ex:
                            futil.debug("lip: a loop edge was left out (%s)" % ex)
        try:
            found = L.chains([(x.a, x.b) for x in infos], tol=1e-5)
        except ValueError as ex:
            self.errors.append(str(ex))
            return
        for ch in found:
            c = Chain(infos, ch["edges"], ch["closed"])
            up0, p0 = self.up(c.infos[0]), c.infos[0].mid
            if any(L.dot(self.up(x), up0) < 1 - 1e-6 or abs(L.dot(L.sub(x.mid, p0), up0)) > 1e-4 for x in c.infos):
                self.errors.append("The parting faces along these edges aren't all in one plane. The lip needs a "
                                   "flat parting face.")
                return
            self.chains.append(c)
        self.check_fit()

    def info(self, e):
        e = native(e)
        own = native(e.body)
        if _same(own, self.lip):
            return EdgeInfo(e, own, self.groove)
        if _same(own, self.groove):
            return EdgeInfo(e, own, self.lip)
        raise ValueError("it isn't on the lip body or the groove body")

    def up(self, x):
        """Out of the lip body, into the groove body."""
        return x.n_out if _same(x.own, self.lip) else L.mul(x.n_out, -1.0)

    def sides(self, ch):
        """(near gap, far gap) for this chain's lip."""
        _, outer = ch.infos[0].loop()
        return L.gaps(self.cfg["reveal"], self.cfg["gap"], inner_far=outer)

    def sections(self, ch):
        c = self.cfg
        near, far = self.sides(ch)
        lip = L.lip_section(c["offset"], c["width"], c["height"], c["draft"])
        groove = L.groove_section(c["offset"], c["width"], c["height"], c["draft"], near, far, c["top"],
                                  BELOW * c["height"])
        return lip, groove

    def check_fit(self):
        """Does the lip stand on the parting face? (Checked across the middle of each edge.)"""
        c = self.cfg
        for ch in self.chains:
            faces = []
            for x in ch.infos:
                if not any(_same(x.face, f) for f in faces):
                    faces.append(x.face)
            for x in ch.infos:
                w = x.across(x.tm)
                for u in (c["offset"] + 0.02 * c["width"], c["offset"] + 0.98 * c["width"]):
                    q = L.add(x.mid, L.mul(w, u))
                    if not any(_on_face(f, q) for f in faces):
                        self.warnings.append("The lip runs off the parting face: it's wider than the face there, or "
                                             "offset too far.")
                        return


# --------------------------------------------------------------------------- the features
def _collection(items):
    coll = adsk.core.ObjectCollection.create()
    for x in items:
        coll.add(x)
    return coll


def _path(comp, edges):
    try:
        return comp.features.createPath(_collection(edges), False)
    except Exception:
        return adsk.fusion.Path.create(_collection(edges), adsk.fusion.ChainedCurveOptions.noChainedCurves)


def _start(ch, path):
    """(EdgeInfo, proportional distance along its edge) where the cross-sections go. A loop: the
    middle of its longest edge (a sweep starting at a sharp corner can't close up neatly). An
    open chain: its start, where Fusion's path says, else the chain's own."""
    if ch.closed:
        return max(ch.infos, key=lambda x: x.length), 0.5
    try:
        pe = path.item(0)
        ent = native(pe.entity)
        for x in ch.infos:
            if _same(x.edge, ent):
                return x, 1.0 if pe.isOpposedToEntity else 0.0
    except Exception as e:
        futil.debug("lip: couldn't read the path's start (%s); using the chain's" % e)
    x, rev = ch.order[0]
    return x, 1.0 if rev else 0.0


def _crossing(plane, x, guess):
    """The parameter where edge x crosses the plane, found from guess, or None."""
    g = plane.geometry
    O, N = _v(g.origin), L.unit(_v(g.normal))
    t = guess
    for _ in range(40):
        f = L.dot(L.sub(x.point(t), O), N)
        if abs(f) < 1e-9:
            break
        df = L.dot(_v(x.ev.getFirstDerivative(t)[1]), N)
        if abs(df) < 1e-12:
            break
        t = min(max(t - f / df, x.t0), x.t1)
    return t if abs(L.dot(L.sub(x.point(t), O), N)) < 1e-5 else None


def _profile_plane(comp, x, d):
    """A construction plane across edge x, at proportional distance d along it. Returns (plane, the
    edge's parameter there). At an end, Fusion's distance may run the other way from the edge's
    evaluator: then the plane is made again from the other end."""
    planes = comp.constructionPlanes
    want = {0.0: x.t0, 1.0: x.t1}.get(d)
    for dd in ((d, 1.0 - d) if want is not None else (d,)):
        pin = planes.createInput()
        pin.setByDistanceOnPath(x.edge, adsk.core.ValueInput.createByReal(dd))
        plane = planes.add(pin)
        guess = x.tm if want is None else want
        t = _crossing(plane, x, guess)
        if t is not None and (want is None or math.dist(x.point(t), x.point(want)) < 1e-5):
            return plane, t
        plane.deleteMe()
    raise RuntimeError("couldn't put a plane across the edges")


def _section_sketch(comp, plane, name, f, section):
    sk = D.add_sketch(comp, plane, name)
    pts = [fusion_geom.p2(sk.modelToSketchSpace(_P(f(q)))) for q in section]
    sk.isComputeDeferred = True
    try:
        fusion_geom.draw_loop(sk, [("seg", pts[i], pts[(i + 1) % len(pts)], False) for i in range(len(pts))],
                              lambda q: q)
    finally:
        sk.isComputeDeferred = False
    if sk.profiles.count < 1:
        raise RuntimeError("the %s didn't make a closed profile" % name.lower())
    return sk


def _sweep(comp, sk, edges, name, closed):
    sweeps = comp.features.sweepFeatures
    si = sweeps.createInput(sk.profiles.item(0), _path(comp, edges),
                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    try:
        sw = sweeps.add(si)
    except Exception as e:
        hint = (" A corner of the edges is probably tighter than the lip reaches across the face (offset + width + "
                "gap): pick the edges on the other side of the parting face, round that corner more, or make the "
                "lip narrower." if closed else "")
        raise RuntimeError("the %s sweep failed (%s).%s" % (name.lower(), e, hint))
    sw.name = name
    return _items(sw.bodies)


def _combine(comp, target, tools, op, name):
    ci = comp.features.combineFeatures.createInput(target, _collection(tools))
    ci.operation = op
    ci.isKeepToolBodies = False
    c = comp.features.combineFeatures.add(ci)
    c.name = name
    return c


def build(plan):
    """Make the lip and groove features at the timeline marker. Returns the lip's Combine."""
    comp = plan.lip.parentComponent
    d = D.design()
    tl = d.timeline if d and d.designType == adsk.fusion.DesignTypes.ParametricDesignType else None
    start_idx = tl.markerPosition if tl is not None else None
    lips, grooves, planes, sketches = [], [], [], []
    for k, ch in enumerate(plan.chains):
        x, d = _start(ch, None if ch.closed else _path(comp, ch.edges))
        plane, t = _profile_plane(comp, x, d)
        plane.name = "Lip plane"
        P = x.point(t)
        f = L.frame_map(P, x.across(t), plan.up(x))
        lip, groove = plan.sections(ch)
        futil.debug("lip: chain %d, %d edge(s)%s, cross-sections at %s; lip %s; groove %s" % (
            k + 1, len(ch.edges), " (loop)" if ch.closed else "", futil.fmt_pt(_P(P)),
            [tuple(round(c, 5) for c in q) for q in lip], [tuple(round(c, 5) for c in q) for q in groove]))
        sk_lip = _section_sketch(comp, plane, "Lip profile", f, lip)
        sk_groove = _section_sketch(comp, plane, "Lip groove profile", f, groove)
        # sweep both before combining, so the edges they follow haven't been changed yet
        grooves += _sweep(comp, sk_groove, ch.edges, "Lip groove", ch.closed)
        lips += _sweep(comp, sk_lip, ch.edges, "Lip", ch.closed)
        planes.append(plane)
        sketches += [sk_lip, sk_groove]
    F = adsk.fusion.FeatureOperations
    _combine(comp, plan.groove, grooves, F.CutFeatureOperation, "Lip groove")
    join = _combine(comp, plan.lip, lips, F.JoinFeatureOperation, "Lip")
    for p in planes:
        p.isLightBulbOn = False
    for sk in sketches:
        sk.isVisible = False
    if tl is not None:
        end_idx = tl.markerPosition - 1
        if end_idx > start_idx:
            try:
                tl.timelineGroups.add(start_idx, end_idx).name = NAME
            except Exception as e:
                futil.log("couldn't group the lip in the timeline: %s" % e)
    return join


# --------------------------------------------------------------------------- the dialog
class Dialog:
    """State and event handlers for one run of the Lip command."""

    def __init__(self):
        self.inp = {}
        self.loading = False

    def keep(self, inp):
        self.inp[inp.id] = inp
        return inp

    def created(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        d = D.design()
        units = d.unitsManager.defaultLengthUnits if d else "mm"
        vin = adsk.core.ValueInput.createByString
        futil.debug("=== %s opened" % NAME)
        df = dict(DEFAULTS)
        try:
            df.update(json.loads(gx_comb.get_setting(LAST_KEY, "") or "{}"))
        except Exception as e:
            futil.log("couldn't restore the last lip settings: %s" % e)
        if df["reveal"] not in REVEALS.values():
            df["reveal"] = DEFAULTS["reveal"]

        e = self.keep(inputs.addSelectionInput("edges", "Edges", "Pick the edges along the parting line"))
        e.addSelectionFilter("Edges")
        e.setSelectionLimits(1, 0)
        e.tooltip = ("Edges where a parting face (the flat face where the two bodies meet) runs into a wall, on "
                     "either body. The lip stands on the parting face, measured across it from these edges.")
        self.keep(inputs.addBoolValueInput("wholeLoop", "Whole loop", True, "", bool(df["wholeLoop"]))).tooltip = (
            "One edge stands for the whole loop of the parting face it's on")
        for id_, name, tip in (("lipBody", "Lip body", "The body the lip stands on"),
                               ("grooveBody", "Groove body", "The body the groove is cut into")):
            s = self.keep(inputs.addSelectionInput(id_, name, tip))
            s.addSelectionFilter("SolidBodies")
            s.setSelectionLimits(1, 1)
        self.keep(inputs.addValueInput("height", "Lip height", units, vin(df["height"])))
        self.keep(inputs.addValueInput("width", "Lip width", units, vin(df["width"]))).tooltip = (
            "Width of the lip where it stands on the parting face")
        self.keep(inputs.addValueInput("offset", "Offset from edge", units, vin(df["offset"]))).tooltip = (
            "How far across the parting face from the picked edge the lip starts (0: flush with that wall)")
        self.keep(inputs.addValueInput("draft", "Draft angle", "deg", vin(df["draft"]))).tooltip = (
            "Both sides of the lip lean in by this angle, and the groove's sides with them")
        self.keep(D.dropdown(inputs, "reveal", "Reveal", REVEALS, df["reveal"])).tooltip = (
            "A gap between the lip and the groove's wall: on the lip's inner side (toward the inside of the "
            "loop), its outer side, or both")
        self.keep(inputs.addValueInput("gap", "Reveal gap", units, vin(df["gap"]))).tooltip = (
            "The gap between the lip and the groove's wall, square to the drafted sides")
        self.keep(inputs.addValueInput("topGap", "Top gap", units, vin(df["topGap"]))).tooltip = (
            "The gap between the top of the lip and the bottom of the groove")
        self.keep(inputs.addTextBoxCommandInput("info", "", "Pick the edges along the parting line.", 5, True))
        self.refresh()
        for event, fn, cls in ((cmd.inputChanged, self.on_input_changed, adsk.core.InputChangedEventHandler),
                               (cmd.validateInputs, self.on_validate, adsk.core.ValidateInputsEventHandler),
                               (cmd.executePreview, self.on_preview, adsk.core.CommandEventHandler),
                               (cmd.execute, self.on_execute, adsk.core.CommandEventHandler),
                               (cmd.destroy, self.on_destroy, adsk.core.CommandEventHandler)):
            futil.add_handler(event, fn, cls)

    # ------------------------------------------------------------- reading the dialog
    def refresh(self):
        self.inp["gap"].isVisible = REVEALS[self.inp["reveal"].selectedItem.name] != L.NONE

    def cfg(self):
        c = sizes(self.inp)
        c["reveal"] = REVEALS[self.inp["reveal"].selectedItem.name]
        return c

    def settings(self):
        data = {id_: self.inp[id_].expression for id_ in VALUES}
        data.update(reveal=REVEALS[self.inp["reveal"].selectedItem.name], wholeLoop=bool(self.inp["wholeLoop"].value))
        return data

    def selected(self, id_):
        sel = self.inp[id_]
        return [sel.selection(k).entity for k in range(sel.selectionCount)]

    def body(self, id_):
        b = self.selected(id_)
        return native(b[0]) if b else None

    def edges(self):
        """The picked edges, or None if any can't be read right now (while a preview is applied, the
        edges it changed are gone)."""
        out = []
        for e in self.selected("edges"):
            e = adsk.fusion.BRepEdge.cast(e)
            if e is None or not e.isValid:
                return None
            out.append(e)
        return out

    def quick_errors(self):
        """What's wrong that's quick to check (not the edges themselves; see Plan)."""
        errs = []
        for id_ in VALUES:
            inp = self.inp[id_]
            if inp.isVisible and not inp.isValidExpression:
                errs.append("%s isn't a valid value." % inp.name)
        if errs:
            return errs
        lip, groove = self.body("lipBody"), self.body("grooveBody")
        if not self.selected("edges"):
            errs.append("Pick the edges along the parting line.")
        if lip is None or groove is None:
            errs.append("Pick the lip body and the groove body.")
        elif _same(lip, groove):
            errs.append("The lip body and the groove body must be different bodies.")
        elif not _same(lip.parentComponent, groove.parentComponent):
            errs.append("The lip body and the groove body must be in the same component.")
        c = self.cfg()
        errs += L.errors(c["offset"], c["width"], c["height"], c["draft"], c["gap"], c["top"])
        return errs

    def plan(self):
        """A Plan from live edges, or None if they can't be read right now."""
        edges = self.edges()
        if edges is None:
            return None
        return Plan(edges, self.body("lipBody"), self.body("grooveBody"), self.cfg(), self.inp["wholeLoop"].value)

    # ------------------------------------------------------------- events
    def on_input_changed(self, args):
        inp = args.input
        futil.debug("%s input '%s' = %s" % (NAME, inp.id, futil.describe_input(inp)))
        if self.loading:
            return
        if inp.id == "reveal":
            self.refresh()
        elif inp.id == "edges":
            self.fill_bodies()

    def fill_bodies(self):
        """Picking an edge fills in empty body boxes: the edge's body is the lip body, and the body
        on the other side of its parting face the groove body."""
        edges = self.selected("edges")
        if not edges or (self.inp["lipBody"].selectionCount and self.inp["grooveBody"].selectionCount):
            return
        e = adsk.fusion.BRepEdge.cast(edges[0])
        if e is None or not e.isValid:
            return
        self.loading = True
        try:
            own = e.body
            if not self.inp["lipBody"].selectionCount and not _same(native(own), self.body("grooveBody")):
                self.inp["lipBody"].addSelection(own)
            if not self.inp["grooveBody"].selectionCount:
                comp = native(own).parentComponent
                other = touching_body(e, _items(comp.bRepBodies))
                if other is not None and not _same(other, self.body("lipBody")):
                    ctx = getattr(e, "assemblyContext", None)
                    self.inp["grooveBody"].addSelection(other.createForAssemblyContext(ctx) if ctx else other)
        except Exception as ex:
            futil.debug("lip: couldn't fill in the bodies: %s" % ex)
        finally:
            self.loading = False

    def on_validate(self, args):
        errs = self.quick_errors()
        args.areInputsValid = not errs
        if errs:
            self.show(errs)

    def show(self, errs, warnings=()):
        self.inp["info"].formattedText = "<br>".join(
            ["<span style='color:#c33'>%s</span>" % e for e in errs] +
            ["<span style='color:#b60'>%s</span>" % w for w in warnings])

    def info(self, plan):
        c, fmt = plan.cfg, D.Dialog.fmt
        top, _, depth = L.describe(c["offset"], c["width"], c["height"], c["draft"], 0.0, 0.0, c["top"])
        lines = ["<b>Lip</b> %s wide, %s tall%s" % (fmt(c["width"]), fmt(c["height"]),
                                                    ", %s across the top" % fmt(top) if c["draft"] > 0 else "")]
        for k, ch in enumerate(plan.chains[:3]):
            near, far = plan.sides(ch)
            _, wide, _ = L.describe(c["offset"], c["width"], c["height"], c["draft"], near, far, c["top"])
            _, outer = ch.infos[0].loop()
            inner_gap, outer_gap = (far, near) if outer else (near, far)
            lines.append("%s %d: %d edge(s)%s; groove %s wide, %s deep; gap %s inside, %s outside" % (
                "Loop" if ch.closed else "Chain", k + 1, len(ch.edges), "" if ch.closed else " (open)",
                fmt(wide), fmt(depth), fmt(inner_gap), fmt(outer_gap)))
        if len(plan.chains) > 3:
            lines.append("... and %d more" % (len(plan.chains) - 3))
        lines += ["<span style='color:#b60'>%s</span>" % w for w in plan.warnings]
        return "<br>".join(lines)

    def on_preview(self, args):
        if self.quick_errors():
            return
        plan = self.plan()
        if plan is None:
            return
        if plan.errors:
            self.show(plan.errors)
            return
        try:
            join = build(plan)
        except Exception as e:
            futil.handle_error("lip preview", show=False)
            self.show(["Couldn't build the lip: %s" % e], plan.warnings)
            return
        self.inp["info"].formattedText = self.info(plan)
        self.finish(join)
        args.isValidResult = True  # the preview is the result

    def finish(self, join):
        data = self.settings()
        try:
            join.attributes.add(ATTR_GROUP, SETTINGS_ATTR, json.dumps(data))
        except Exception as e:
            futil.debug("lip: couldn't store its settings: %s" % e)
        gx_comb.set_setting(LAST_KEY, json.dumps(data))

    def on_execute(self, args):
        """Only runs when the preview didn't make the result."""
        errs = self.quick_errors()
        plan = None if errs else self.plan()
        if plan is not None:
            errs = plan.errors
        if not errs and plan is None:
            errs = ["The picked edges can't be read. Pick them again."]
        try:
            if not errs:
                self.finish(build(plan))
                return
        except Exception as e:
            futil.handle_error("lip", show=False)
            errs = ["Couldn't build the lip: %s. The details are in logs/gx_tool.log." % e]
        args.executeFailed = True
        args.executeFailedMessage = "The Lip wasn't made:\n" + "\n".join(errs)

    def on_destroy(self, args):
        futil.debug("=== %s closed (reason: %s)" % (NAME, args.terminationReason))


def _on_created(args):
    global _dlg
    _dlg = Dialog()
    _dlg.created(args)


def start():
    futil.create_button(CMD_ID, NAME,
                        "A lip on one body and the groove it fits into on the other, swept along the edges where they "
                        "meet, with draft and a reveal (gap) on the inner side, the outer side or both.",
                        RESOURCES, _on_created, panel_id=PANEL)


def stop():
    futil.delete_button(CMD_ID, panel_id=PANEL)
