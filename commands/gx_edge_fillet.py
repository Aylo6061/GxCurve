"""'GX Edge Fillet' command (Solid > Modify): like the Fillet feature, but rounds solid edges
with a GX G3/G4 profile instead of a circular arc.

How it works, per edge:
  1. Check the edge is straight and joins two planar faces.
  2. Make a construction plane across the edge, at its start, and a sketch on it.
  3. In that sketch draw the GX fillet profile: the corner region bounded by the two faces
     and the G3/G4 curve (same layout as GX G3 Fillet: smoothest by default, or "Emulate
     fillet" for the closest match to a radius-R arc).
  4. Extrude that region along the whole edge: a cut for an outside (convex) edge, a join for
     an inside (concave) edge.
All planes and sketches are made first, while the selected edges still exist; then the
extrudes run. The steps are grouped in the timeline as "GX Edge Fillet".

Faces may also be curved, if their cross-section is the same all along the edge (cylinders,
elliptical cylinders, extruded parabolas/hyperbolas): the blend is then G3 to the face's own
curvature (gx_math.curve_fillet), and the profile follows the face exactly (each cross-section
piece is drawn as an exact conic arc).

Limits: straight edges only; cones, spheres and tori aren't supported (their cross-section
changes along the edge). Where several filleted edges meet at a vertex, the cuts overlap rather
than blending into a corner patch.
"""

import math
import os

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib import gx_math as g
from ..lib.futil import app, ui

CMD_ID = "GXTool_EdgeFillet"
PANEL = "SolidModifyPanel"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gx_fillet")

COUNTS = {"7 points (G3)": 7, "9 points (G4)": 9}
MATCHES = {
    "Best fit to the fillet": g.FILLET_BEST,
    "Same depth into the corner": g.FILLET_DEPTH,
    "Same tangent points": g.FILLET_TANGENT,
    "Never tighter than R": g.FILLET_MINRADIUS,
}


def _i(inputs, id_):
    return inputs.itemById(id_)


def _dd(inputs, id_, table):
    return table[_i(inputs, id_).selectedItem.name]


def _v(p):
    return (p.x, p.y, p.z)


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _vnorm(a):
    n = math.sqrt(_vdot(a, a))
    return _vmul(a, 1.0 / n)


def _P(a):
    return adsk.core.Point3D.create(a[0], a[1], a[2])


# --------------------------------------------------------------------------- edge analysis
class EdgeGeo:
    """Straight edge between two faces whose cross-section is the same all along the edge
    (planes, cylinders, elliptical cylinders, extruded conics): direction, in-face leg
    directions, convexity, and each face's cross-section as a 2D curve in a frame across the
    edge (origin at the edge start, x = into face A, y = t x x)."""

    def __init__(self, edge):
        self.edge = edge
        if not adsk.core.Line3D.cast(edge.geometry):
            raise ValueError("Only straight edges are supported.")
        if edge.faces.count != 2:
            raise ValueError("The edge must join exactly two faces.")
        f1, f2 = edge.faces.item(0), edge.faces.item(1)
        self.p0, self.p1 = _v(edge.startVertex.geometry), _v(edge.endVertex.geometry)
        self.t = _vnorm(_vsub(self.p1, self.p0))
        self.length = math.sqrt(_vdot(_vsub(self.p1, self.p0), _vsub(self.p1, self.p0)))
        self.mid = _vmul(_vadd(self.p0, self.p1), 0.5)
        self.faces = (f1, f2)
        self.planar_faces = tuple(bool(adsk.core.Plane.cast(f.geometry)) for f in self.faces)
        for k, (f, pl) in enumerate(zip(self.faces, self.planar_faces)):
            if not pl and not self._extruded_along(f):
                raise ValueError("Face %d changes shape along the edge. Supported: flat faces and faces with "
                                 "the same cross-section all along the edge (cylinders, elliptical cylinders, "
                                 "extruded ellipses/parabolas/hyperbolas). Cones, spheres and tori aren't "
                                 "supported yet." % (k + 1))
        self.planar = all(self.planar_faces)
        self.n = tuple(self._normal(f) for f in self.faces)
        self.v = tuple(self._into_face(f, n) for f, n in zip(self.faces, self.n))
        self.convex = _vdot(self.v[0], self.n[1]) < 0
        self.phi = math.acos(max(-1.0, min(1.0, _vdot(self.v[0], self.v[1]))))
        if self.phi < 1e-3 or self.phi > math.pi - 1e-3:
            raise ValueError("The faces are (nearly) flat across this edge; there's no corner to round.")
        self.body = edge.body
        self.e1 = self.v[0]
        self.e2 = _vnorm(_vcross(self.t, self.e1))
        if not self.planar:
            self.sections = tuple(self._section(f, v, pl) for f, v, pl in zip(self.faces, self.v, self.planar_faces))

    # 2D frame across the edge
    def to2d(self, p):
        d = _vsub(p, self.p0)
        return (_vdot(d, self.e1), _vdot(d, self.e2))

    def vec2d(self, v):
        return (_vdot(v, self.e1), _vdot(v, self.e2))

    def from2d(self, q, origin=None):
        o = origin or self.p0
        return _vadd(o, _vadd(_vmul(self.e1, q[0]), _vmul(self.e2, q[1])))

    def _extruded_along(self, face):
        """True if every face normal is perpendicular to the edge: the face is swept straight along
        the edge, so its cross-section doesn't change (cylinders, extruded curves)."""
        ev = face.evaluator
        box = ev.parametricRange()
        a, b = box.minPoint, box.maxPoint
        for i in range(1, 5):
            for j in range(1, 5):
                uv = adsk.core.Point2D.create(a.x + (b.x - a.x) * i / 5, a.y + (b.y - a.y) * j / 5)
                ok, nrm = ev.getNormalAtParameter(uv)
                if ok and abs(_vdot(_vnorm(_v(nrm)), self.t)) > 1e-6:
                    return False
        return True

    def _section(self, face, v, planar):
        """(Curve2D, parameter at the edge, kept sign) for the face's cross-section."""
        if planar:
            return g.line_curve((0.0, 0.0), self.vec2d(v)), 0.0, +1
        ev = face.evaluator
        ok, uv = ev.getParameterAtPoint(_P(self.p0))
        best = None
        # try both iso directions at both parameters (robust to the API's u/v convention); keep the
        # curve that passes through the edge start and runs across the edge
        for is_u, par in ((False, uv.x), (True, uv.y), (True, uv.x), (False, uv.y)):
            try:
                curves = ev.getIsoCurve(par, is_u)
            except Exception as e:
                futil.debug("  getIsoCurve(%s) failed: %s" % (is_u, e))
                continue
            items = [curves.item(i) for i in range(curves.count)] if hasattr(curves, "count") else list(curves)
            for cv in items:
                cev = cv.evaluator
                ok, u = cev.getParameterAtPoint(_P(self.p0))
                if not ok:
                    continue
                ok, q = cev.getPointAtParameter(u)
                ok2, d1 = cev.getFirstDerivative(u)
                if not (ok and ok2) or math.dist(_v(q), self.p0) > 1e-6 * max(1.0, self.length):
                    continue
                tan = _vnorm(_v(d1))
                if abs(_vdot(tan, self.t)) < 1e-6:
                    best = cv
                    break
            if best:
                break
        if best is None:
            raise ValueError("Couldn't read the face's cross-section.")
        periodic = bool(adsk.core.Circle3D.cast(best) or adsk.core.Ellipse3D.cast(best))
        c = fusion_geom.curve2d_from_evaluator(best.evaluator, project_point=lambda p: self.to2d(_v(p)),
                                               project_vec=lambda w: self.vec2d(_v(w)), periodic=periodic)
        ok, u0 = best.evaluator.getParameterAtPoint(_P(self.p0))
        uX = c.closest((0.0, 0.0), u0)
        sign = 1 if g.dot(c.frame(uX, +1)["T"], self.vec2d(v)) > 0 else -1
        futil.debug("  face cross-section: %s, k at edge %.5g" % (
            type(best).__name__, c.frame(uX, sign)["k"]))
        return c, uX, sign

    def _normal(self, face):
        ok, n = face.evaluator.getNormalAtPoint(_P(self.mid))
        if not ok:
            raise ValueError("Couldn't read a face normal.")
        return _vnorm(_v(n))

    def on_face(self, face, p):
        ev = face.evaluator
        ok, uv = ev.getParameterAtPoint(_P(p))
        if not ok or not ev.isParameterOnFace(uv):
            return False
        ok, q = ev.getPointAtParameter(uv)  # projection must be the point itself
        return ok and math.dist(_v(q), p) < 1e-6 + 1e-6 * self.length

    def _into_face(self, face, n):
        """Unit direction lying in the face, perpendicular to the edge, pointing into the face."""
        v = _vnorm(_vcross(self.t, n))
        eps = 1e-4 * max(self.length, 1.0)
        if self.on_face(face, _vadd(self.mid, _vmul(v, eps))):
            return v
        if self.on_face(face, _vadd(self.mid, _vmul(v, -eps))):
            return _vmul(v, -1)
        raise ValueError("Couldn't tell which side of the edge a face is on.")

    def describe(self):
        return ("edge len %.4f, angle between faces %.2f deg, %s, faces %s" %
                (self.length, math.degrees(self.phi), "outside (cut)" if self.convex else "inside (join)",
                 "/".join("flat" if p else "curved" for p in self.planar_faces)))

    def curve_fillet(self, R, count, match, emulate):
        (A, ua, sa), (B, ub, sb) = self.sections
        return g.curve_fillet(A, ua, sa, B, ub, sb, R, count, emulate, match)


# --------------------------------------------------------------------------- command
def _on_created(args):
    S.clear()
    cmd = args.command
    inputs = cmd.commandInputs
    futil.debug("=== GX Edge Fillet opened")
    sel = inputs.addSelectionInput("edges", "Edges", "Pick straight edges (faces: flat, cylinders, or extruded conics)")
    sel.addSelectionFilter("LinearEdges")
    sel.setSelectionLimits(0, 0)
    units = app.activeProduct.unitsManager.defaultLengthUnits if app.activeProduct else "mm"
    inputs.addValueInput("radius", "Radius", units, adsk.core.ValueInput.createByReal(0.2))
    for id_, name, table in (("count", "Curve", COUNTS), ("match", "Match", MATCHES)):
        dd = inputs.addDropDownCommandInput(id_, name, adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, label in enumerate(table):
            dd.listItems.add(label, k == 0, "")
    inputs.addBoolValueInput("emulate", "Emulate fillet", True, "", False)
    inputs.addTextBoxCommandInput("info", "", "Select edges.", 5, True)

    futil.add_handler(cmd.inputChanged, _on_input_changed, adsk.core.InputChangedEventHandler)
    futil.add_handler(cmd.validateInputs, _on_validate, adsk.core.ValidateInputsEventHandler)
    futil.add_handler(cmd.executePreview, _on_preview, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.execute, _on_execute, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.destroy, _on_destroy, adsk.core.CommandEventHandler)


S = {}  # per-command state: last good plan


def _on_input_changed(args):
    futil.debug("edge fillet input '%s' = %s" % (args.input.id, futil.describe_input(args.input)))


def _edges(inputs):
    """Selected edges, or None if any can't be read right now. (While the preview's cut is
    showing, the original edges are gone and Fusion can hand back None for them.)"""
    sel = _i(inputs, "edges")
    out = []
    for i in range(sel.selectionCount):
        e = adsk.fusion.BRepEdge.cast(sel.selection(i).entity)
        if e is None or not e.isValid:
            return None
        out.append(e)
    return out


def _plan(inputs):
    """([(EdgeGeo, layout)], problems), or None if the selected edges can't be read right now.

    Fusion checks the inputs (validateInputs) *before* it rolls back the previous preview, so
    right after you pick a new edge, the edges the previous preview's cut consumed can't be read.
    That says nothing about whether they're valid: the preview runs after the rollback and does
    the real check with live edges."""
    edges = _edges(inputs)
    if edges is None:
        futil.debug("edge fillet: selected edges unreadable (previous preview still applied); deferring check")
        return None
    return _make_plan(inputs, edges)


def _curve_fillet_cached(geo, R, count, match, emulate):
    """Curved-face blends take ~1-7 s to optimise; remember them per edge and setting."""
    key = (geo.edge.entityToken, round(R, 9), count, match, emulate)
    cache = S.setdefault("ccache", {})
    if key not in cache:
        try:
            cache[key] = geo.curve_fillet(R, count, match, emulate)
            futil.debug("  curved blend: gap %.4g, tightest radius %.4g" % (cache[key]["max_gap"], cache[key]["min_radius"]))
        except ValueError as e:
            cache[key] = str(e)
    if isinstance(cache[key], str):
        raise ValueError(cache[key])
    return cache[key]


def _make_plan(inputs, edges):
    R = _i(inputs, "radius").value
    count, match = _dd(inputs, "count", COUNTS), _dd(inputs, "match", MATCHES)
    emulate = _i(inputs, "emulate").value
    plans, errs = [], []
    if not edges:
        return [], ["Select one or more straight edges. Their faces can be flat, cylindrical, or extruded conics."]
    if R <= 0:
        return [], ["The radius must be above zero."]
    for k, e in enumerate(edges):
        try:
            geo = EdgeGeo(e)
            # check points near both ends and at the middle of the edge (not exactly at the ends,
            # which lie on the face boundary where the on-face test is unreliable)
            bases = (_vadd(geo.p0, _vmul(geo.t, 0.02 * geo.length)), geo.mid,
                     _vadd(geo.p1, _vmul(geo.t, -0.02 * geo.length)))
            if geo.planar:
                lay = g.fillet_layout(geo.phi, R, count, match, emulate=emulate)
                lay["kind"] = "lines"
                contacts = [_vmul(v, lay["L"]) for v in geo.v]  # offsets from the edge
            else:
                res = _curve_fillet_cached(geo, R, count, match, emulate)
                c2 = (res["ctrl"][0], res["ctrl"][-1])
                contacts = [_vsub(geo.from2d(q), geo.p0) for q in c2]
                lay = dict(kind="curves", res=res, min_radius=res["min_radius"],
                           L=max(g.length(q) for q in c2))
            for f, off in zip(geo.faces, contacts):
                for base in bases:
                    if not geo.on_face(f, _vadd(base, off)):
                        raise ValueError("The radius is too big: the blend would run %.4g cm onto a face "
                                         "that's narrower than that." % lay["L"])
            plans.append((geo, lay))
            futil.debug("edge %d: %s, leg %.5f" % (k + 1, geo.describe(), lay["L"]))
        except ValueError as ex:
            errs.append("Edge %d: %s" % (k + 1, ex))
            futil.debug("edge %d rejected: %s" % (k + 1, ex))
    return plans, errs


def _on_validate(args):
    inputs = args.firingEvent.sender.commandInputs
    plan = _plan(inputs)
    if plan is None:
        # can't tell yet: keep OK enabled; the preview re-checks with live edges and execute
        # won't build anything invalid
        args.areInputsValid = True
        return
    plans, errs = plan
    args.areInputsValid = not errs
    if errs:
        _show_errors(inputs, errs)


def _show_errors(inputs, errs):
    _i(inputs, "info").formattedText = "<span style='color:#c33'>%s</span>" % "<br>".join(errs)


def _curved_profile(sk, geo, res, P):
    """Region bounded by each face's exact cross-section (corner -> contact; a line or an exact
    conic arc) and the G3 blend between the contacts."""
    def sp(q):  # 2D frame point -> sketch-space Point3D
        return sk.modelToSketchSpace(_P(geo.from2d(q, origin=P)))

    spline = fusion_geom.add_spline(sk, [fusion_geom.p2(sp(q)) for q in res["ctrl"]], res["degree"])
    cps = fusion_geom.spline_control_sketch_points(spline)
    ends = [cps[0], cps[-1]] if cps else [spline.startSketchPoint, spline.endSketchPoint]
    segs = []
    for (C, uX, s), uc in zip(geo.sections, (res["ua"], res["ub"])):
        seg = g.conic_segment(C, uX, uc)
        if seg[0] == "line":
            c = sk.sketchCurves.sketchLines.addByTwoPoints(sp(seg[1]), sp(seg[2]))
        else:
            c = sk.sketchCurves.sketchConicCurves.add(sp(seg[1]), sp(seg[2]), sp(seg[3]), seg[4])
            futil.debug("  face section drawn as an exact conic arc, rho %.5f" % seg[4])
        segs.append(c)
    gc = sk.geometricConstraints
    for a, b, label in ((segs[0].startSketchPoint, segs[1].startSketchPoint, "corner"),
                        (segs[0].endSketchPoint, ends[0], "contact A"),
                        (segs[1].endSketchPoint, ends[1], "contact B")):
        try:
            gc.addCoincident(a, b)
        except Exception as e:
            futil.debug("  coincident at %s not added: %s" % (label, e))


def _profile_sketch(comp, geo, lay, count):
    """Plane across the edge + sketch of the region between the faces and the GX curve."""
    planes = comp.constructionPlanes
    pin = planes.createInput()
    pin.setByDistanceOnPath(geo.edge, adsk.core.ValueInput.createByReal(0.0))
    plane = planes.add(pin)
    plane.name = "GX fillet plane"
    pg = plane.geometry
    origin = _v(pg.origin)
    # where the edge's line crosses the plane
    s = _vdot(_vsub(origin, geo.p0), geo.t)
    P = _vadd(geo.p0, _vmul(geo.t, s))

    sk = comp.sketches.add(plane)
    sk.name = "GX fillet profile"
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    if lay["kind"] == "lines":
        m, apex_mode, n, deg = g.corner_style(count)
        pts3 = ([_vadd(P, _vmul(geo.v[0], d)) for d in lay["dist"]] + [P] +
                [_vadd(P, _vmul(geo.v[1], d)) for d in reversed(lay["dist"])])
        ctrl = [fusion_geom.p2(sk.modelToSketchSpace(_P(q))) for q in pts3]
        corner2 = fusion_geom.p2(sk.modelToSketchSpace(_P(P)))
        spline = fusion_geom.add_spline(sk, ctrl, deg)
        cps = fusion_geom.spline_control_sketch_points(spline)
        start = cps[0] if cps else spline.startSketchPoint
        end = cps[-1] if cps else spline.endSketchPoint
        l1 = lines.addByTwoPoints(start, fusion_geom.P3(corner2))
        lines.addByTwoPoints(l1.endSketchPoint, end)
    else:
        _curved_profile(sk, geo, lay["res"], P)
    sk.isComputeDeferred = False

    profs = sk.profiles
    if profs.count == 0:
        raise RuntimeError("The profile sketch didn't form a closed region.")
    prof = min((profs.item(i) for i in range(profs.count)),
               key=lambda p: p.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area)
    # how far to extrude, and which way, to cover the whole edge
    s0, s1 = _vdot(_vsub(geo.p0, P), geo.t), _vdot(_vsub(geo.p1, P), geo.t)
    far = s1 if abs(s1) > abs(s0) else s0
    positive = _vdot(_v(pg.normal), geo.t) * far > 0
    futil.debug("  profile: corner %s, %d profile(s), extrude %.4f %s" % (
        tuple(round(c, 4) for c in P), profs.count, abs(far), "+" if positive else "-"))
    return plane, sk, prof, abs(far), positive


def _build(inputs):
    edges = _edges(inputs)
    if edges is None:  # building needs the live edges, never a remembered plan
        futil.log("edge fillet: can't build, selected edges are unreadable")
        return 0
    plans, errs = _make_plan(inputs, edges)
    if errs or not plans:
        return 0
    design = adsk.fusion.Design.cast(app.activeProduct)
    tl = design.timeline if design.designType == adsk.fusion.DesignTypes.ParametricDesignType else None
    start_idx = tl.markerPosition if tl else None
    count = _dd(inputs, "count", COUNTS)
    futil.debug("edge fillet: %d edge(s), R=%.5f, count=%d" % (len(plans), _i(inputs, "radius").value, count))

    # pass 1: planes and sketches, while the selected edges still exist
    staged = []
    for geo, lay in plans:
        futil.debug("  " + geo.describe() + ", leg %.5f" % lay["L"])
        comp = geo.body.parentComponent
        plane, sk, prof, dist, positive = _profile_sketch(comp, geo, lay, count)
        staged.append((geo, comp, plane, sk, prof, dist, positive))

    # pass 2: the extrudes
    for geo, comp, plane, sk, prof, dist, positive in staged:
        op = (adsk.fusion.FeatureOperations.CutFeatureOperation if geo.convex
              else adsk.fusion.FeatureOperations.JoinFeatureOperation)
        ext = comp.features.extrudeFeatures
        ei = ext.createInput(prof, op)
        direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if positive
                     else adsk.fusion.ExtentDirections.NegativeExtentDirection)
        ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(dist)),
                            direction)
        ei.participantBodies = [geo.body]
        f = ext.add(ei)
        f.name = "GX edge fillet"
        sk.isVisible = False
        plane.isLightBulbOn = False

    if tl is not None:
        end_idx = tl.markerPosition - 1
        if end_idx > start_idx:
            try:
                grp = tl.timelineGroups.add(start_idx, end_idx)
                grp.name = "GX Edge Fillet"
            except Exception as e:
                futil.log("couldn't group timeline items: %s" % e)
    return len(staged)


def _on_preview(args):
    inputs = args.command.commandInputs
    plan = _plan(inputs)
    if plan is None:
        return
    plans, errs = plan
    if errs:
        _show_errors(inputs, errs)  # the real verdict, from live edges
        return
    n = _build(inputs)
    R = _i(inputs, "radius").value
    lines = ["<b>%d edge(s) rounded with a G%d profile (%s)</b>" % (
        n, 3 if _dd(inputs, "count", COUNTS) == 7 else 4,
        "closest to the arc" if _i(inputs, "emulate").value else "smoothest shape")]
    for k, (geo, lay) in enumerate(plans[:4]):
        lines.append("Edge %d: %.0f°, %s, runs %.4g cm onto each face; tightest radius %.0f%% of R" % (
            k + 1, math.degrees(geo.phi), "cut" if geo.convex else "join", lay["L"], 100 * lay["min_radius"] / R))
    _i(inputs, "info").formattedText = "<br>".join(lines)
    args.isValidResult = n > 0  # if the preview couldn't build, let execute try with live edges


def _on_execute(args):
    inputs = args.command.commandInputs
    plan = _plan(inputs)
    if plan is not None and plan[1]:
        ui.messageBox("GX Edge Fillet wasn't applied:\n  - " + "\n  - ".join(plan[1]))
        return
    if not _build(inputs):
        ui.messageBox("GX Edge Fillet couldn't build the fillet. The details are in logs/gx_tool.log.")


def _on_destroy(args):
    futil.debug("=== GX Edge Fillet closed (reason: %s)" % args.terminationReason)
    S.clear()


def start():
    futil.create_button(CMD_ID, "GX Edge Fillet",
                        "Round solid edges with a G3/G4 curvature-continuous profile instead of a circular arc.",
                        RESOURCES, _on_created, panel_id=PANEL)


def stop():
    futil.delete_button(CMD_ID, panel_id=PANEL)
