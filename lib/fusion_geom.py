"""Fusion-side geometry: reading selections, creating splines, adding constraints."""

import json
import math

import adsk.core
import adsk.fusion

from . import gx_math as g
from . import polygrid as pg
from .futil import ATTR_GROUP, debug, log


def p2(pt):
    return (pt.x, pt.y)


def P3(p):
    return adsk.core.Point3D.create(p[0], p[1], 0)


def model_point(sketch, p):
    return sketch.sketchToModelSpace(P3(p))


def model_dir(sketch, origin, u):
    a = model_point(sketch, origin)
    b = model_point(sketch, g.add(origin, u))
    v = a.vectorTo(b)
    v.normalize()
    return v


# --------------------------------------------------------------------------- corner analysis
class CornerGeo:
    """Two sketch lines, their (virtual) intersection and the leg directions.

    Which side of the corner each leg lies on is taken from the line's endpoints
    when both are on one side, otherwise from where the user clicked the line
    (like the sketch fillet tool).
    """

    def __init__(self, lineA, clickA, lineB, clickB):
        if lineA.parentSketch != lineB.parentSketch:
            raise ValueError("Both lines must be in the same sketch.")
        self.sketch = lineA.parentSketch
        self.lineA, self.lineB = lineA, lineB
        a0, a1 = p2(lineA.startSketchPoint.geometry), p2(lineA.endSketchPoint.geometry)
        b0, b1 = p2(lineB.startSketchPoint.geometry), p2(lineB.endSketchPoint.geometry)
        hit = g.line_intersection(a0, g.sub(a1, a0), b0, g.sub(b1, b0))
        if hit is None:
            raise ValueError("The lines are parallel, so they have no corner.")
        self.corner = hit[0]
        self.uA, self.farA, self.availA = self._leg(lineA, a0, a1, clickA)
        self.uB, self.farB, self.availB = self._leg(lineB, b0, b1, clickB)
        if abs(g.cross(self.uA, self.uB)) < 1e-6:
            raise ValueError("The lines are collinear, so they have no corner.")
        self.bis = g.bisector(self.uA, self.uB)

    def _leg(self, line, e0, e1, click):
        c = self.corner
        u = g.normalize(g.sub(e1, e0))
        t0, t1 = g.dot(g.sub(e0, c), u), g.dot(g.sub(e1, c), u)
        tol = 1e-7
        if (t0 >= -tol and t1 >= -tol) or (t0 <= tol and t1 <= tol):
            side = 1.0 if (t0 + t1) > 0 else -1.0
        else:
            tc = 0.0
            if click is not None:
                tc = g.dot(g.sub(p2(self.sketch.modelToSketchSpace(click)), c), u)
            side = 1.0 if tc >= 0 else -1.0
        u = g.mul(u, side)
        t0, t1 = t0 * side, t1 * side
        far = line.startSketchPoint if t0 > t1 else line.endSketchPoint
        return u, far, max(t0, t1)

    def control_points(self, distA, distB, apex_mode, apex_dist):
        return g.corner_control_points(self.corner, self.uA, self.uB, distA, distB, apex_mode, apex_dist)


def line_dir_into_curve(sketch_point):
    """If the point is the end of a sketch line, the unit direction continuing that line."""
    here = p2(sketch_point.geometry)
    try:
        ents = sketch_point.connectedEntities
    except Exception:
        return None
    for e in ents or []:
        line = adsk.fusion.SketchLine.cast(e)
        if not line or line.isConstruction:
            continue
        s, t = p2(line.startSketchPoint.geometry), p2(line.endSketchPoint.geometry)
        other = t if g.length(g.sub(s, here)) < g.length(g.sub(t, here)) else s
        try:
            return g.normalize(g.sub(here, other))
        except ValueError:
            continue
    return None


# --------------------------------------------------------------------------- creation
def add_spline(sketch, ctrl, degree):
    deg = adsk.fusion.SplineDegrees.SplineDegreeThree if degree == 3 else adsk.fusion.SplineDegrees.SplineDegreeFive
    splines = sketch.sketchCurves.sketchControlPointSplines
    pts = [P3(c) for c in ctrl]
    debug("  add_spline degree %d, %d control points: %s"
          % (degree, len(ctrl), ", ".join("(%.4f, %.4f)" % (c[0], c[1]) for c in ctrl)))
    try:
        return splines.add(pts, deg)
    except Exception as e:
        debug("  add_spline with a list failed (%s); retrying with ObjectCollection" % e)
        coll = adsk.core.ObjectCollection.create()
        for p in pts:
            coll.add(p)
        return splines.add(coll, deg)


def spline_control_sketch_points(spline):
    """The spline's control points as SketchPoints (needed for constraints)."""
    cps = getattr(spline, "controlPoints", None)
    items = list(cps or [])
    debug("  spline.controlPoints: %d item(s), type %s"
          % (len(items), getattr(items[0], "objectType", type(items[0]).__name__) if items else "-"))
    out = []
    for c in items:
        sp = adsk.fusion.SketchPoint.cast(c)
        if not sp:
            return []
        out.append(sp)
    return out


def read_spline(spline):
    """(control SketchPoints, 2D control points, degree, knots or None) of a control-point spline."""
    cps = spline_control_sketch_points(spline)
    ctrl = [p2(sp.geometry) for sp in cps]
    degree, knots = g.degree_for_count(len(ctrl)), None
    try:
        geo = spline.geometry
        degree = int(geo.degree)
        k = list(geo.knots)
        if len(k) == len(ctrl) + degree + 1:
            knots = k
    except Exception as e:
        log("could not read spline degree/knots, assuming clamped uniform: %s" % e)
    debug("  read_spline: %d control points, degree %d, knots %s" % (len(ctrl), degree, knots))
    return cps, ctrl, degree, knots


def _point_dimensions(sketch):
    """{entityToken of a sketch point: [(dimension, other_entity)]} for point-to-point linear dims."""
    out = {}
    for i in range(sketch.sketchDimensions.count):
        d = sketch.sketchDimensions.item(i)
        e1, e2 = getattr(d, "entityOne", None), getattr(d, "entityTwo", None)
        if not (e1 and e2) or not getattr(d, "isDriving", True):
            continue
        s1, s2 = adsk.fusion.SketchPoint.cast(e1), adsk.fusion.SketchPoint.cast(e2)
        if not (s1 and s2):
            continue
        out.setdefault(s1.entityToken, []).append((d, s2))
        out.setdefault(s2.entityToken, []).append((d, s1))
    return out


def apply_control_points(sketch, cps, new_ctrl, tol=1e-7):
    """Move the spline's control points to new_ctrl while respecting the sketch's constraints:
    a point held by a driving point-to-point dimension is moved by changing that dimension's
    value; other points are dragged (and the constraint solver keeps them on their lines, in
    symmetry, etc). Returns the largest remaining distance from a target."""
    dims = _point_dimensions(sketch)
    for i, (sp, target) in enumerate(zip(cps, new_ctrl)):
        cur = p2(sp.geometry)
        if g.length(g.sub(cur, target)) < tol:
            continue  # already there (e.g. moved with its symmetric partner)
        handled = False
        for dim, other in dims.get(sp.entityToken, []):
            val = g.length(g.sub(target, p2(other.geometry)))
            try:
                dim.parameter.value = val
                handled = True
                debug("  cp %d: dimension -> %.5f" % (i, val))
            except Exception as e:
                log("  cp %d: could not set dimension (%s)" % (i, e))
        if not handled:
            try:
                sp.move(adsk.core.Vector3D.create(target[0] - cur[0], target[1] - cur[1], 0))
                debug("  cp %d: moved by (%.5f, %.5f)" % (i, target[0] - cur[0], target[1] - cur[1]))
            except Exception as e:
                log("  cp %d: could not move (%s)" % (i, e))
    resid = max(g.length(g.sub(p2(sp.geometry), t)) for sp, t in zip(cps, new_ctrl))
    debug("  apply_control_points: largest remaining offset %.3g" % resid)
    return resid


class Constrainer:
    """Adds constraints one by one; a failure is logged and counted, not fatal."""

    def __init__(self, sketch):
        self.sketch = sketch
        self.gc = sketch.geometricConstraints
        self.dims = sketch.sketchDimensions
        self.failed = []

    def _try(self, label, fn, *args):
        try:
            r = fn(*args)
            debug("  + %s" % label)
            return r
        except Exception as e:
            self.failed.append(label)
            log("constraint failed: %s (%s)" % (label, e))
            return None

    def coincident(self, pt, ent, label="coincident"):
        return self._try(label, self.gc.addCoincident, pt, ent)

    def symmetry(self, e1, e2, axis, label="symmetry"):
        return self._try(label, self.gc.addSymmetry, e1, e2, axis)

    def distance(self, p1, p2_, text_pt, label="dimension"):
        orient = adsk.fusion.DimensionOrientations.AlignedDimensionOrientation
        return self._try(label, self.dims.addDistanceDimension, p1, p2_, orient, P3(text_pt))


def tag(entity, params, comb):
    try:
        entity.attributes.add(ATTR_GROUP, "gx", json.dumps(params))
        if comb:
            entity.attributes.add(ATTR_GROUP, "comb", "1")
    except Exception as e:
        log("could not tag entity: %s" % e)


def build_corner(geo, P, final):
    """Create the corner spline. P: dict of parameters. final=False for preview (no constraints).

    Returns (spline, list_of_failed_constraint_labels).
    """
    m, apex_mode, n, degree = g.corner_style(P["count"], P["five_type"], P["apex"])
    ctrl = geo.control_points(P["distA"], P["distB"], apex_mode, P["apex_dist"])
    sketch = geo.sketch
    if not final:
        return add_spline(sketch, ctrl, degree), []

    sketch.isComputeDeferred = True
    try:
        spline = add_spline(sketch, ctrl, degree)
        cps = spline_control_sketch_points(spline)
        con = Constrainer(sketch)
        if len(cps) != n:
            con.failed.append("control points not accessible via API; curve left unconstrained")
            tag(spline, P, P["comb"])
            return spline, con.failed

        lines = sketch.sketchCurves.sketchLines
        c = geo.corner
        L = P["distA"][0]
        corner_pt = sketch.sketchPoints.add(P3(c))
        con.coincident(corner_pt, geo.lineA, "corner on line A")
        con.coincident(corner_pt, geo.lineB, "corner on line B")

        # trim: the original lines become the construction "virtual sharp"; new solid
        # segments run from each line's far end to the curve ends.
        if P["trim"]:
            geo.lineA.isConstruction = True
            geo.lineB.isConstruction = True
            lines.addByTwoPoints(geo.farA, cps[0])
            lines.addByTwoPoints(geo.farB, cps[-1])

        A = cps[:m]
        B = list(reversed(cps[-m:]))  # B[i] mirrors A[i]
        apex = cps[m] if apex_mode != g.APEX_NONE else None

        for i, pt in enumerate(A):
            con.coincident(pt, geo.lineA, "A%d on line A" % i)

        if apex_mode == g.APEX_CORNER:
            con.coincident(apex, corner_pt, "apex on corner")

        # outward normals, for placing dimension text away from the curve
        nA = (-geo.uA[1], geo.uA[0])
        if g.dot(nA, geo.uB) > 0:
            nA = g.mul(nA, -1)
        nB = (-geo.uB[1], geo.uB[0])
        if g.dot(nB, geo.uA) > 0:
            nB = g.mul(nB, -1)

        if P["symmetric"]:
            axis = lines.addByTwoPoints(corner_pt, P3(g.add(c, g.mul(geo.bis, L))))
            axis.isConstruction = True
            # B0 on line B + pairwise symmetry fixes the axis as the bisector; the other
            # B points get onto line B through the symmetry (adding it again would over-constrain).
            con.coincident(B[0], geo.lineB, "B0 on line B")
            for i in range(m):
                con.symmetry(A[i], B[i], axis, "symmetry %d" % i)
            if apex_mode == g.APEX_FREE:
                con.coincident(apex, axis, "apex on bisector")
        else:
            for i, pt in enumerate(B):
                con.coincident(pt, geo.lineB, "B%d on line B" % i)

        if P["dims"]:
            step = 0.12 * L
            for i, pt in enumerate(A):
                d = P["distA"][i]
                con.distance(corner_pt, pt, g.add(c, g.add(g.mul(geo.uA, d * 0.5), g.mul(nA, step * (i + 1)))),
                             "dim A%d" % i)
            if not P["symmetric"]:
                for i, pt in enumerate(B):
                    d = P["distB"][i]
                    con.distance(corner_pt, pt, g.add(c, g.add(g.mul(geo.uB, d * 0.5), g.mul(nB, step * (i + 1)))),
                                 "dim B%d" % i)
            elif apex_mode == g.APEX_FREE and abs(P["apex_dist"]) > 1e-9:
                con.distance(corner_pt, apex, g.add(c, g.mul(geo.bis, P["apex_dist"] * 0.5 + step)), "dim apex")

        tag(spline, P, P["comb"])
        return spline, con.failed
    finally:
        sketch.isComputeDeferred = False


def build_freeform(sketch, start_pt, end_pt, P, final):
    """start_pt/end_pt: SketchPoints. Returns (spline, failed, info)."""
    a, b = p2(start_pt.geometry), p2(end_pt.geometry)
    ds = line_dir_into_curve(start_pt) if P["match"] else None
    de = line_dir_into_curve(end_pt) if P["match"] else None
    debug("  freeform: start %s dir %s, end %s dir %s" % (a, ds, b, de))
    n = 6 if (P["count"] == 5 and P["five_type"] == "quintic") else P["count"]
    ctrl, degree, gs, ge, warning = g.freeform_control_points(a, b, n, ds, de, P["reach"])
    spline = add_spline(sketch, ctrl, degree)
    failed = []
    if final:
        cps = spline_control_sketch_points(spline)
        con = Constrainer(sketch)
        if cps:
            con.coincident(cps[0], start_pt, "start point")
            con.coincident(cps[-1], end_pt, "end point")
        else:
            con.failed.append("control points not accessible via API")
        failed = con.failed
        tag(spline, dict(P, mode="freeform"), P["comb"])
    return spline, failed, (gs, ge, warning, len(ctrl), degree)


# --------------------------------------------------------------------------- blends into curves (conics)
def curve2d_from_evaluator(ev, project_point=None, project_vec=None, periodic=False):
    """g.Curve2D over a Fusion CurveEvaluator3D. project_* map 3D -> 2D (default: sketch x, y)."""
    pp = project_point or (lambda p: (p.x, p.y))
    pv = project_vec or (lambda v: (v.x, v.y))
    ok, u0, u1 = ev.getParameterExtents()

    def third(u, h=1e-5):
        try:
            ok3, d3 = ev.getThirdDerivative(u)
            if ok3:
                return pv(d3)
        except Exception:
            pass
        _, a = ev.getSecondDerivative(u - h)
        _, b = ev.getSecondDerivative(u + h)
        va, vb = pv(a), pv(b)
        return ((vb[0] - va[0]) / (2 * h), (vb[1] - va[1]) / (2 * h))

    def fn(u):
        _, p = ev.getPointAtParameter(u)
        _, d1 = ev.getFirstDerivative(u)
        _, d2 = ev.getSecondDerivative(u)
        return pp(p), pv(d1), pv(d2), third(u)

    c = g.Curve2D(fn, u0, u1, periodic=(u1 - u0) if periodic else None)

    def param_at(p2d, guess=None):
        ok, u = ev.getParameterAtPoint(adsk.core.Point3D.create(p2d[0], p2d[1], 0))
        u = u if ok else (guess if guess is not None else 0.5 * (u0 + u1))
        return c.closest(p2d, u)
    c.param_at = param_at
    return c


def sketch_curve2d(curve):
    """g.Curve2D for a sketch curve (sketch space). Lines become infinite lines with parameter 0..1
    over the drawn segment, so a corner can lie beyond a line's end."""
    line = adsk.fusion.SketchLine.cast(curve)
    if line:
        a, b = p2(line.startSketchPoint.geometry), p2(line.endSketchPoint.geometry)
        c = g.line_curve(a, g.sub(b, a))
        d = g.sub(b, a)
        c.param_at = lambda p, guess=None: g.dot(g.sub(p, a), d) / g.dot(d, d)
        c.sample_range = (-3.0, 4.0)
        return c
    periodic = bool(adsk.fusion.SketchCircle.cast(curve) or adsk.fusion.SketchEllipse.cast(curve))
    c = curve2d_from_evaluator(curve.geometry.evaluator, periodic=periodic)
    c.sample_range = (c.u_min, c.u_max)
    return c


def _wrapped_diff(curve, a, b):
    d = a - b
    if curve.periodic:
        P = curve.periodic
        d = (d + P / 2) % P - P / 2
    return d


class CurveCornerGeo:
    """Two sketch curves (lines, arcs, circles, ellipses, conics, splines) that cross: the
    crossing nearest the clicks, and which way along each curve is the kept side (the clicked side)."""

    def __init__(self, entA, clickA, entB, clickB):
        if entA.parentSketch != entB.parentSketch:
            raise ValueError("Both curves must be in the same sketch.")
        self.sketch = entA.parentSketch
        self.entA, self.entB = entA, entB
        self.A, self.B = sketch_curve2d(entA), sketch_curve2d(entB)
        ca = p2(self.sketch.modelToSketchSpace(clickA)) if clickA else None
        cb = p2(self.sketch.modelToSketchSpace(clickB)) if clickB else None
        ua_c = self.A.param_at(ca) if ca else None
        ub_c = self.B.param_at(cb) if cb else None

        starts = []
        if ua_c is not None and ub_c is not None:
            starts.append((ua_c, ub_c))
        # coarse search for near-crossings along both curves
        n = 32
        ra, rb = self.A.sample_range, self.B.sample_range
        ua_s = [ra[0] + (ra[1] - ra[0]) * i / n for i in range(n + 1)]
        ub_s = [rb[0] + (rb[1] - rb[0]) * i / n for i in range(n + 1)]
        pa = [self.A.point(u) for u in ua_s]
        pb = [self.B.point(u) for u in ub_s]
        pairs = sorted(((g.length(g.sub(x, y)), i, j) for i, x in enumerate(pa) for j, y in enumerate(pb)))[:6]
        starts += [(ua_s[i], ub_s[j]) for _, i, j in pairs]
        hits = []
        for s in starts:
            h = g.find_intersection(self.A, s[0], self.B, s[1])
            if h and self.A.inside(h[0]) and self.B.inside(h[1]):
                hits.append(h)
        if not hits:
            raise ValueError("These curves don't cross (a G3 fillet between curves needs a corner).")
        ref = [p for p in (ca, cb) if p] or [self.A.point(hits[0][0])]
        self.uX, self.vX = min(hits, key=lambda h: sum(g.length(g.sub(self.A.point(h[0]), r)) for r in ref))
        self.X = self.A.point(self.uX)
        if ua_c is None or ub_c is None:
            raise ValueError("Click each curve on the side you want to keep.")
        da, db = _wrapped_diff(self.A, ua_c, self.uX), _wrapped_diff(self.B, ub_c, self.vX)
        if abs(da) < 1e-9 or abs(db) < 1e-9:
            raise ValueError("Click each curve a little away from the corner, on the side to keep.")
        self.sA, self.sB = (1 if da > 0 else -1), (1 if db > 0 else -1)
        a, b = self.A.frame(self.uX, self.sA)["T"], self.B.frame(self.vX, self.sB)["T"]
        self.bis = g.bisector(a, b)
        debug("curve corner: X=(%.4f, %.4f) uX=%.5f vX=%.5f kept %+d/%+d, angle %.2f deg" % (
            self.X[0], self.X[1], self.uX, self.vX, self.sA, self.sB, math.degrees(math.acos(max(-1, min(1, g.dot(a, b)))))))

    def fillet(self, R, count, match, emulate):
        return g.curve_fillet(self.A, self.uX, self.sA, self.B, self.vX, self.sB, R, count, emulate, match)


def _split(curve, pt):
    """Split a sketch curve at a point; returns the resulting curves or None."""
    for name in ("split", "breakCurve"):
        fn = getattr(curve, name, None)
        if not fn:
            continue
        try:
            res = fn(P3(pt), True)
            out = [res.item(i) for i in range(res.count)]
            debug("  %s at (%.4f, %.4f): %d piece(s)" % (name, pt[0], pt[1], len(out)))
            return out
        except Exception as e:
            log("  %s failed: %s" % (name, e))
    return None


def _ends(c):
    return p2(c.startSketchPoint.geometry), p2(c.endSketchPoint.geometry)


def _trim_curve(ent, contact, X, tol=1e-6):
    """Split ent at the contact point (and at X if needed); make the piece between them
    construction. Returns the kept piece that ends at the contact, or None if splitting failed."""
    pieces = _split(ent, contact)
    if not pieces:
        return None
    at_contact = [c for c in pieces if any(g.length(g.sub(e, contact)) < tol for e in _ends(c))]
    # the piece on the corner side: its other end (or a point on it) is closer to X
    def toward_x(c):
        e0, e1 = _ends(c)
        far = e1 if g.length(g.sub(e0, contact)) < tol else e0
        return g.length(g.sub(far, X))
    if len(at_contact) < 2:
        return at_contact[0] if at_contact else None
    corner_piece, kept = sorted(at_contact, key=toward_x)[:2]
    e0, e1 = _ends(corner_piece)
    far = e1 if g.length(g.sub(e0, contact)) < tol else e0
    if g.length(g.sub(far, X)) > tol:  # X is inside this piece: split again there
        more = _split(corner_piece, X)
        if more:
            for c in more:
                if any(g.length(g.sub(e, contact)) < tol for e in _ends(c)):
                    corner_piece = c
    corner_piece.isConstruction = True
    return kept


def build_curve_blend(cg, res, P, final):
    """Blend from a CurveCornerGeo fillet result. Returns (spline, failed constraint labels)."""
    sketch = cg.sketch
    if not final:
        return add_spline(sketch, res["ctrl"], res["degree"]), []
    sketch.isComputeDeferred = True
    try:
        spline = add_spline(sketch, res["ctrl"], res["degree"])
        cps = spline_control_sketch_points(spline)
        con = Constrainer(sketch)
        ca, cb = res["ctrl"][0], res["ctrl"][-1]
        keptA, keptB = cg.entA, cg.entB
        if P["trim"]:
            ka = _trim_curve(cg.entA, ca, cg.X)
            kb = _trim_curve(cg.entB, cb, cg.X)
            if ka is None or kb is None:
                con.failed.append("trim (couldn't split a curve; it was left as it was)")
            keptA, keptB = ka or keptA, kb or keptB
        if cps:
            con.coincident(cps[0], keptA, "start on curve A")
            con.coincident(cps[-1], keptB, "end on curve B")
        for other, label in ((keptA, "A"), (keptB, "B")):
            try:
                sketch.geometricConstraints.addSmooth(spline, other)
                debug("  + smooth (G2) with curve %s" % label)
            except Exception as e:
                debug("  smooth (G2) with curve %s not added: %s" % (label, e))
        try:
            spline.isFixed = True  # sketch constraints can't hold G3 to a curve; keep the exact shape
        except Exception as e:
            log("  couldn't fix the blend: %s" % e)
        tag(spline, dict(P, mode="curve_fillet", ua=res["ua"], ub=res["ub"]), P["comb"])
        return spline, con.failed
    finally:
        sketch.isComputeDeferred = False


# --------------------------------------------------------------------------- sketch areas (polygrid)
def _mapper(sketch, pc):
    """Point3D -> 2D sketch space for a profile's curve geometry. ProfileCurve.geometry should be
    in sketch space; this checks it against the sketch curve it comes from, and falls back to
    model space if that matches better."""
    def sk(p):
        return (p.x, p.y)

    def md(p):
        return p2(sketch.modelToSketchSpace(p))
    try:
        ev = pc.sketchEntity.geometry.evaluator  # sketch space
        ok, s, _ = pc.geometry.evaluator.getEndPoints()

        def off(p):
            ok1, t = ev.getParameterAtPoint(p)
            ok2, q = ev.getPointAtParameter(t) if ok1 else (False, None)
            return p.distanceTo(q) if ok2 else float("inf")
        d_sk, d_md = off(s), off(sketch.modelToSketchSpace(s))
        debug("  profile curves: %.3g off in sketch space, %.3g in model space" % (d_sk, d_md))
        return md if d_md < d_sk else sk
    except Exception as e:
        log("couldn't check the profile's coordinate space (%s); assuming sketch space" % e)
        return sk


def _curve_prim(geo, to2, tol):
    """A profile curve's Curve3D as a polygrid primitive (lib/polygrid.py)."""
    T = adsk.core.Curve3DTypes
    ct = geo.curveType
    if ct == T.Line3DCurveType:
        return ("line", to2(geo.startPoint), to2(geo.endPoint))
    if ct == T.Circle3DCurveType:
        return ("arc", to2(geo.center), geo.radius, 0.0, 2 * math.pi)
    ev = geo.evaluator
    ok, t0, t1 = ev.getParameterExtents()
    if ct == T.Arc3DCurveType:
        c = to2(geo.center)
        _, pm = ev.getPointAtParameter(0.5 * (t0 + t1))
        a, m, b = to2(geo.startPoint), to2(pm), to2(geo.endPoint)
        a0 = math.atan2(a[1] - c[1], a[0] - c[0])
        ccw = (math.atan2(b[1] - c[1], b[0] - c[0]) - a0) % (2 * math.pi)
        mid = (math.atan2(m[1] - c[1], m[0] - c[0]) - a0) % (2 * math.pi)
        return ("arc", c, g.length(g.sub(a, c)), a0, ccw if mid < ccw else ccw - 2 * math.pi)
    ok, pts = ev.getStrokes(t0, t1, tol)
    return ("curve", [to2(p) for p in pts])


def profile_loops(profile, tol=1e-4):
    """A sketch Profile as polygrid loops: [[primitives]] in the profile's sketch space."""
    sketch = profile.parentSketch
    loops, to2 = [], None
    for i in range(profile.profileLoops.count):
        pl = profile.profileLoops.item(i)
        prims = []
        for j in range(pl.profileCurves.count):
            pc = pl.profileCurves.item(j)
            if to2 is None:
                to2 = _mapper(sketch, pc)
            prims.append(_curve_prim(pc.geometry, to2, tol))
        loops.append(prims)
    return loops


def curve_prims(curves, tol=1e-4):
    """Whole sketch curves (an outline) as polygrid primitives, in their sketch's space."""
    return [_curve_prim(c.geometry, p2, tol) for c in curves]


def profile_point(profile):
    """A 2D point well inside a sketch profile (sketch space), or None if it can't be read."""
    try:
        return pg.interior_point(pg.prepare_area(profile_loops(profile)))
    except Exception as e:
        debug("  couldn't find a point inside a profile: %s" % e)
        return None


def sketch_map(src, dst):
    """(f, coplanar): f maps 2D points in src's sketch space to dst's."""
    o, x, y = (dst.modelToSketchSpace(src.sketchToModelSpace(P3(q))) for q in ((0, 0), (1, 0), (0, 1)))
    coplanar = max(abs(o.z), abs(x.z), abs(y.z)) < 1e-6
    ax, ay, bx, by = x.x - o.x, x.y - o.y, y.x - o.x, y.y - o.y

    def f(p):
        return (o.x + ax * p[0] + bx * p[1], o.y + ay * p[0] + by * p[1])
    return f, coplanar


def model_map(sketch):
    """f mapping a 2D sketch-space point to model-space (x, y, z)."""
    o, x, y = (sketch.sketchToModelSpace(P3(q)) for q in ((0, 0), (1, 0), (0, 1)))
    X, Y = (x.x - o.x, x.y - o.y, x.z - o.z), (y.x - o.x, y.y - o.y, y.z - o.z)

    def f(p):
        return (o.x + X[0] * p[0] + Y[0] * p[1], o.y + X[1] * p[0] + Y[1] * p[1], o.z + X[2] * p[0] + Y[2] * p[1])
    return f


def _is_sketch_point(x):
    return getattr(x, "objectType", "").endswith("SketchPoint")


def _xyz(x):
    return x.geometry if _is_sketch_point(x) else x


def _join(sketch, keep, other):
    """Make two sketch points at the same place into one (or failing that, coincident)."""
    if keep is None or other is None or keep == other:
        return
    try:
        if keep.merge(other):
            return
    except Exception as e:
        debug("  merge failed (%s); adding a coincident constraint" % e)
    try:
        sketch.geometricConstraints.addCoincident(other, keep)
    except Exception as e:
        debug("  coincident failed too: %s" % e)


def _ends_at(ent, a):
    """The curve's (sketch point nearest a, the other end point)."""
    s, e = ent.startSketchPoint, ent.endSketchPoint
    ds = math.hypot(s.geometry.x - a[0], s.geometry.y - a[1])
    de = math.hypot(e.geometry.x - a[0], e.geometry.y - a[1])
    return (s, e) if ds <= de else (e, s)


def _attach(sketch, ent, start, end, a):
    """After a curve was made from plain points: merge its ends into the sketch points they
    should share. Returns the sketch point at its far end."""
    s_sp, e_sp = _ends_at(ent, a)
    if _is_sketch_point(start):
        _join(sketch, start, s_sp)
    if _is_sketch_point(end):
        _join(sketch, end, e_sp)
        return end
    return e_sp


_arc_by_points = [True]  # does SketchArcs.addByThreePoints take sketch points? (learned on first use)


def draw_loop(sketch, prims, f, fixed=True):
    """Draw polygrid drawing primitives (lib/polygrid.py) in order. Pieces that meet head to tail share
    their sketch point, and a closed loop closes onto its first point. f maps a 2D point into this
    sketch's space. Returns the curves made, in drawing order."""
    if not prims:
        return []
    k = max((i for i, p in enumerate(prims) if p[0] == "seg"), default=None)
    if k is not None:
        prims = prims[k + 1:] + prims[:k + 1]  # end on a line: lines can close onto a sketch point
    cv = sketch.sketchCurves
    first_sp = first_pt = prev_sp = prev_pt = None
    made = []

    def near(p, q):
        return q is not None and math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-7

    for i, p in enumerate(prims):
        kind = p[0]
        if kind == "circle":
            ent = cv.sketchCircles.addByCenterRadius(P3(f(p[1])), p[2])
            prev_sp = None
        elif kind == "spline" and p[2]:
            coll = adsk.core.ObjectCollection.create()
            for q in p[1]:
                coll.add(P3(f(q)))
            ent = cv.sketchFittedSplines.add(coll)
            try:
                ent.isClosed = True
            except Exception as e:
                debug("  couldn't close a fitted spline: %s" % e)
            prev_sp = None
        else:
            if kind == "seg":
                a, b = f(p[1]), f(p[2])
            elif kind == "arc3":
                a, b = f(p[1]), f(p[3])
            else:  # spline / cspline
                a, b = f(p[1][0]), f(p[1][-1])
            start = prev_sp if (prev_sp is not None and near(a, prev_pt)) else P3(a)
            closes = i == len(prims) - 1 and first_sp is not None and near(b, first_pt)
            end = first_sp if closes else P3(b)
            if kind == "seg":
                ent = cv.sketchLines.addByTwoPoints(start, end)
                s_sp, e_sp = ent.startSketchPoint, ent.endSketchPoint
            else:
                ent = None
                if kind == "arc3":
                    mid = P3(f(p[2]))
                    if _arc_by_points[0]:
                        try:
                            ent = cv.sketchArcs.addByThreePoints(start, mid, end)
                        except Exception as e:
                            _arc_by_points[0] = False
                            debug("  arcs can't be made from sketch points (%s); merging their ends instead" % e)
                    if ent is None:
                        ent = cv.sketchArcs.addByThreePoints(_xyz(start), mid, _xyz(end))
                elif kind == "cspline":  # a G3 / G4 corner
                    ent = add_spline(sketch, [f(q) for q in p[1]], p[2])
                else:
                    coll = adsk.core.ObjectCollection.create()
                    for q in p[1]:
                        coll.add(P3(f(q)))
                    ent = cv.sketchFittedSplines.add(coll)
                s_sp, _ = _ends_at(ent, a)
                e_sp = _attach(sketch, ent, start, end, a)
                if _is_sketch_point(start):
                    s_sp = start
            if first_sp is None:
                first_sp, first_pt = s_sp, a
            prev_sp, prev_pt = e_sp, b
        made.append(ent)
        if fixed:
            try:
                ent.isFixed = True
            except Exception:
                pass
    return made


def clear_sketch(sketch):
    """Delete every curve and point in a sketch (except its origin)."""
    sketch.isComputeDeferred = True
    try:
        curves = sketch.sketchCurves
        for c in [curves.item(i) for i in range(curves.count)]:
            try:
                if c.isValid:
                    c.deleteMe()
            except Exception as e:
                debug("  couldn't delete a curve: %s" % e)
        origin = sketch.originPoint
        pts = sketch.sketchPoints
        for p in [pts.item(i) for i in range(pts.count)]:
            try:
                if p.isValid and p != origin:
                    p.deleteMe()
            except Exception:
                pass
    finally:
        sketch.isComputeDeferred = False
