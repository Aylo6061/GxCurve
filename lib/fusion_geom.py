"""Fusion-side geometry: reading selections, creating splines, adding constraints."""

import json

import adsk.core
import adsk.fusion

from . import gx_math as g
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
