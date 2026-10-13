"""Bottom blends for Polygrid: a radius, or a G3/G4 curve, where the walls of its cut meet the floor
(or, for what it extrudes, where the walls meet the body below).

- Radius / Curvature (G2): one Fillet on those edges (Fusion's Fillet, tangent or curvature continuity).
- G3 / G4: Fusion's Fillet stops at G2, so the blend is built. Its cross-section is the GX corner curve
  between wall and floor (what GX G3 Fillet makes for a 90 degree corner), swept around a pocket
  outline into a ring. Outlines that are translated copies of one another (every whole pocket, and
  the cut pockets along a straight edge) share one sweep: the ring is copied to the others (in one
  base feature) and all the rings are joined to the bodies with Combine. An outline whose sweep
  fails gets Fusion's G2 fillet instead.
"""

import math

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib import polygrid as W
from . import polygrid_dialog as D

STYLES = {"None": "none", "Radius": "radius", "Curvature (G2)": "g2", "G3 curve (7 points)": "g3",
          "G4 curve (9 points)": "g4"}
SWEPT = ("g3", "g4")
INTO_POCKET = ("cut_holes", "extrude_ribs")  # operations whose blend goes inside the pocket outline


# --------------------------------------------------------------------------- 3D vectors
def _v(p):
    return (p.x, p.y, p.z)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return _mul(a, 1.0 / n)


class Frame:
    """A sketch's plane in model space: 2D sketch points <-> 3D, plus height along its normal."""

    def __init__(self, sk):
        o, x, y = (_v(sk.sketchToModelSpace(fusion_geom.P3(q))) for q in ((0, 0), (1, 0), (0, 1)))
        self.o, self.X, self.Y = o, _sub(x, o), _sub(y, o)
        self.N = _unit(_cross(self.X, self.Y))

    def to3(self, p, z=0.0):
        return _add(self.o, _add(_add(_mul(self.X, p[0]), _mul(self.Y, p[1])), _mul(self.N, z)))

    def vec3(self, d):
        return _add(_mul(self.X, d[0]), _mul(self.Y, d[1]))

    def to2(self, P):
        """(x, y, height above the plane) of a model point."""
        d = _sub(P, self.o)
        return (_dot(d, self.X) / _dot(self.X, self.X), _dot(d, self.Y) / _dot(self.Y, self.Y), _dot(d, self.N))


# --------------------------------------------------------------------------- applies?
def applies(cfg):
    """Does this operation have a bottom to blend? (Not a cut right through, not a new body.)"""
    if cfg["op"].startswith("cut"):
        return cfg["extent"] != "all"
    return cfg["extrudeAs"] == "join"


def end_level(ext, frame):
    """Height of the extrude's end (its floor, for a cut) above the grid sketch's plane."""
    faces = ext.endFaces
    if not faces or faces.count == 0:
        raise ValueError("the extrude has no end face (it goes right through)")
    return frame.to2(_v(faces.item(0).pointOnFace))[2]


# --------------------------------------------------------------------------- G3 / G4 rings
def _path(comp, ents):
    """A closed path through every curve of one pocket outline (in drawing order). Built from the
    full list, not by chaining from one curve: Fusion only chains curves it sees as connected, and
    the outline's lines and corner splines don't always count, which swept just one corner."""
    coll = adsk.core.ObjectCollection.create()
    for e in ents:
        coll.add(e)
    try:
        path = comp.features.createPath(coll, False)
    except Exception as e:
        futil.debug("  createPath from %d curves failed (%s); trying Path.create" % (len(ents), e))
        path = adsk.fusion.Path.create(coll, adsk.fusion.ChainedCurveOptions.noChainedCurves)
    n = path.count
    closed = getattr(path, "isClosed", True)
    futil.debug("  sweep path: %d of %d outline curves, closed=%s" % (n, len(ents), closed))
    if n < len(ents) or not closed:
        raise RuntimeError("the sweep path covers %d of the outline's %d curves%s"
                           % (n, len(ents), "" if closed else " and isn't closed"))
    return path


def _ring(comp, frame, ents, lp, cfg, z_end, f, back, in_pocket):
    """Sweep the blend's cross-section around one pocket outline (its curves in the grid sketch,
    in drawing order). Returns (the ring body, [plane, profile sketch, sweep])."""
    cut = cfg["op"].startswith("cut")
    ent = ents[0]
    ev = ent.worldGeometry.evaluator
    _, t0, _ = ev.getParameterExtents()
    _, P = ev.getPointAtParameter(t0)
    _, d = ev.getFirstDerivative(t0)
    p0, t = _v(P), _unit(_v(d))
    left = _unit(_cross(frame.N, t))  # left of the outline as it runs, in the sketch plane
    x, y, _ = frame.to2(_add(p0, _mul(left, 0.01)))
    n = left if in_pocket(back((x, y))) == (cfg["op"] in INTO_POCKET) else _mul(left, -1.0)
    zc = z_end if cut else 0.0                        # the corner: on the floor, or on the body below
    up = (-1.0 if z_end > 0 else 1.0) if cut else (1.0 if z_end > 0 else -1.0)
    w = _mul(frame.N, up)                             # up the wall
    C = _add(p0, _mul(frame.N, zc))
    pts = [_add(C, _add(_mul(n, a), _mul(w, b))) for a, b in W.bottom_profile(cfg["bottomR"], cfg["bottom"])]

    made = []
    try:
        planes = comp.constructionPlanes
        pin = planes.createInput()
        pin.setByDistanceOnPath(ent, adsk.core.ValueInput.createByReal(0.0))
        plane = planes.add(pin)
        made.append(plane)
        plane.name = "Polygrid bottom plane"
        psk = comp.sketches.add(plane)
        made.append(psk)
        psk.name = "Polygrid bottom profile"

        def to2(Q):
            return fusion_geom.p2(psk.modelToSketchSpace(adsk.core.Point3D.create(*Q)))
        c2, k2 = to2(C), [to2(Q) for Q in pts]
        psk.isComputeDeferred = True
        try:
            fusion_geom.draw_loop(psk, [("seg", c2, k2[0], False), ("cspline", k2, 5), ("seg", k2[-1], c2, False)],
                                  lambda q: q)
        finally:
            psk.isComputeDeferred = False
        if psk.profiles.count < 1:
            raise RuntimeError("the blend's cross-section didn't make a profile")
        sweeps = comp.features.sweepFeatures
        si = sweeps.createInput(psk.profiles.item(0), _path(comp, ents),
                                adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
        sw = sweeps.add(si)
        made.append(sw)
        sw.name = "Polygrid bottom"
        body = sw.bodies.item(0)
        _check_ring(body, frame, lp, f, zc, cfg)
        plane.isLightBulbOn = False
        psk.isVisible = False
        return body, made
    except Exception:
        for x_ in reversed(made):  # take back what this outline made
            try:
                x_.deleteMe()
            except Exception:
                pass
        raise


def _check_ring(body, frame, lp, f, zc, cfg):
    """The ring must sit around its own outline (a sweep started in the wrong place doesn't). Loose:
    the middle of its model-space box must be near the outline, at about the right height."""
    reach = W.bottom_reach(cfg["bottomR"], cfg["bottom"]) * 1.01 + 1e-4
    bb = body.boundingBox
    cx, cy, cz = frame.to2(_mul(_add(_v(bb.minPoint), _v(bb.maxPoint)), 0.5))
    x0, y0, x1, y1 = W.bbox([f(q) for p in lp for q in W.prim_polyline(p, 8)])
    slack = 0.5 * math.hypot(x1 - x0, y1 - y0) + reach
    if not (x0 - slack <= cx <= x1 + slack and y0 - slack <= cy <= y1 + slack and abs(cz - zc) <= slack):
        raise RuntimeError("the swept ring isn't around its outline")


def _copies(comp, rings_and_moves):
    """Copies of rings moved by 3D vectors, as bodies of one base feature. Returns the feature."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    temps = []
    for body, v in rings_and_moves:
        t = tbm.copy(body)
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(*v)
        if not tbm.transform(t, m):
            raise RuntimeError("couldn't move a copy of a ring")
        temps.append(t)
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    try:
        for t in temps:
            comp.bRepBodies.add(t, bf)
    finally:
        bf.finishEdit()
    bf.name = "Polygrid bottom copies"
    return bf


def _combine(comp, tools, bodies):
    """Join the rings to the bodies they touch. Returns the Combine features."""
    made, left = [], list(tools)
    for b in bodies:
        mine = [t for t in left if b.boundingBox.intersects(t.boundingBox)]
        if not mine:
            continue
        left = [t for t in left if not any(t == m for m in mine)]
        coll = adsk.core.ObjectCollection.create()
        for t in mine:
            coll.add(t)
        ci = comp.features.combineFeatures.createInput(b, coll)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        ci.isKeepToolBodies = False
        c = comp.features.combineFeatures.add(ci)
        c.name = "Polygrid bottom"
        made.append(c)
    if left:
        futil.log("polygrid bottom: %d ring(s) touch none of the picked bodies" % len(left))
    return made


# --------------------------------------------------------------------------- fillets
def bottom_edges(ext, frame, back, loops, cut, z_end, tol=2e-3):
    """The extrude's edges along the bottom of the given outlines: the floor's edges for a cut, the
    walls' bottom edges for an extrusion."""
    faces = ext.endFaces if cut else ext.sideFaces
    level = z_end if cut else 0.0
    boxes = []
    for _, _, lp, _ in loops:
        x0, y0, x1, y1 = W.bbox([q for p in lp for q in W.prim_polyline(p, 8)])
        boxes.append((x0 - tol, y0 - tol, x1 + tol, y1 + tol, lp))
    found, seen = [], set()
    for i in range(faces.count):
        edges = faces.item(i).edges
        for k in range(edges.count):
            e = edges.item(k)
            key = D.token(e) or id(e)
            if key in seen:
                continue
            seen.add(key)
            x, y, z = frame.to2(_v(e.pointOnEdge))
            if abs(z - level) > 1e-4:
                continue  # a vertical edge, or the top
            q = back((x, y))
            if any(bx[0] <= q[0] <= bx[2] and bx[1] <= q[1] <= bx[3] and W.loop_distance(bx[4], q) < tol
                   for bx in boxes):
                found.append(e)
    return found


def _fillet(comp, edges, expr, g2):
    fi = comp.features.filletFeatures.createInput()
    coll = adsk.core.ObjectCollection.create()
    for e in edges:
        coll.add(e)
    vi = adsk.core.ValueInput.createByString(expr)
    try:
        es = fi.edgeSetInputs.addConstantRadiusEdgeSet(coll, vi, True)
        if g2:
            es.continuity = adsk.fusion.SurfaceContinuityTypes.CurvatureSurfaceContinuityType
    except AttributeError:  # older API
        fi.addConstantRadiusEdgeSet(coll, vi, True)
        fi.isG2 = g2
    try:
        fi.isRollingBallCorner = True
    except Exception:
        pass
    fil = comp.features.filletFeatures.add(fi)
    fil.name = "Polygrid bottom"
    return fil


# --------------------------------------------------------------------------- messages
def explain(e, cfg):
    """Fusion's modelling errors, in plain words with what to change."""
    s = str(e)
    size = cfg.get("bottomRexpr") or "the bottom size"
    if "ASM_SELF_INTER" in s or "intersect itself" in s:
        return ("the %s bottom curve (%s) is too big for the pocket corners. Swept around a corner tighter "
                "than the curve's reach, it would run into itself. Use a smaller bottom size, or bigger "
                "pocket corners." % (cfg["bottom"].upper(), size))
    if "ASM_BL_UNFIN_SHEET" in s or "requested size" in s:
        return ("the bottom radius (%s) doesn't fit. It's too big for these pockets: their corners are too "
                "tight, the ribs too narrow, or the depth too shallow. Try a smaller bottom size." % size)
    if "no end face" in s:
        return ("the cut goes right through the body, so there's no floor to blend. Make the cut shallower, "
                "or set Bottom to None.")
    if "sweep path covers" in s:
        return "%s, so the swept bottom wouldn't go all the way round." % s
    first = s.strip().splitlines()[0] if s.strip() else type(e).__name__
    return first.split(" // ")[-1].split(" - ", 1)[-1].lstrip("0123456789 :").strip() or first


# --------------------------------------------------------------------------- all together
def build(comp, sk, ext, res, cfg, f, back, loop_ents, bodies):
    """Blend the bottom of the polygrid's extrude (with the timeline marker just after it).
    loop_ents: the grid sketch's curves for each pocket outline, in res order (None if the
    outlines aren't closed in the sketch). Returns (features made in timeline order, notes)."""
    notes = dict(sweeps=0, copies=0, g2=0, edges=0)
    style = cfg["bottom"]
    if style == "none" or not applies(cfg):
        return [], notes
    loops = W.blend_loops(res, cfg["op"])
    if not loops:
        return [], notes
    frame = Frame(sk)
    cut = cfg["op"].startswith("cut")
    try:
        z_end = end_level(ext, frame)
    except ValueError as e:
        notes["error"] = explain(e, cfg)
        futil.log("polygrid bottom: %s" % e)
        return [], notes
    items, fallback = [], []
    if style in SWEPT:
        flat, n = {}, 0
        for i, pk in enumerate(res["pockets"]):
            for j in range(len(pk["loops"])):
                flat[(i, j)] = n
                n += 1
        if loop_ents is None:
            futil.log("polygrid bottom: the outlines aren't closed in the sketch (no border); using G2 fillets")
            fallback = list(loops)
        else:
            in_pocket = W.pocket_finder(res)
            rings, moves, moved_loops = [], [], []
            for seed, members in W.shape_groups([lp for _, _, lp, _ in loops]):
                i, j, lp, _ = loops[seed]
                try:
                    body, made = _ring(comp, frame, loop_ents[flat[(i, j)]], lp, cfg, z_end, f, back, in_pocket)
                except Exception as e:
                    futil.log("polygrid bottom: sweep around pocket %d failed (%s); %d outline(s) get G2 fillets"
                              % (i, e, len(members)))
                    notes.setdefault("why", explain(e, cfg))
                    fallback.extend(loops[k] for k, _ in members)
                    continue
                items.extend(made)
                rings.append(body)
                notes["sweeps"] += 1
                o = f((0.0, 0.0))
                for k, dd in members[1:]:
                    g = f(dd)
                    moves.append((body, frame.vec3((g[0] - o[0], g[1] - o[1]))))
                    moved_loops.append(loops[k])
            tools = list(rings)
            if moves:
                try:
                    bf = _copies(comp, moves)
                    items.append(bf)
                    tools += [bf.bodies.item(k) for k in range(bf.bodies.count)]
                    notes["copies"] = len(moves)
                except Exception as e:
                    futil.log("polygrid bottom: copying the rings failed (%s); those outlines get G2 fillets" % e)
                    fallback.extend(moved_loops)
            if tools:
                items.extend(_combine(comp, tools, bodies))
    fillet_loops = loops if style not in SWEPT else fallback
    if fillet_loops:
        edges = bottom_edges(ext, frame, back, fillet_loops, cut, z_end)
        notes["edges"] = len(edges)
        notes["g2"] = len(fallback)
        if edges:
            try:
                items.append(_fillet(comp, edges, cfg["bottomRexpr"], g2=style != "radius"))
            except Exception as e:  # keep what was made (rings, combines); explain the rest
                futil.log("polygrid bottom: fillet of %d edge(s) failed: %s" % (len(edges), e))
                notes["error"] = explain(e, cfg)
        else:
            futil.log("polygrid bottom: found no bottom edges to fillet")
            notes["error"] = "no bottom edges were found to fillet."
    futil.debug("polygrid bottom %s: %s" % (style, notes))
    return items, notes
