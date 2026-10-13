"""'Screw Boss' command (Solid > Create): a boss for a screw at each picked sketch point on a flat
face, with a hole sized for a tapped thread, a thread-forming screw or a heat-set insert (M2 to M10,
see lib/screw_boss.py), or a diameter you set.

It's made of ordinary features, grouped in the timeline as "Screw Boss":
  1. "Screw boss sketch", on the points' sketch plane: a circle of the boss's outside diameter round
     each point (a linked projection of it), one with a diameter dimension and the rest equal to it.
  2. "Screw boss": one Extrude of the circles, joined to the body they stand on or as a new body, a
     distance, or up to a face, plane or body with an offset.
  3. For each plane the bosses' tops end on: a "Screw boss top" sketch on a top face with the points
     projected onto it, and a "Screw boss hole" Hole feature going down from there, with a 118° drill
     point or a flat bottom. The flat bottom is a 180° tip angle; if Fusion won't take that, the
     hole is an extruded cut instead.
Each part can be changed afterwards in Fusion's own dialogs: the outside diameter in the sketch, the
height in the Extrude, the hole in the Hole feature. The settings are stored on the extrude
(attribute GX_tool/screwBoss; the other items point to it with GX_tool/screwBossOf), and the last
ones used come back next time.

Editing: select the Screw Boss (its group, or any of its sketches or features) and run the command,
or right-click > Edit Screw Boss. The dialog reads the stored settings, with any changes made since in
Fusion's own dialogs on top, and the timeline rolls back to just before it while the dialog is open.
OK redraws the boss sketch and points the *same* Extrude at it, so later features keep their
references to the bosses; the top sketches and holes are made again.
"""

import json
import math
import os

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib import screw_boss as SB
from ..lib.futil import ATTR_GROUP, ui
from . import gx_comb
from . import polygrid_dialog as D

CMD_ID = "GXTool_ScrewBoss"
EDIT_ID = "GXTool_ScrewBossEdit"
PANEL = "SolidCreatePanel"
NAME = "Screw Boss"
SKETCH_NAME, TOP_NAME = "Screw boss sketch", "Screw boss top"
BOSS_NAME, HOLE_NAME = "Screw boss", "Screw boss hole"
ATTR = "screwBoss"          # on the boss extrude: its settings (JSON)
OF_ATTR = "screwBossOf"     # on its sketches and holes: the boss extrude's entityToken
LAST_KEY = "screwBossLast"  # design setting: the last settings used
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "screw_boss")
MM = 0.1  # in cm, Fusion's internal length unit

KINDS = {"Tap drill": SB.TAP, "Self-tapping screw": SB.SELF_TAP, "Heat-set insert": SB.INSERT,
         "Custom diameter": SB.CUSTOM}
SIZES = {s: s for s in SB.SIZES}
ENDS = {"Drill point (118°)": "point", "Flat": "flat"}
TIP = {"point": "118 deg", "flat": "180 deg"}
SIZE_BY = {"Wall thickness": SB.WALL, "Outer diameter": SB.OD}
EXTENTS = {"Distance": "distance", "To object": "to"}
BOSS_AS = {"Join": "join", "New body": "new"}
DROPDOWNS = (("kind", KINDS), ("size", SIZES), ("end", ENDS), ("sizeBy", SIZE_BY), ("extent", EXTENTS),
             ("bossAs", BOSS_AS))
VALUES = ("holeDia", "holeDepth", "wall", "od", "height", "offset")
DEFAULTS = dict(kind=SB.TAP, size="M3", end="point", sizeBy=SB.WALL, extent="distance", bossAs="join",
                holeDia="2.5 mm", holeDepth="6 mm", wall="2 mm", od="8 mm", height="10 mm", offset="0 mm",
                flip=False)
NO_TOP = ("The bosses have no top face: they run into the body they're joined to. Leave a gap with the "
          "Offset, or make them a new body.")

_dlg = None  # the open dialog


# --------------------------------------------------------------------------- geometry
def _v(p):
    return (p.x, p.y, p.z)


def _P(a):
    return adsk.core.Point3D.create(a[0], a[1], a[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit(a):
    return _mul(a, 1.0 / math.sqrt(_dot(a, a)))


def _items(coll):
    return [coll.item(k) for k in range(coll.count)] if coll else []


def _collection(items):
    coll = adsk.core.ObjectCollection.create()
    for x in items:
        coll.add(x)
    return coll


def sketch_frame(sk):
    """(origin, unit normal) of a sketch's plane, in model space."""
    o = _v(sk.sketchToModelSpace(_P((0.0, 0.0, 0.0))))
    z = _v(sk.sketchToModelSpace(_P((0.0, 0.0, 1.0))))
    return o, _unit(_sub(z, o))


def model_pt(p):
    """A sketch point's place in model space."""
    return _v(p.parentSketch.sketchToModelSpace(p.geometry))


def on_face(face, q):
    """Is model point q on the face?"""
    try:
        ev = face.evaluator
        ok, uv = ev.getParameterAtPoint(_P(q))
        if not ok or not ev.isParameterOnFace(uv):
            return False
        ok, r = ev.getPointAtParameter(uv)
        return ok and math.dist(_v(r), q) < 1e-4
    except Exception as e:
        futil.debug("  on-face test failed: %s" % e)
        return False


def target_point(ent):
    """A point on (or in) the face, plane or body the bosses go up to, or None."""
    try:
        face = adsk.fusion.BRepFace.cast(ent)
        if face:
            return _v(face.pointOnFace)
        plane = adsk.fusion.ConstructionPlane.cast(ent)
        if plane:
            return _v(plane.geometry.origin)
        bb = ent.boundingBox
        return tuple((a + b) / 2 for a, b in zip(_v(bb.minPoint), _v(bb.maxPoint)))
    except Exception as e:
        futil.debug("  couldn't tell which side the target is on: %s" % e)
        return None


def target_kind(ent):
    if adsk.fusion.BRepFace.cast(ent):
        return "face"
    if adsk.fusion.ConstructionPlane.cast(ent):
        return "plane"
    return "body"


# --------------------------------------------------------------------------- the features
def project_point(sk, p):
    """Sketch point p (in another sketch) in sketch sk: a linked projection where Fusion allows it,
    or else a plain point at the same place."""
    try:
        for e in sk.project2([p], True):
            q = adsk.fusion.SketchPoint.cast(e)
            if q:
                return q
    except Exception as e:
        futil.debug("screw boss: couldn't project a point into '%s' (%s); placing a plain one" % (sk.name, e))
    return sk.sketchPoints.add(sk.modelToSketchSpace(_P(model_pt(p))))


def draw_circles(sk, centres, dia, expr):
    """A circle of diameter dia (cm) round each centre. The first gets a diameter dimension of expr
    and the rest are equal to it, so one number sizes them all."""
    circles = sk.sketchCurves.sketchCircles
    made = [circles.addByCenterRadius(c, dia / 2) for c in centres]
    try:
        g = centres[0].geometry
        dim = sk.sketchDimensions.addDiameterDimension(made[0], _P((g.x + 0.7 * dia, g.y + 0.7 * dia, 0.0)))
        dim.parameter.expression = expr
    except Exception as e:
        futil.log("screw boss: couldn't dimension a circle in '%s' as %s: %s" % (sk.name, expr, e))
    for c in made[1:]:
        try:
            sk.geometricConstraints.addEqual(made[0], c)
        except Exception as e:
            futil.debug("  equal constraint not added: %s" % e)
    return made


def directions(cfg, o, n):
    """Extrude directions to try, best first: for a distance, the one asked for; up to an object,
    toward the side of the sketch it's on, then the other side."""
    E = adsk.fusion.ExtentDirections
    pos, neg = E.PositiveExtentDirection, E.NegativeExtentDirection
    if cfg["extent"] != "to":
        return [neg if cfg["flip"] else pos]
    q = target_point(cfg["to"])
    return [neg, pos] if q is not None and _dot(_sub(q, o), n) < 0 else [pos, neg]


def operation(cfg):
    F = adsk.fusion.FeatureOperations
    return F.NewBodyFeatureOperation if cfg["bossAs"] == "new" else F.JoinFeatureOperation


def extent(cfg):
    vin = adsk.core.ValueInput.createByString
    if cfg["extent"] == "to":
        return adsk.fusion.ToEntityExtentDefinition.create(cfg["to"], False, vin(cfg["offset"]))
    return adsk.fusion.DistanceExtentDefinition.create(vin(cfg["height"]))


def add_boss(comp, profs, cfg, o, n):
    """The bosses' Extrude."""
    exts = comp.features.extrudeFeatures
    dirs = directions(cfg, o, n)
    for k, d in enumerate(dirs):
        inp = exts.createInput(_collection(profs), operation(cfg))
        inp.setOneSideExtent(extent(cfg), d)
        try:
            ext = exts.add(inp)
        except RuntimeError as e:
            if k == len(dirs) - 1:
                raise
            futil.debug("screw boss: the extrude failed going one way (%s); trying the other" % e)
            continue
        ext.name = BOSS_NAME
        return ext


def repoint(ext, profs, cfg, o, n):
    """Point an existing boss extrude at new circles and settings (the marker must be just before it)."""
    ext.profile = _collection(profs)
    ext.operation = operation(cfg)
    ext.setOneSideExtent(extent(cfg), directions(cfg, o, n)[0])


def tops(ext, o, n, pts):
    """[(top face, points)]: a flat face the bosses end on for each plane they end on, with the
    points whose bosses end there. Raises ValueError when a top isn't flat and parallel to the sketch."""
    faces = _items(ext.endFaces)
    if not faces:
        raise ValueError(NO_TOP)
    levels = []
    for f in faces:
        pl = adsk.core.Plane.cast(f.geometry)
        if pl is None:
            raise ValueError("A boss ends on a curved face, so its hole has no flat top to start from. Stop the "
                             "bosses at a flat face or a plane.")
        if abs(_dot(_unit(_v(pl.normal)), n)) < 1 - 1e-6:
            raise ValueError("A boss's top isn't parallel to the points' sketch, so its hole would be tilted. Stop "
                             "the bosses at a face or plane parallel to the sketch.")
        levels.append(_dot(_sub(_v(pl.origin), o), n))
    groups = SB.group_levels(levels)
    if len(groups) == 1:
        return [(faces[groups[0][0]], list(pts))]
    out = [(faces[g[0]], []) for g in groups]
    for p in pts:
        q0 = model_pt(p)
        for g, (_, members) in zip(groups, out):
            q = _add(q0, _mul(n, levels[g[0]] - _dot(_sub(q0, o), n)))  # up the boss's axis to this level
            if any(on_face(faces[k], q) for k in g):
                members.append(p)
                break
        else:
            raise ValueError("Couldn't find the top of one of the bosses.")
    return [(f, m) for f, m in out if m]


def hole_feature(comp, marks, bodies, cfg, natural):
    """A Hole feature at the marks (sketch points); natural: drilling against the sketch's normal is
    into the boss."""
    vin = adsk.core.ValueInput.createByString
    holes = comp.features.holeFeatures
    hi = holes.createSimpleInput(vin(cfg["holeExpr"]))
    hi.setPositionBySketchPoints(_collection(marks))
    hi.setDistanceExtent(vin(cfg["depthExpr"]))
    hi.tipAngle = vin(TIP[cfg["end"]])
    hi.isDefaultDirection = natural
    if bodies:
        hi.participantBodies = bodies
    hole = holes.add(hi)
    hole.name = HOLE_NAME
    return hole


def cut_hole(comp, top, marks, bodies, cfg, natural):
    """A flat-bottomed hole as an extruded cut: circles round the marks, cut down by the depth."""
    draw_circles(top, marks, cfg["hole"], cfg["holeExpr"])
    E = adsk.fusion.ExtentDirections
    exts = comp.features.extrudeFeatures
    inp = exts.createInput(_collection(_items(top.profiles)), adsk.fusion.FeatureOperations.CutFeatureOperation)
    depth = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(cfg["depthExpr"]))
    inp.setOneSideExtent(depth, E.NegativeExtentDirection if natural else E.PositiveExtentDirection)
    if bodies:
        inp.participantBodies = bodies
    cut = exts.add(inp)
    cut.name = HOLE_NAME
    return cut


def add_hole(comp, face, pts, bodies, cfg, floor):
    """A top sketch on the face with the points on it, and the hole down from there. floor: a model
    point on the points' sketch plane. Returns the timeline items made."""
    top = D.add_sketch(comp, face, TOP_NAME)
    marks = [project_point(top, p) for p in pts]
    o, n = sketch_frame(top)
    natural = _dot(n, _sub(floor, o)) < 0  # Fusion drills against the sketch's normal: is that down the boss?
    try:
        hole = hole_feature(comp, marks, bodies, cfg, natural)
    except Exception as e:
        if cfg["end"] != "flat":
            raise
        futil.log("screw boss: Fusion wouldn't make a flat-bottomed Hole feature (%s); cutting the hole with an "
                  "extrude instead" % e)
        hole = cut_hole(comp, top, marks, bodies, cfg, natural)
    top.isVisible = False
    return [top, hole]


def timeline():
    d = D.design()
    if d and d.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        return d.timeline
    return None


def group(tl, first, last):
    """Group the screw boss's timeline items: its sketch through its last feature."""
    i0, i1 = first.timelineObject.index, last.timelineObject.index
    if i1 <= i0:
        futil.log("screw boss: its features aren't after its sketch in the timeline; not grouping them")
        return None
    try:
        g = tl.timelineGroups.add(i0, i1)
        g.name = NAME
        return g
    except Exception as e:
        futil.log("couldn't group the screw boss in the timeline: %s" % e)
        return None


def build(cfg):
    """Make the bosses and holes (see the module docstring) at the timeline marker.
    Returns (boss sketch, boss extrude, every timeline item made)."""
    pts = [getattr(p, "nativeObject", None) or p for p in cfg["points"]]
    src = pts[0].parentSketch
    comp = src.parentComponent
    o, n = sketch_frame(src)
    sk = D.new_sketch(src, SKETCH_NAME)
    tl = timeline()
    moved = False
    if tl is not None and tl.markerPosition != sk.timelineObject.index + 1:
        # the sketch went in earlier in the timeline (see new_sketch): put the rest right after it
        marker, at_end, count = tl.markerPosition, tl.markerPosition == tl.count, tl.count
        sk.timelineObject.rollTo(False)
        moved = True
    made = [sk]
    try:
        draw_circles(sk, [project_point(sk, p) for p in pts], cfg["od"], cfg["odExpr"])
        profs = _items(sk.profiles)
        if not profs:
            raise ValueError("The boss sketch has no profiles.")
        ext = add_boss(comp, profs, cfg, o, n)
        made.append(ext)
        bodies = _items(ext.bodies)
        for face, members in tops(ext, o, n, pts):
            made += add_hole(comp, face, members, bodies, cfg, o)
        sk.isVisible = False
    finally:
        if moved:
            D.restore_marker(tl, marker, at_end, tl.count - count)
    if tl is not None:
        group(tl, sk, made[-1])
    futil.debug("screw boss: %d boss(es) made, %d timeline item(s)" % (len(pts), len(made)))
    return sk, ext, made


def problem(e):
    """What to tell the user about a failed build."""
    if isinstance(e, ValueError):
        return str(e)
    return "Fusion couldn't make the bosses: %s" % e


# --------------------------------------------------------------------------- finding and reading a screw boss
def screw_boss_of(ent):
    """The boss extrude of the Screw Boss an entity belongs to (its extrude, sketches, holes,
    anything in its sketches, or its timeline group), or None."""
    if ent is None:
        return None
    grp = adsk.fusion.TimelineGroup.cast(ent)
    if grp:
        for k in range(grp.count):
            ext = screw_boss_of(grp.item(k).entity)
            if ext:
                return ext
        return None
    for cls in (adsk.fusion.SketchCurve, adsk.fusion.Profile, adsk.fusion.SketchPoint):
        e = cls.cast(ent)
        if e:
            ent = e.parentSketch
            break
    if D.attr(ent, ATTR):
        return adsk.fusion.ExtrudeFeature.cast(ent)
    return adsk.fusion.ExtrudeFeature.cast(D.entity(D.attr(ent, OF_ATTR)))


def tag(sk, ext, items, data):
    """Store the settings on the extrude, and point the other items at it."""
    ext.attributes.add(ATTR_GROUP, ATTR, json.dumps(data))
    tok = D.token(ext) or ""
    for x in [sk] + list(items):
        try:
            x.attributes.add(ATTR_GROUP, OF_ATTR, tok)
        except Exception as e:
            futil.debug("  couldn't tag %s: %s" % (getattr(x, "name", x), e))


def od_parameter(sk):
    """The parameter of the boss sketch's diameter dimension, or None."""
    try:
        dims = sk.sketchDimensions
        for k in range(dims.count):
            d = adsk.fusion.SketchDiameterDimension.cast(dims.item(k))
            if d:
                return d.parameter
    except Exception as e:
        futil.debug("  couldn't read the boss sketch's dimension: %s" % e)
    return None


def _changed(param, expr, value):
    """Has a parameter been changed from the expression (and value) the screw boss gave it?"""
    return param.expression != expr and (value is None or abs(param.value - value) > 1e-7)


def live_settings(ext, data):
    """The stored settings, with what's been changed since in Fusion's own dialogs on top: the
    Extrude's extent and operation, the boss sketch's diameter, and the hole's diameter, depth and tip."""
    out = dict(data)
    try:
        e1 = ext.extentOne
        dist = adsk.fusion.DistanceExtentDefinition.cast(e1)
        to = adsk.fusion.ToEntityExtentDefinition.cast(e1)
        if dist:
            out.update(extent="distance", height=dist.distance.expression)
        elif to:
            out.update(extent="to", offset=to.offset.expression, to=D.token(to.entity))
        out["bossAs"] = "new" if ext.operation == adsk.fusion.FeatureOperations.NewBodyFeatureOperation else "join"
    except Exception as e:
        futil.debug("couldn't read the screw boss's extrude (%s); using the stored settings" % e)
    od = od_parameter(adsk.fusion.Sketch.cast(D.entity(data.get("sketch"))))
    if od is not None and _changed(od, data.get("odExpr"), data.get("odValue")):
        out.update(sizeBy=SB.OD, od=od.expression)
    for x in (D.entity(t) for t in data.get("holeItems") or []):
        try:
            hole = adsk.fusion.HoleFeature.cast(x)
            cut = None if hole else adsk.fusion.ExtrudeFeature.cast(x)
            if hole:
                if _changed(hole.holeDiameter, data.get("holeExpr"), data.get("holeValue")):
                    out.update(kind=SB.CUSTOM, holeDia=hole.holeDiameter.expression)
                depth = adsk.fusion.DistanceExtentDefinition.cast(hole.extentDefinition)
                out["end"] = "flat" if abs(hole.tipAngle.value - math.pi) < 1e-6 else "point"
            elif cut:  # a flat-bottomed hole made as a cut
                depth = adsk.fusion.DistanceExtentDefinition.cast(cut.extentOne)
            else:
                continue
            if depth:
                out["holeDepth"] = depth.distance.expression
            break
        except Exception as e:
            futil.debug("couldn't read the screw boss's hole (%s); using the stored settings" % e)
    if out != data:
        futil.debug("screw boss: changed in Fusion's dialogs since: %s"
                    % {k: v for k, v in out.items() if data.get(k) != v})
    return out


# --------------------------------------------------------------------------- the dialog
class Dialog:
    """State and event handlers for one run of the command."""

    def __init__(self):
        self.inp = {}
        self.loading = False
        self.last_err = None
        self.parts = {}          # insert dropdown label -> McMaster part number
        self.ext = self.sk = None  # editing: the screw boss's extrude and sketch...
        self.data = None         # ...and its settings
        self.marker = None       # (marker position, at end, timeline count) before rolling back for the edit
        self.rolled = self.ungrouped = self.done = False

    def keep(self, inp):
        self.inp[inp.id] = inp
        return inp

    def dd(self, id_, table):
        return table[self.inp[id_].selectedItem.name]

    def created(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        d = D.design()
        units = d.unitsManager.defaultLengthUnits if d else "mm"
        vin = adsk.core.ValueInput.createByString
        df = DEFAULTS
        futil.debug("=== %s opened" % NAME)
        pre = [ui.activeSelections.item(k).entity for k in range(ui.activeSelections.count)]
        edit = next((x for x in (screw_boss_of(e) for e in pre) if x), None)

        p = self.keep(inputs.addSelectionInput("points", "Points", "Pick the sketch points where the bosses go"))
        p.addSelectionFilter("SketchPoints")
        p.setSelectionLimits(0, 0)
        p.tooltip = "Sketch points on a flat face or plane: a boss goes up from each one"
        self.keep(inputs.addTextBoxCommandInput("editing", "", "", 2, True)).isVisible = False
        self.keep(D.dropdown(inputs, "kind", "Hole for", KINDS, df["kind"])).tooltip = (
            "Tap drill: to tap an ISO thread. Self-tapping screw: a thread-forming screw for plastic (hole 0.8 × d). "
            "Heat-set insert: McMaster's brass inserts for plastic. Custom: a diameter you set.")
        self.keep(D.dropdown(inputs, "size", "Size", SIZES, df["size"]))
        self.keep(inputs.addDropDownCommandInput("insert", "Insert", adsk.core.DropDownStyles.TextListDropDownStyle)
                  ).tooltip = "McMaster-Carr 94459A brass heat-set insert: sets the hole's diameter and depth"
        self.keep(inputs.addValueInput("holeDia", "Hole Ø", units, vin(df["holeDia"]))).tooltip = (
            "Set by the size above; choose Custom diameter in 'Hole for' to set your own")
        self.keep(inputs.addValueInput("holeDepth", "Hole depth", units, vin(df["holeDepth"]))).tooltip = (
            "Measured down from the top of the boss. Picking an insert sets it to the insert's length plus room "
            "below it.")
        self.keep(D.dropdown(inputs, "end", "Hole bottom", ENDS, df["end"]))
        self.keep(D.dropdown(inputs, "sizeBy", "Boss size by", SIZE_BY, df["sizeBy"]))
        self.keep(inputs.addValueInput("wall", "Wall thickness", units, vin(df["wall"]))).tooltip = (
            "From the hole to the outside of the boss")
        self.keep(inputs.addValueInput("od", "Outer Ø", units, vin(df["od"])))
        self.keep(D.dropdown(inputs, "extent", "Extent", EXTENTS, df["extent"]))
        self.keep(inputs.addValueInput("height", "Height", units, vin(df["height"])))
        t = self.keep(inputs.addSelectionInput("to", "To", "Pick the face, plane or body the bosses go up to"))
        t.addSelectionFilter("Faces")
        t.addSelectionFilter("ConstructionPlanes")
        t.addSelectionFilter("SolidBodies")
        t.setSelectionLimits(0, 1)
        self.keep(inputs.addValueInput("offset", "Offset", units, vin(df["offset"]))).tooltip = (
            "How far from the object to stop; the sign decides whether it stops short of it or goes past it")
        self.keep(inputs.addBoolValueInput("flip", "Flip direction", True, "", df["flip"]))
        self.keep(D.dropdown(inputs, "bossAs", "Boss as", BOSS_AS, df["bossAs"])).tooltip = (
            "Join the bosses to the body they stand on, or make them a new body")
        self.keep(inputs.addTextBoxCommandInput("info", "", "Select the points for the bosses.", 4, True))
        self.fill_inserts()
        self.restore_last()
        if edit:
            self.load(edit)
        self.apply_preset(set_depth=False)
        self.refresh()
        for event, fn, cls in ((cmd.inputChanged, self.on_input_changed, adsk.core.InputChangedEventHandler),
                               (cmd.validateInputs, self.on_validate, adsk.core.ValidateInputsEventHandler),
                               (cmd.preSelect, self.on_pre_select, adsk.core.SelectionEventHandler),
                               (cmd.activate, self.on_activate, adsk.core.CommandEventHandler),
                               (cmd.executePreview, self.on_preview, adsk.core.CommandEventHandler),
                               (cmd.execute, self.on_execute, adsk.core.CommandEventHandler),
                               (cmd.destroy, self.on_destroy, adsk.core.CommandEventHandler)):
            futil.add_handler(event, fn, cls)

    # ------------------------------------------------------------- settings
    def restore_last(self):
        last = gx_comb.get_setting(LAST_KEY, "")
        if not last:
            return
        try:
            self.apply(json.loads(last))
        except Exception as e:
            futil.log("couldn't restore the last %s settings: %s" % (NAME, e))

    def apply(self, data):
        """Set the dialog from stored settings."""
        self.loading = True
        try:
            for id_, table in DROPDOWNS:
                if id_ in data:
                    D.select(self.inp[id_], table, data[id_])
            for id_ in VALUES:
                if data.get(id_):
                    D.set_expr(self.inp[id_], data[id_])
            if "flip" in data:
                self.inp["flip"].value = bool(data["flip"])
        finally:
            self.loading = False
        self.fill_inserts(data.get("insert"))

    def settings(self):
        """The dialog's settings, as stored (expressions, so user parameters keep working)."""
        data = {id_: self.dd(id_, t) for id_, t in DROPDOWNS}
        data.update({id_: self.inp[id_].expression for id_ in VALUES})
        data.update(flip=self.inp["flip"].value, insert=self.part())
        return data

    def part(self):
        """The McMaster part number of the insert picked."""
        it = self.inp["insert"].selectedItem
        return self.parts.get(it.name) if it else None

    def fill_inserts(self, part=None):
        """List the inserts of the chosen size, selecting this part, or the one selected before if
        it's this size, or else the typical one."""
        size = self.dd("size", SIZES)
        rows = SB.INSERTS[size]
        part = part or self.part()
        if part not in [r["part"] for r in rows]:
            part = SB.TYPICAL[size]
        dd = self.inp["insert"]
        self.loading = True
        try:
            dd.listItems.clear()
            self.parts = {}
            for r in rows:
                label = "%g mm long, McMaster %s" % (r["length"], r["part"])
                self.parts[label] = r["part"]
                dd.listItems.add(label, r["part"] == part, "")
        finally:
            self.loading = False

    def apply_preset(self, set_depth):
        """Put the preset's hole diameter in its box (and with set_depth, an insert's hole depth)."""
        kind, size = self.dd("kind", KINDS), self.dd("size", SIZES)
        hole = SB.preset_hole(kind, size, self.part())
        self.loading = True
        try:
            if hole is not None:
                self.inp["holeDia"].expression = "%g mm" % hole
            if set_depth and kind == SB.INSERT:
                self.inp["holeDepth"].expression = "%g mm" % SB.insert_depth(size, self.part())
        finally:
            self.loading = False

    def convert_size(self):
        """Keep the boss the same size when switching between wall thickness and outer diameter."""
        dia, wall, od = self.inp["holeDia"], self.inp["wall"], self.inp["od"]
        if not all(x.isValidExpression for x in (dia, wall, od)):
            return
        self.loading = True
        try:
            if self.dd("sizeBy", SIZE_BY) == SB.OD:
                od.value = SB.boss_od(dia.value, SB.WALL, wall.value)
            elif od.value > dia.value:
                wall.value = (od.value - dia.value) / 2
        finally:
            self.loading = False

    def refresh(self):
        kind, by, ex = self.dd("kind", KINDS), self.dd("sizeBy", SIZE_BY), self.dd("extent", EXTENTS)
        self.inp["size"].isVisible = kind != SB.CUSTOM
        self.inp["insert"].isVisible = kind == SB.INSERT
        self.inp["holeDia"].isEnabled = kind == SB.CUSTOM
        self.inp["wall"].isVisible = by == SB.WALL
        self.inp["od"].isVisible = by == SB.OD
        self.inp["height"].isVisible = self.inp["flip"].isVisible = ex == "distance"
        self.inp["to"].isVisible = self.inp["offset"].isVisible = ex == "to"
        self.inp["editing"].isVisible = self.ext is not None

    # ------------------------------------------------------------- checks
    def points(self):
        """(the picked sketch points, one per place; how many were dropped). Raises ValueError."""
        sel = self.inp["points"]
        pts = [adsk.fusion.SketchPoint.cast(sel.selection(k).entity) for k in range(sel.selectionCount)]
        pts = [p for p in pts if p]
        if not pts:
            raise ValueError("Select the sketch points where the bosses go.")
        sk = pts[0].parentSketch
        if any(p.parentSketch != sk for p in pts[1:]):
            raise ValueError("The points must all be in one sketch.")
        keep = SB.merge_points([(p.geometry.x, p.geometry.y) for p in pts])
        return [pts[k] for k in keep], len(pts) - len(keep)

    def value_errors(self):
        errs = []
        for id_ in VALUES:
            inp = self.inp[id_]
            if not inp.isVisible:
                continue
            if not inp.isValidExpression:
                errs.append("%s: that isn't a valid value." % inp.name)
            elif id_ != "offset" and inp.value <= 0:
                errs.append("%s must be above zero." % inp.name)
        return errs

    def cfg(self):
        """The settings for building: values in cm, and the expressions as typed."""
        kind, size, by = self.dd("kind", KINDS), self.dd("size", SIZES), self.dd("sizeBy", SIZE_BY)
        dia, depth, wall, od, height = (self.inp[k] for k in ("holeDia", "holeDepth", "wall", "od", "height"))
        if by == SB.WALL:
            od_v, od_expr = SB.boss_od(dia.value, by, wall.value), "(%s) + 2 * (%s)" % (dia.expression, wall.expression)
        else:
            od_v, od_expr = od.value, od.expression
        to = self.inp["to"]
        ex = self.dd("extent", EXTENTS)
        return dict(kind=kind, size=size, part=self.part() if kind == SB.INSERT else None,
                    hole=dia.value, holeExpr=dia.expression, depth=depth.value, depthExpr=depth.expression,
                    end=self.dd("end", ENDS), od=od_v, odExpr=od_expr, extent=ex, height=height.expression,
                    heightValue=height.value if ex == "distance" else None, offset=self.inp["offset"].expression,
                    flip=self.inp["flip"].value, to=to.selection(0).entity if to.selectionCount else None,
                    bossAs=self.dd("bossAs", BOSS_AS))

    def check(self):
        """(settings with the points, or None; errors)."""
        try:
            pts, merged = self.points()
        except ValueError as e:
            return None, [str(e)]
        errs = self.value_errors()
        if errs:
            return None, errs
        cfg = self.cfg()
        cfg.update(points=pts, merged=merged)
        if cfg["od"] <= cfg["hole"]:
            errs.append("The boss (Ø%s) must be wider than its hole (Ø%s)." % (D.Dialog.fmt(cfg["od"]),
                                                                              D.Dialog.fmt(cfg["hole"])))
        if cfg["extent"] == "to" and cfg["to"] is None:
            errs.append("Pick the face, plane or body the bosses go up to.")
        return cfg, errs

    @staticmethod
    def notes(cfg):
        """Warnings that don't stop it."""
        fmt = D.Dialog.fmt
        out = []
        if cfg["kind"] == SB.INSERT:
            length = SB.insert(cfg["size"], cfg["part"])["length"] * MM
            if cfg["depth"] < length - 1e-9:
                out.append("The hole is shorter than the insert (%s)." % fmt(length))
        if cfg["extent"] == "distance":
            tip = cfg["hole"] / 2 / math.tan(math.radians(59)) if cfg["end"] == "point" else 0.0
            if cfg["depth"] + tip > cfg["heightValue"] + 1e-9:
                out.append("The hole%s reaches below the boss, into what it stands on."
                           % (" (with its drill point)" if tip else ""))
        least = SB.suggested_od(cfg["kind"], cfg["size"], cfg["hole"] / MM)
        if least and cfg["od"] < least * MM - 1e-9:
            out.append("Guideline for %s: a boss at least Ø%s." % (
                "thread-forming screws" if cfg["kind"] == SB.SELF_TAP else "heat-set inserts", fmt(least * MM)))
        if cfg["merged"]:
            out.append("%d picked point(s) were on top of another; one boss for each place." % cfg["merged"])
        over = SB.overlapping([(p.geometry.x, p.geometry.y) for p in cfg["points"]], cfg["od"])
        if over:
            out.append("%d pair(s) of bosses overlap and will merge." % len(over))
        return out

    @staticmethod
    def info(cfg, notes):
        fmt = D.Dialog.fmt
        n = len(cfg["points"])
        where = (", %s tall" % fmt(cfg["heightValue"]) if cfg["extent"] == "distance"
                 else ", up to the %s" % target_kind(cfg["to"]))
        lines = ["<b>%d boss%s</b>: Ø%s (wall %s)%s" % (n, "" if n == 1 else "es", fmt(cfg["od"]),
                                                       fmt((cfg["od"] - cfg["hole"]) / 2), where),
                 "Hole Ø%s, %s deep, %s; %s" % (fmt(cfg["hole"]), fmt(cfg["depth"]),
                                                "drill point" if cfg["end"] == "point" else "flat bottom",
                                                SB.describe(cfg["kind"], cfg["size"], cfg["part"]))]
        lines += ["<span style='color:#b60'>%s</span>" % t for t in notes]
        return "<br>".join(lines)

    # ------------------------------------------------------------- events
    def on_input_changed(self, args):
        inp = args.input
        futil.debug("%s input '%s' = %s" % (NAME, inp.id, futil.describe_input(inp)))
        if self.loading:
            return
        id_ = inp.id
        if id_ == "size":
            self.fill_inserts()
        if id_ in ("kind", "size", "insert"):
            self.apply_preset(set_depth=True)
        elif id_ == "sizeBy":
            self.convert_size()
        self.refresh()

    def on_validate(self, args):
        _, errs = self.check()
        err = "<br>".join(errs)
        args.areInputsValid = not err
        if err != self.last_err:
            self.last_err = err
            futil.debug("%s validate: %s" % (NAME, err or "ok"))
        if err:
            self.inp["info"].formattedText = "<span style='color:#c33'>%s</span>" % err

    def on_preview(self, args):
        cfg, errs = self.check()
        if errs or not cfg:
            return
        try:
            build(cfg)
        except Exception as e:
            futil.debug("%s preview failed: %s" % (NAME, e))
            self.inp["info"].formattedText = "<span style='color:#c33'>%s</span>" % problem(e)
            return
        self.inp["info"].formattedText = self.info(cfg, self.notes(cfg))
        args.isValidResult = False  # execute builds the real result

    def on_execute(self, args):
        cfg, errs = self.check()
        if errs or not cfg:
            ui.messageBox("\n".join(errs) or "Nothing to make.")
            return
        try:
            if self.ext is not None:
                done = self.update(cfg)
                if not done:
                    return
                sk, ext, made = done
            else:
                sk, ext, made = build(cfg)
        except Exception as e:
            futil.handle_error(NAME, show=False)
            args.executeFailed = True
            args.executeFailedMessage = problem(e)
            return
        data = self.settings()
        data.update(v=1, sketch=D.token(sk), points=[D.token(p) for p in cfg["points"]],
                    to=D.token(cfg["to"]) if cfg["to"] else None, holeItems=[D.token(x) for x in made[2:]],
                    odExpr=cfg["odExpr"], odValue=cfg["od"], holeExpr=cfg["holeExpr"], holeValue=cfg["hole"])
        try:
            tag(sk, ext, made[2:], data)
        except Exception as e:
            futil.log("couldn't store the screw boss's settings: %s" % e)
        gx_comb.set_setting(LAST_KEY, json.dumps(self.settings()))
        self.done = True
        futil.debug("screw boss %s" % ("updated" if self.ext is not None else "made"))

    def on_destroy(self, args):
        futil.debug("=== %s closed (reason: %s)" % (NAME, args.terminationReason))
        self.closed()

    # ------------------------------------------------------------- editing
    def load(self, ext):
        """Edit an existing screw boss: its stored settings, with changes made since in Fusion's own
        dialogs on top. The selections are restored on activate."""
        data = live_settings(ext, json.loads(D.attr(ext, ATTR)))
        self.ext, self.data = ext, data
        self.sk = adsk.fusion.Sketch.cast(D.entity(data.get("sketch")))
        if self.sk is None:
            try:
                prof = ext.profile
                prof = adsk.fusion.Profile.cast(prof) or adsk.fusion.Profile.cast(prof.item(0))
                self.sk = prof.parentSketch
            except Exception as e:
                futil.log("couldn't find the screw boss's sketch: %s" % e)
        futil.debug("editing screw boss: %s" % data)
        self.apply(data)
        self.inp["editing"].formattedText = "Editing <b>%s</b>. OK updates it; Cancel leaves it as it was." % NAME

    def on_activate(self, args):
        if self.ext is None:
            return
        tl = timeline()
        if tl is not None and self.sk is not None:
            self.marker = (tl.markerPosition, tl.markerPosition == tl.count, tl.count,
                           self.sk.timelineObject.index < tl.markerPosition)
            try:
                self.sk.timelineObject.rollTo(True)
            except Exception as e:
                futil.debug("couldn't roll into the screw boss's group (%s); ungrouping it" % e)
                self.ungroup()
                self.sk.timelineObject.rollTo(True)
            self.rolled = True
            args.command.beginStep()  # keep the roll while previews come and go
        problem = self.restore_selections()
        if problem:
            self.inp["editing"].formattedText = problem
        self.refresh()

    def restore_selections(self):
        """Select the stored points and object again. Returns a problem to show, or ''."""
        problems = []
        self.loading = True
        try:
            sel = self.inp["points"]
            sel.clearSelection()
            toks = self.data.get("points") or []
            found = [e for e in (D.entity(t) for t in toks) if e]
            for e in found:
                sel.addSelection(e)
            if len(found) < len(toks):
                problems.append("Some of the bosses' points are gone. Select the points again.")
            to = self.inp["to"]
            to.clearSelection()
            e = D.entity(self.data.get("to"))
            if e:
                to.addSelection(e)
            elif self.data.get("extent") == "to":
                problems.append("The object the bosses went up to is gone. Pick it again.")
        finally:
            self.loading = False
        return " ".join(problems)

    def ungroup(self):
        try:
            g = self.ext.timelineObject.parentGroup
            if g:
                g.deleteMe(False)
                self.ungrouped = True
        except Exception as e:
            futil.log("couldn't ungroup the screw boss: %s" % e)

    def restore_marker(self, tl):
        """Put the timeline marker back where it was before the edit."""
        if self.marker:
            pos, at_end, count, before = self.marker
            D.restore_marker(tl, pos, at_end, tl.count - count if before else 0)
        self.rolled = False

    def on_pre_select(self, args):
        inp = getattr(args, "activeInput", None)
        if inp is not None and inp.id == "points" and screw_boss_of(args.selection.entity):
            args.isSelectable = False  # not the points in a screw boss's own sketches

    def update(self, cfg):
        """Edit OK: redraw the boss sketch, point the same extrude at it, and make the tops and holes
        again. Returns (sketch, extrude, every timeline item), or None if it can't be edited."""
        sk, ext = self.sk, self.ext
        if sk is None or not sk.isValid or not ext.isValid:
            ui.messageBox("This Screw Boss's sketch or extrude is missing, so it can't be edited. Delete it and make "
                          "a new one.")
            return None
        pts = [getattr(p, "nativeObject", None) or p for p in cfg["points"]]
        src = pts[0].parentSketch
        if not fusion_geom.sketch_map(src, sk)[1]:
            ui.messageBox("The points aren't in the plane of this Screw Boss's sketch. Make a new Screw Boss for "
                          "them instead.")
            return None
        comp = sk.parentComponent
        o, n = sketch_frame(src)
        tl = timeline()
        if tl is not None and not self.ungrouped:
            self.ungroup()
        made = [sk, ext]
        try:
            for tok in reversed(self.data.get("holeItems") or []):  # the old tops and holes: made again below
                old = D.entity(tok)
                if old is not None:
                    try:
                        old.deleteMe()
                    except Exception as e:
                        futil.log("couldn't delete an old screw boss top or hole: %s" % e)
            if tl is not None:
                ext.timelineObject.rollTo(True)  # between the boss sketch and its extrude
            fusion_geom.clear_sketch(sk)
            draw_circles(sk, [project_point(sk, p) for p in pts], cfg["od"], cfg["odExpr"])
            profs = _items(sk.profiles)
            if not profs:
                raise ValueError("The boss sketch has no profiles.")
            try:
                repoint(ext, profs, cfg, o, n)
            except Exception as e:
                futil.log("couldn't update the screw boss's extrude (%s); making a new one" % e)
                ext.deleteMe()
                ext = self.ext = made[1] = add_boss(comp, profs, cfg, o, n)
            if tl is not None:
                ext.timelineObject.rollTo(False)  # just after the extrude: the holes go there
            bodies = _items(ext.bodies)
            for face, members in tops(ext, o, n, pts):
                made += add_hole(comp, face, members, bodies, cfg, o)
        finally:
            if tl is not None:
                self.restore_marker(tl)
                if self.ext.timelineObject.parentGroup is None:
                    group(tl, sk, made[-1])
                self.ungrouped = False
        return sk, self.ext, made

    def closed(self):
        """Cancelled (or not updated): put the timeline back as it was."""
        if self.done or self.ext is None:
            return
        tl = timeline()
        try:
            if self.rolled and tl is not None:
                self.restore_marker(tl)
            if self.ungrouped and tl is not None and self.ext.isValid and self.ext.timelineObject.parentGroup is None:
                items = [x for x in (D.entity(t) for t in self.data.get("holeItems") or []) if x]
                group(tl, self.sk, items[-1] if items else self.ext)
        except Exception as e:
            futil.log("couldn't put the timeline back after cancelling: %s" % e)


def _marking_menu(args):
    """Show Edit Screw Boss in the right-click menu when part of a screw boss is selected."""
    try:
        show = any(screw_boss_of(e) for e in list(args.selectedEntities or []))
        ctrls = args.linearMarkingMenu.controls
        c = ctrls.itemById(EDIT_ID)
        if c:
            c.isVisible = show
        elif show:
            ctrls.addCommand(ui.commandDefinitions.itemById(EDIT_ID))
    except Exception:
        futil.handle_error("screw boss marking menu", show=False)


def _on_created(args):
    global _dlg
    _dlg = Dialog()
    _dlg.created(args)


def start():
    futil.create_button(CMD_ID, NAME,
                        "Bosses for screws at sketch points on a flat face, with a hole for a tapped thread, a "
                        "self-tapping screw or a heat-set insert (M2 to M10), or a diameter you set. Select a Screw "
                        "Boss and run it again to edit it.",
                        RESOURCES, _on_created, panel_id=PANEL)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
    d = ui.commandDefinitions.addButtonDefinition(EDIT_ID, "Edit " + NAME, "Change this Screw Boss's settings",
                                                  RESOURCES)
    futil.add_handler(d.commandCreated, _on_created, adsk.core.CommandCreatedEventHandler)
    futil.add_handler(ui.markingMenuDisplaying, _marking_menu, adsk.core.MarkingMenuEventHandler)


def stop():
    futil.delete_button(CMD_ID, panel_id=PANEL)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
