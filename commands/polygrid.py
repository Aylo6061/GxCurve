"""'Polygrid' command (Solid > Modify): cuts or extrudes a regular grid of polygonal pockets
(triangles, squares, hexagons or rectangles) separated by ribs, over a sketch area or outline.

Like GX Edge Fillet, it is made of ordinary features, grouped in the timeline as "Polygrid":
  1. "Polygrid sketch": the grid, on the area's plane (the pockets; for rib operations, the area's
     outline as well, so the rib web is a profile).
  2. "Polygrid": one Extrude of the pocket profiles (Cut holes / Extrude holes) or of the rib-web
     profiles (Cut ribs / Extrude ribs), into the picked bodies, a distance, through all, or to a
     face with an offset.
  3. Optionally, a bottom blend where the walls meet the floor (or the body below): a radius or
     curvature (G2) Fillet, or swept G3/G4 rings (see polygrid_bottom.py).
Its settings are stored on the extrude (attribute GX_tool/polygrid).

Editing: select the Polygrid (its group, extrude or sketch) and run the command, or right-click >
Edit Polygrid. The timeline rolls back to just before it while the dialog is open. OK redraws the
sketch and points the *same* Extrude at the new profiles, so later features keep their references
wherever Fusion can resolve them. Like GX Edge Fillet, it doesn't refit by itself when the area
changes: edit it and click OK.
"""

import json
import os

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib import polygrid as W
from ..lib.futil import ATTR_GROUP, ui
from . import polygrid_bottom as B
from . import polygrid_dialog as D

CMD_ID = "GXTool_Polygrid"
EDIT_ID = "GXTool_PolygridEdit"
PANEL = "SolidModifyPanel"
NAME = "Polygrid"
SKETCH_NAME = "Polygrid sketch"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "polygrid")

OPS = {"Cut holes": "cut_holes", "Cut ribs": "cut_ribs", "Extrude holes": "extrude_holes",
       "Extrude ribs": "extrude_ribs"}
EXTRUDE_AS = {"Join": "join", "New body": "new"}
EXTENTS = {"Distance": "distance", "Through all": "all", "To face": "face"}

_dlg = None  # the open dialog


def ribs(op):
    return op.endswith("ribs")


def cutting(op):
    return op.startswith("cut")


# --------------------------------------------------------------------------- the features
def operation(cfg):
    F = adsk.fusion.FeatureOperations
    if cutting(cfg["op"]):
        return F.CutFeatureOperation
    return F.NewBodyFeatureOperation if cfg["extrudeAs"] == "new" else F.JoinFeatureOperation


def extent(cfg):
    vin = adsk.core.ValueInput.createByString
    if cfg["extent"] == "all":
        return adsk.fusion.ThroughAllExtentDefinition.create()
    if cfg["extent"] == "face":
        return adsk.fusion.ToEntityExtentDefinition.create(cfg["face"], False, vin(cfg["offset"]))
    return adsk.fusion.DistanceExtentDefinition.create(vin(cfg["depth"]))


def direction(cfg):
    E = adsk.fusion.ExtentDirections
    return E.NegativeExtentDirection if cfg["flip"] else E.PositiveExtentDirection


def _collection(items):
    coll = adsk.core.ObjectCollection.create()
    for x in items:
        coll.add(x)
    return coll


def add_extrude(comp, profs, cfg):
    exts = comp.features.extrudeFeatures
    op = operation(cfg)
    inp = exts.createInput(_collection(profs), op)
    inp.setOneSideExtent(extent(cfg), direction(cfg))
    if op != adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
        inp.participantBodies = list(cfg["bodies"])
    ext = exts.add(inp)
    ext.name = NAME
    return ext


def repoint(ext, profs, cfg):
    """Point an existing extrude at new profiles and settings (the marker must be just before it)."""
    ext.profile = _collection(profs)
    op = operation(cfg)
    ext.operation = op
    ext.setOneSideExtent(extent(cfg), direction(cfg))
    if op != adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
        ext.participantBodies = list(cfg["bodies"])


def grid_profiles(sk, res, src, rib_op):
    """The profiles of the grid sketch to extrude: the pockets, or the rib web (inside the area
    but in no pocket; a hole in the area isn't rib)."""
    back, _ = fusion_geom.sketch_map(sk, src)  # grid sketch -> the area sketch's space, where res lives
    in_pocket, in_area = W.pocket_finder(res), W.area_finder(res)
    out = []
    for k in range(sk.profiles.count):
        p = sk.profiles.item(k)
        q = fusion_geom.profile_point(p)
        if q is None:
            continue
        q = back(q)
        if (not in_pocket(q) and in_area(q)) if rib_op else in_pocket(q):
            out.append(p)
    futil.debug("polygrid: %d of %d profiles are %s" % (len(out), sk.profiles.count, "rib web" if rib_op else "pockets"))
    return out


def group(tl, sk, last):
    """Group the polygrid's timeline items: its sketch through its last feature."""
    i0, i1 = sk.timelineObject.index, last.timelineObject.index
    if i1 <= i0:
        futil.log("polygrid: its features aren't after its sketch in the timeline; not grouping them")
        return None
    try:
        g = tl.timelineGroups.add(i0, i1)
        g.name = NAME
        return g
    except Exception as e:
        futil.log("couldn't group the polygrid in the timeline: %s" % e)
        return None


def timeline():
    d = D.design()
    if d and d.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        return d.timeline
    return None


def draw_grid(dlg, sk, res, cfg, f):
    """Draw the grid into sk. Returns the curves of each pocket outline (in res order), or None
    when the outlines aren't closed in the sketch (ribs with no border: the area's outline closes them)."""
    border = dlg.inp["border"].value
    drawn = dlg.draw(sk, res, ribs(cfg["op"]), border, f)
    if ribs(cfg["op"]) and border <= 0:
        return None
    return drawn[:sum(len(pk["loops"]) for pk in res["pockets"])]


def add_bottom(sk, ext, res, cfg, f, back, loop_ents):
    """The bottom blend's features, made right after the extrude (the marker must be there)."""
    if cfg["bottom"] == "none" or not B.applies(cfg):
        return [], {}
    bodies = [D.entity(D.token(b)) or b for b in cfg["bodies"]]  # as they are after the extrude
    try:
        return B.build(sk.parentComponent, sk, ext, res, cfg, f, back, loop_ents, bodies)
    except Exception as e:
        futil.handle_error("polygrid bottom blend", show=False)
        return [], dict(error=str(e))


def build_new(dlg, res, cfg, bottom=True):
    """A new grid sketch + extrude (+ bottom blend, + timeline group) at the marker.
    Returns (sketch, extrude, bottom features, bottom notes)."""
    src = res["src"]
    native = src.nativeObject or src
    sk = D.new_sketch(native, SKETCH_NAME)
    f, _ = fusion_geom.sketch_map(native, sk)
    back, _ = fusion_geom.sketch_map(sk, native)
    loop_ents = draw_grid(dlg, sk, res, cfg, f)
    profs = grid_profiles(sk, res, native, ribs(cfg["op"]))
    if not profs:
        raise ValueError("The grid's sketch has no %s profiles." % ("rib" if ribs(cfg["op"]) else "pocket"))
    tl = timeline()
    moved = False
    if tl is not None and tl.markerPosition != sk.timelineObject.index + 1:
        # the sketch went in earlier in the timeline (see new_sketch): put the rest right after it
        marker, at_end, count = tl.markerPosition, tl.markerPosition == tl.count, tl.count
        sk.timelineObject.rollTo(False)
        moved = True
    items = []
    try:
        ext = add_extrude(sk.parentComponent, profs, cfg)
        items, notes = add_bottom(sk, ext, res, cfg, f, back, loop_ents) if bottom else ([], {})
    finally:
        if moved:
            D.restore_marker(tl, marker, at_end, tl.count - count)
    if tl is not None:
        group(tl, sk, items[-1] if items else ext)
    return sk, ext, items, notes


# --------------------------------------------------------------------------- the dialog
class SolidDialog(D.Dialog):
    name = NAME
    dropdowns = D.GRID_DROPDOWNS + (("op", OPS), ("extrudeAs", EXTRUDE_AS), ("extent", EXTENTS),
                                    ("bottom", B.STYLES))
    values = D.GRID_VALUES + ("depth", "offset", "bottomRadius")
    can_be_negative = ("offset",)
    defaults = dict(D.GRID_DEFAULTS, op="cut_holes", extrudeAs="join", extent="distance", depth="5 mm",
                    offset="0 mm", flip=False, bottom="none", bottomRadius="1 mm")
    last_key = "polygridLast"

    def __init__(self):
        super().__init__()
        self.ext = None          # editing: the polygrid's extrude...
        self.sk = None           # ...and sketch
        self.data = None         # ...and its stored settings
        self.marker = None       # (marker position, at end) before rolling back for the edit
        self.rolled = self.ungrouped = False

    def bools(self):
        return ("flip",)

    def created(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        d = D.design()
        units = d.unitsManager.defaultLengthUnits if d else "mm"
        futil.debug("=== %s opened" % NAME)
        pre = [ui.activeSelections.item(k).entity for k in range(ui.activeSelections.count)]
        edit = next((obj for kind, obj in (D.polygrid_of(e) for e in pre) if kind == "solid"), None)
        df = self.defaults
        vin = adsk.core.ValueInput.createByString

        self.add_area(inputs, "Pick the sketch regions or outline to fill")
        self.keep(D.dropdown(inputs, "op", "Operation", OPS, df["op"])).tooltip = (
            "Cut the pockets out, cut the ribs away, or extrude the pockets or the ribs as solid")
        b = self.keep(inputs.addSelectionInput("bodies", "Bodies", "Pick the bodies to cut or join"))
        b.addSelectionFilter("SolidBodies")
        b.setSelectionLimits(0, 0)
        self.keep(D.dropdown(inputs, "extrudeAs", "Extrude as", EXTRUDE_AS, df["extrudeAs"]))
        self.keep(D.dropdown(inputs, "extent", "Extent", EXTENTS, df["extent"]))
        self.keep(inputs.addValueInput("depth", "Depth", units, vin(df["depth"])))
        f = self.keep(inputs.addSelectionInput("face", "To face", "Pick the face to stop at"))
        f.addSelectionFilter("Faces")
        f.addSelectionFilter("ConstructionPlanes")
        f.setSelectionLimits(0, 1)
        self.keep(inputs.addValueInput("offset", "Offset", units, vin(df["offset"]))).tooltip = (
            "How far from the face to stop; the sign decides whether it stops short of the face or goes past it")
        self.keep(inputs.addBoolValueInput("flip", "Flip direction", True, "", df["flip"]))
        self.keep(D.dropdown(inputs, "bottom", "Bottom", B.STYLES, df["bottom"])).tooltip = (
            "Round where the walls meet the floor (or the body below): a radius, a curvature-continuous (G2) "
            "fillet, or a G3 / G4 curve swept around each pocket (falls back to G2 where the sweep fails)")
        self.keep(inputs.addValueInput("bottomRadius", "Bottom radius", units, vin(df["bottomRadius"])))
        self.add_grid(inputs, units)
        self.keep(inputs.addBoolValueInput("previewSolid", "Preview solid", True, "", False)).tooltip = (
            "Show the real cut or extrude while the dialog is open (slower on big grids)")
        self.add_info(inputs)
        self.restore_last()
        if edit:
            self.load(edit)
        self.refresh()
        self.connect(cmd)

    def editing(self):
        return self.ext is not None

    def load(self, ext):
        """Edit an existing polygrid: its stored settings, with the extrude's own values on top (it
        may have been changed in Fusion's Extrude dialog). Selections are restored on activate."""
        data = json.loads(D.attr(ext, D.SOLID_ATTR))
        self.ext = ext
        self.sk = adsk.fusion.Sketch.cast(D.entity(data.get("sketch")))
        if self.sk is None:
            try:
                prof = ext.profile
                prof = adsk.fusion.Profile.cast(prof) or adsk.fusion.Profile.cast(prof.item(0))
                self.sk = prof.parentSketch
            except Exception as e:
                futil.log("couldn't find the polygrid's sketch: %s" % e)
        try:
            e1 = ext.extentOne
            dist = adsk.fusion.DistanceExtentDefinition.cast(e1)
            to = adsk.fusion.ToEntityExtentDefinition.cast(e1)
            if dist:
                data.update(extent="distance", depth=dist.distance.expression)
            elif to:
                data.update(extent="face", offset=to.offset.expression, face=D.token(to.entity))
            elif adsk.fusion.ThroughAllExtentDefinition.cast(e1):
                data["extent"] = "all"
            data["bodies"] = [D.token(x) for x in ext.participantBodies] or data.get("bodies") or []
        except Exception as e:
            futil.debug("couldn't read the extrude's own settings (%s); using the stored ones" % e)
        futil.debug("editing polygrid: %s" % data)
        self.apply(data)
        self.data = data
        self.inp["editing"].formattedText = "Editing <b>%s</b>. OK updates it; Cancel leaves it as it was." % NAME

    def on_activate(self, args):
        if self.ext is None:
            return
        tl = timeline()
        if tl is not None and self.sk is not None:
            self.marker = (tl.markerPosition, tl.markerPosition == tl.count)
            try:
                self.sk.timelineObject.rollTo(True)
            except Exception as e:
                futil.debug("couldn't roll into the polygrid's group (%s); ungrouping it" % e)
                self.ungroup()
                self.sk.timelineObject.rollTo(True)
            self.rolled = True
            args.command.beginStep()  # keep the roll while previews come and go
        problem = self.restore_area(self.data)
        self.loading = True
        try:
            for id_, toks in (("bodies", self.data.get("bodies") or []), ("face", [self.data.get("face")])):
                sel = self.inp[id_]
                sel.clearSelection()
                for e in (D.entity(t) for t in toks if t):
                    if e:
                        sel.addSelection(e)
        finally:
            self.loading = False
        if problem:
            self.inp["editing"].formattedText = problem
        self.key = None
        self.refresh()

    def ungroup(self):
        try:
            g = self.ext.timelineObject.parentGroup
            if g:
                g.deleteMe(False)
                self.ungrouped = True
        except Exception as e:
            futil.log("couldn't ungroup the polygrid: %s" % e)

    def refresh_more(self):
        op, ex = self.dd("op", OPS), self.dd("extent", EXTENTS)
        self.inp["extrudeAs"].isVisible = not cutting(op)
        self.inp["bodies"].isVisible = cutting(op) or self.dd("extrudeAs", EXTRUDE_AS) == "join"
        self.inp["depth"].isVisible = ex == "distance"
        self.inp["face"].isVisible = self.inp["offset"].isVisible = ex == "face"
        bottom = B.applies(dict(op=op, extent=ex, extrudeAs=self.dd("extrudeAs", EXTRUDE_AS)))
        self.inp["bottom"].isVisible = bottom
        self.inp["bottomRadius"].isVisible = bottom and self.dd("bottom", B.STYLES) != "none"

    def cfg(self):
        sel = self.inp["bodies"]
        face = self.inp["face"]
        return dict(op=self.dd("op", OPS), extrudeAs=self.dd("extrudeAs", EXTRUDE_AS),
                    extent=self.dd("extent", EXTENTS), depth=self.inp["depth"].expression,
                    offset=self.inp["offset"].expression, flip=self.inp["flip"].value,
                    bodies=[sel.selection(k).entity for k in range(sel.selectionCount)] if sel.isVisible else [],
                    face=face.selection(0).entity if face.selectionCount else None,
                    bottom=self.dd("bottom", B.STYLES) if self.inp["bottom"].isVisible else "none",
                    bottomR=self.inp["bottomRadius"].value or 0.0, bottomRexpr=self.inp["bottomRadius"].expression)

    def extra_errors(self):
        cfg = self.cfg()
        errs = []
        if self.inp["bodies"].isVisible and not cfg["bodies"]:
            errs.append("Pick the bodies to %s." % ("cut" if cutting(cfg["op"]) else "join to"))
        if cfg["extent"] == "face" and cfg["face"] is None:
            errs.append("Pick the face to stop at.")
        if cfg["bottom"] != "none" and cfg["extent"] == "distance" and self.inp["depth"].isValidExpression \
                and cfg["bottomR"] > 0:
            reach, depth = W.bottom_reach(cfg["bottomR"], cfg["bottom"]), self.inp["depth"].value
            if reach >= depth:
                errs.append("The bottom blend reaches %s up the wall, more than the depth (%s)."
                            % (self.fmt(reach), self.fmt(depth)))
        return errs

    def info(self, res):
        txt = super().info(res)
        op = self.dd("op", OPS)
        what = {"cut_holes": "Cuts the pockets", "cut_ribs": "Cuts the ribs away",
                "extrude_holes": "Extrudes the pockets", "extrude_ribs": "Extrudes the ribs"}[op]
        txt += "<br>" + what + {"distance": "", "all": ", through all", "face": ", to the face"}[
            self.dd("extent", EXTENTS)]
        cfg = self.cfg()
        if cfg["bottom"] in B.SWEPT and cfg["bottomR"] > 0:
            loops = W.blend_loops(res, cfg["op"])
            n_shapes = len(W.shape_groups([lp for _, _, lp, _ in loops]))
            txt += "<br>Bottom: %s, swept around %d distinct outline(s), copied to the other %d" % (
                cfg["bottom"].upper(), n_shapes, len(loops) - n_shapes)
            reach = W.bottom_reach(cfg["bottomR"], cfg["bottom"])
            corner = self.inp["corner"].value or 0.0
            if cfg["op"] in B.INTO_POCKET and corner < reach:
                txt += ("<br><span style='color:#b60'>The pocket corners (radius %s) are tighter than the blend "
                        "(reaches %s): sweeps there may fail and fall back to G2.</span>"
                        % (self.fmt(corner), self.fmt(reach)))
        elif cfg["bottom"] in ("radius", "g2"):
            txt += "<br>Bottom: %s fillet" % ("radius" if cfg["bottom"] == "radius" else "curvature (G2)")
        return txt

    def preview(self, args, res):
        cfg = self.cfg()
        self.preview_outlines(res, ribs(cfg["op"]))
        if self.inp["previewSolid"].value and not self.extra_errors():
            try:  # (Fusion takes it away again; swept G3/G4 bottoms are left out, they're slow)
                build_new(self, res, cfg, bottom=cfg["bottom"] in ("radius", "g2"))
            except Exception as e:
                futil.debug("solid preview failed: %s" % e)

    def on_execute(self, args):
        self.clear_preview()
        self.key = None
        res, err = self.compute()
        err = err or "<br>".join(self.extra_errors())
        if err or not res:
            ui.messageBox((err or "Nothing to make.").replace("<br>", "\n"))
            return
        cfg = self.cfg()
        if self.ext is not None:
            done = self.update(res, cfg)
            if not done:
                return
            sk, ext, items, notes = done
        else:
            sk, ext, items, notes = build_new(self, res, cfg)
        problems = []
        if notes.get("g2"):
            problems.append("%d pocket outline(s) couldn't take a swept %s bottom, so they got a G2 fillet instead%s"
                            % (notes["g2"], cfg["bottom"].upper(),
                               (": " + notes["why"]) if notes.get("why") else "."))
        if notes.get("error"):
            problems.append("The bottom blend %sfailed: %s" % ("fillet " if notes.get("g2") else "", notes["error"]))
        if problems:
            ui.messageBox("The Polygrid was made, but:\n\n" + "\n\n".join(problems)
                          + "\n\nThe details are in logs/gx_tool.log.")
        data = self.settings()
        data.update(v=1, sketch=D.token(sk), bodies=[D.token(x) for x in cfg["bodies"]],
                    face=D.token(cfg["face"]) if cfg["face"] else None, bottomItems=[D.token(x) for x in items],
                    **self.area_record(res))
        ext.attributes.add(ATTR_GROUP, D.SOLID_ATTR, json.dumps(data))
        sk.attributes.add(ATTR_GROUP, D.SOLID_SKETCH_ATTR, D.token(ext) or "")
        self.done = True
        self.save_last(data)
        futil.debug("polygrid %s" % ("updated" if self.ext is not None else "made"))

    def update(self, res, cfg):
        """Edit OK: redraw the polygrid's sketch and repoint its extrude. Returns (sketch, extrude)."""
        sk, ext = self.sk, self.ext
        if sk is None or not sk.isValid:
            ui.messageBox("This Polygrid's sketch is missing, so it can't be edited. Delete it and make a new one.")
            return None
        src = res["src"]
        native = src.nativeObject or src
        f, coplanar = fusion_geom.sketch_map(native, sk)
        if not coplanar:
            ui.messageBox("The area isn't in the plane of the Polygrid's sketch. Make a new Polygrid for it instead.")
            return None
        back, _ = fusion_geom.sketch_map(sk, native)
        tl = timeline()
        if not self.ungrouped:
            self.ungroup()
        items, notes = [], {}
        try:
            for tok in reversed(self.data.get("bottomItems") or []):  # the old bottom blend: made again below
                old = D.entity(tok)
                if old is not None:
                    try:
                        old.deleteMe()
                    except Exception as e:
                        futil.log("couldn't delete an old bottom feature: %s" % e)
            ext.timelineObject.rollTo(True)  # between the grid sketch and its extrude
            fusion_geom.clear_sketch(sk)
            loop_ents = draw_grid(self, sk, res, cfg, f)
            profs = grid_profiles(sk, res, native, ribs(cfg["op"]))
            if not profs:
                raise ValueError("The grid's sketch has no profiles to extrude.")
            try:
                repoint(ext, profs, cfg)
            except Exception as e:
                futil.log("couldn't update the polygrid's extrude (%s); making a new one" % e)
                ext.deleteMe()
                ext = add_extrude(sk.parentComponent, profs, cfg)
                self.ext = ext
            self.ext.timelineObject.rollTo(False)  # just after the extrude: the bottom blend goes there
            items, notes = add_bottom(sk, self.ext, res, cfg, f, back, loop_ents)
        finally:
            if tl is not None:
                if self.marker:
                    D.restore_marker(tl, *self.marker)
                self.rolled = False
                if self.ext.timelineObject.parentGroup is None:
                    group(tl, sk, items[-1] if items else self.ext)
                self.ungrouped = False
        return sk, self.ext, items, notes

    def closed(self):
        if self.done:
            return
        tl = timeline()
        try:
            if self.rolled and tl is not None and self.marker:
                D.restore_marker(tl, *self.marker)
            if self.ungrouped and self.ext is not None and self.ext.isValid and \
                    self.ext.timelineObject.parentGroup is None and tl is not None:
                group(tl, self.sk, self.ext)
        except Exception as e:
            futil.log("couldn't put the timeline back after cancelling: %s" % e)


def _on_created(args):
    global _dlg
    _dlg = SolidDialog()
    _dlg.created(args)


def start():
    futil.create_button(CMD_ID, NAME,
                        "Cut or extrude a grid of triangle, square, hexagon or rectangle pockets, separated by ribs, "
                        "over a sketch area. Select a Polygrid and run it again to edit it.",
                        RESOURCES, _on_created, panel_id=PANEL)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
    d = ui.commandDefinitions.addButtonDefinition(EDIT_ID, "Edit " + NAME, "Change this Polygrid's settings", RESOURCES)
    futil.add_handler(d.commandCreated, _on_created, adsk.core.CommandCreatedEventHandler)
    futil.add_handler(ui.markingMenuDisplaying, D.marking_menu(EDIT_ID, "solid"), adsk.core.MarkingMenuEventHandler)


def stop():
    D.Dialog.clear_preview()
    futil.delete_button(CMD_ID, panel_id=PANEL)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
