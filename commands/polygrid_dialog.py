"""What Sketch Polygrid and Polygrid share: the area input (sketch regions or an outline), the grid
settings, the outline preview, and drawing a grid into a new sketch. Geometry: lib/polygrid.py.

Each run of either command gets one Dialog (subclassed in sketch_polygrid.py / polygrid.py); its
methods are the command's event handlers.
"""

import json
import math
import time

import adsk.core
import adsk.fusion

from ..lib import comb_graphics, fusion_geom, futil
from ..lib import polygrid as W
from ..lib.futil import ATTR_GROUP, app, ui
from . import gx_comb

SHAPES = {"Triangle": W.TRIANGLE, "Square": W.SQUARE, "Hexagon": W.HEXAGON, "Rectangle": W.RECTANGLE}
SIZE_BY = {"Side length": W.SIDE, "Inscribed circle diameter": W.INSCRIBED}
PHASES = {"Cell centres": W.CELL, "Ribs": W.RIB}
STYLES = {"Arc": "arc", "G3 curve (7 points)": "g3", "G4 curve (9 points)": "g4"}
GRID_DROPDOWNS = (("shape", SHAPES), ("sizeBy", SIZE_BY), ("cornerStyle", STYLES), ("vphase", PHASES),
                  ("hphase", PHASES))
GRID_VALUES = ("side", "inscribed", "length", "width", "rib", "border", "corner")
GRID_DEFAULTS = dict(shape=W.SQUARE, sizeBy=W.SIDE, side="20 mm", inscribed="20 mm", length="30 mm", width="15 mm",
                     rib="2 mm", border="2 mm", corner="0 mm", cornerStyle="arc", rot90=False, vphase=W.CELL,
                     hphase=W.CELL, minPartial=10)
POCKET_RGB, OUTLINE_RGB = (30, 110, 220), (140, 140, 140)
PREVIEW_KEY, OUTLINE_KEY = "polygrid_preview", "polygrid_outline"
PROGRESS_FROM = 300  # loops: show a progress bar while drawing

SKETCH_ATTR = "sketchPolygrid"        # on a Sketch Polygrid's sketch: its settings (JSON)
SOLID_ATTR = "polygrid"               # on a Polygrid's extrude: its settings (JSON)
SOLID_SKETCH_ATTR = "polygridSketch"  # on a Polygrid's sketch: its extrude's entityToken


def design():
    return adsk.fusion.Design.cast(app.activeProduct)


def token(e):
    try:
        return e.entityToken
    except Exception:
        return None


def entity(tok):
    """The live entity for a stored entityToken, or None."""
    d = design()
    if not (d and tok):
        return None
    try:
        for e in d.findEntityByToken(tok) or []:
            if e and e.isValid:
                return e
    except Exception as ex:
        futil.debug("findEntityByToken failed: %s" % ex)
    return None


def attr(obj, name):
    try:
        a = obj.attributes.itemByName(ATTR_GROUP, name)
        return a.value if a else None
    except Exception:
        return None


def dropdown(inputs, id_, name, table, value):
    dd = inputs.addDropDownCommandInput(id_, name, adsk.core.DropDownStyles.TextListDropDownStyle)
    for label, v in table.items():
        dd.listItems.add(label, v == value, "")
    return dd


def select(dd, table, value):
    for k in range(dd.listItems.count):
        it = dd.listItems.item(k)
        if table.get(it.name) == value:
            it.isSelected = True
            return


def set_expr(inp, expr):
    try:
        inp.expression = expr
    except Exception as e:
        futil.log("couldn't restore '%s' = %s (%s)" % (inp.id, expr, e))


def polygrid_of(ent):
    """Which polygrid an entity belongs to: ("sketch", sketch) for a Sketch Polygrid, ("solid",
    extrude) for a Polygrid (its extrude, sketch, timeline group, or anything in its sketch), or
    (None, None)."""
    if ent is None:
        return None, None
    ext = adsk.fusion.ExtrudeFeature.cast(ent)
    if ext and attr(ext, SOLID_ATTR):
        return "solid", ext
    grp = adsk.fusion.TimelineGroup.cast(ent)
    if grp:
        for k in range(grp.count):
            kind, obj = polygrid_of(grp.item(k).entity)
            if kind:
                return kind, obj
        return None, None
    sk = adsk.fusion.Sketch.cast(ent)
    if not sk:
        for cls in (adsk.fusion.SketchCurve, adsk.fusion.Profile, adsk.fusion.SketchPoint):
            e = cls.cast(ent)
            if e:
                sk = e.parentSketch
                break
    if not sk:
        return None, None
    if attr(sk, SKETCH_ATTR):
        return "sketch", sk
    ext = adsk.fusion.ExtrudeFeature.cast(entity(attr(sk, SOLID_SKETCH_ATTR)))
    if ext:
        return "solid", ext
    return None, None


# --------------------------------------------------------------------------- sketches and the timeline
def restore_marker(tl, marker, at_end, added=0):
    if at_end:
        tl.moveToEnd()
    else:
        tl.markerPosition = marker + added  # items added before the marker push it along


def add_sketch(comp, plane, name):
    try:
        sk = comp.sketches.addWithoutEdges(plane)
    except Exception as e:
        futil.debug("addWithoutEdges failed (%s); trying sketches.add" % e)
        sk = comp.sketches.add(plane)
    sk.name = name
    return sk


def new_sketch(src, name):
    """A new sketch on src's plane, at the timeline marker.

    When src is on a body face, Fusion only hands the face over with the marker just before src.
    So read the face there, then find the same face (by its token) at the marker and put the new
    sketch there. If the face isn't there any more, the sketch goes right after src (or, failing
    that, right before it). The marker always ends up where it was."""
    comp = src.parentComponent
    try:
        return add_sketch(comp, src.referencePlane, name)
    except RuntimeError as e:
        futil.debug("the area's sketch is on a face; rolling the timeline to it (%s)" % e)
    tl = design().timeline
    marker, at_end, count = tl.markerPosition, tl.markerPosition == tl.count, tl.count
    try:
        src.timelineObject.rollTo(True)
        tok = token(src.referencePlane)
    finally:
        restore_marker(tl, marker, at_end)
    face = entity(tok)
    if face is not None:
        try:
            return add_sketch(comp, face, name)
        except Exception as e:
            futil.debug("  no sketch on the face at the marker: %s" % e)
    futil.log("the area sketch's face isn't there at the timeline marker; putting the new sketch next to the area's sketch")
    try:
        src.timelineObject.rollTo(False)
        face = entity(tok)
        if face is None:
            src.timelineObject.rollTo(True)
            face = src.referencePlane
        return add_sketch(comp, face, name)
    finally:
        restore_marker(tl, marker, at_end, tl.count - count)


# --------------------------------------------------------------------------- the dialog
class Dialog:
    """State and event handlers for one run of a polygrid command (see subclasses)."""
    name = "Polygrid"
    dropdowns = GRID_DROPDOWNS
    values = GRID_VALUES
    defaults = GRID_DEFAULTS
    can_be_zero = ("border", "corner")
    can_be_negative = ()
    last_key = "polygridLast"  # design setting: the last settings used

    def __init__(self):
        self.inp = {}
        self.loading = False
        self.key = None
        self.res = (None, "Select the area to fill.")
        self.border_follows = True
        self.size_by = None
        self.last_err = None
        self.done = False

    # ------------------------------------------------------------- building
    def keep(self, inp):
        self.inp[inp.id] = inp
        return inp

    def dd(self, id_, table):
        return table[self.inp[id_].selectedItem.name]

    def add_area(self, inputs, prompt):
        a = self.keep(inputs.addSelectionInput("area", "Area", prompt))
        a.addSelectionFilter("Profiles")
        a.addSelectionFilter("SketchCurves")
        a.setSelectionLimits(0, 0)
        a.tooltip = "Sketch regions to fill, or the closed outline curves around the area"
        self.keep(inputs.addTextBoxCommandInput("editing", "", "", 2, True)).isVisible = False

    def add_grid(self, inputs, units):
        d = self.defaults
        vin = adsk.core.ValueInput.createByString
        self.keep(dropdown(inputs, "shape", "Cell shape", SHAPES, d["shape"]))
        self.keep(inputs.addBoolValueInput("rot90", "Rotate 90°", True, "", d["rot90"])).tooltip = (
            "Turn the triangles or hexagons a quarter turn (hexagons: flat top or pointed top)")
        self.keep(dropdown(inputs, "sizeBy", "Size by", SIZE_BY, d["sizeBy"]))
        self.keep(inputs.addValueInput("side", "Side length", units, vin(d["side"]))).tooltip = "Side length of a pocket"
        self.keep(inputs.addValueInput("inscribed", "Inscribed Ø", units, vin(d["inscribed"]))).tooltip = (
            "Diameter of the largest circle that fits in a pocket")
        self.keep(inputs.addValueInput("length", "Cell length", units, vin(d["length"]))).tooltip = (
            "Pocket length (along the grid)")
        self.keep(inputs.addValueInput("width", "Cell width", units, vin(d["width"]))).tooltip = (
            "Pocket width (across the grid)")
        self.keep(inputs.addValueInput("rib", "Rib thickness", units, vin(d["rib"]))).tooltip = (
            "Width of the ribs between pockets")
        self.keep(inputs.addValueInput("border", "Border", units, vin(d["border"]))).tooltip = (
            "Width of the rib along the area's edge and around its holes (0 for none). Follows the rib "
            "thickness until you change it.")
        self.keep(inputs.addValueInput("corner", "Corner radius", units, vin(d["corner"]))).tooltip = (
            "Radius of the pockets' corners (0 for sharp). Pockets cut by the edge are rounded where they meet it too.")
        self.keep(dropdown(inputs, "cornerStyle", "Corner style", STYLES, d["cornerStyle"])).tooltip = (
            "Arc: a plain radius. G3 / G4: a curvature-continuous curve standing in for that radius, as GX G3 "
            "Fillet makes (smoothest shape, best fit to the arc).")
        mp = self.keep(inputs.addIntegerSliderCommandInput("minPartial", "Smallest cut pocket %", 0, 90, False))
        mp.valueOne = d["minPartial"]
        mp.tooltip = "Pockets cut by the edge that are smaller than this share of a whole pocket are left out"

        grp = self.keep(inputs.addGroupCommandInput("symmetry", "Symmetry"))
        gi = grp.children
        for id_, name, axis in (("vline", "Vertical line", "vertical"), ("hline", "Horizontal line", "horizontal")):
            s = self.keep(gi.addSelectionInput(id_, name, "Optional: the pattern is mirrored about this %s line" % axis))
            s.addSelectionFilter("SketchLines")
            s.addSelectionFilter("ConstructionLines")
            s.setSelectionLimits(0, 1)
            self.keep(dropdown(gi, id_[0] + "phase", "On the %s axis" % axis, PHASES, d[id_[0] + "phase"])).tooltip = (
                "Whether the %s mirror axis runs through the middle of a row of pockets or along a rib" % axis)
        self.keep(gi.addTextBoxCommandInput("symInfo", "", "", 2, True))

    def add_info(self, inputs):
        self.keep(inputs.addTextBoxCommandInput("info", "", "Select the area to fill.", 5, True))

    def restore_last(self):
        last = gx_comb.get_setting(self.last_key, "")
        if last:
            try:
                self.apply(json.loads(last))
            except Exception as e:
                futil.log("couldn't restore the last %s settings: %s" % (self.name, e))
        self.border_follows = self.inp["border"].expression == self.inp["rib"].expression
        self.size_by = self.dd("sizeBy", SIZE_BY)

    def connect(self, cmd):
        for event, fn, cls in ((cmd.inputChanged, self.on_input_changed, adsk.core.InputChangedEventHandler),
                               (cmd.validateInputs, self.on_validate, adsk.core.ValidateInputsEventHandler),
                               (cmd.preSelect, self.on_pre_select, adsk.core.SelectionEventHandler),
                               (cmd.activate, self.on_activate, adsk.core.CommandEventHandler),
                               (cmd.executePreview, self.on_preview, adsk.core.CommandEventHandler),
                               (cmd.execute, self.on_execute, adsk.core.CommandEventHandler),
                               (cmd.destroy, self.on_destroy, adsk.core.CommandEventHandler)):
            futil.add_handler(event, fn, cls)

    # ------------------------------------------------------------- settings
    def apply(self, data):
        """Set the dialog from stored settings."""
        self.loading = True
        try:
            for id_, table in self.dropdowns:
                if id_ in data:
                    select(self.inp[id_], table, data[id_])
            for id_ in self.values:
                if data.get(id_):
                    set_expr(self.inp[id_], data[id_])
            for id_ in ("rot90",) + self.bools():
                if id_ in data:
                    self.inp[id_].value = bool(data[id_])
            self.inp["minPartial"].valueOne = int(data.get("minPartial", self.defaults["minPartial"]))
        finally:
            self.loading = False
        self.size_by = self.dd("sizeBy", SIZE_BY)
        self.border_follows = data.get("border") == data.get("rib") if "border" in data else self.border_follows

    def bools(self):
        return ()

    def settings(self):
        """The dialog's settings, as stored (expressions, so user parameters keep working)."""
        data = {id_: self.dd(id_, t) for id_, t in self.dropdowns}
        data.update({id_: self.inp[id_].expression for id_ in self.values})
        for id_ in ("rot90",) + self.bools():
            data[id_] = self.inp[id_].value
        data["minPartial"] = self.inp["minPartial"].valueOne
        return data

    def save_last(self, data):
        keep = [k for k, _ in self.dropdowns] + list(self.values) + ["rot90", "minPartial"] + list(self.bools())
        gx_comb.set_setting(self.last_key, json.dumps({k: data[k] for k in keep if k in data}))

    def area_record(self, res):
        """How to find the area again: the sketch and a point inside each region, or the outline curves."""
        rec = dict(src=token(res["src"].nativeObject or res["src"]), seeds=[list(s) for s in res["seeds"]],
                   area=res["source"])
        for id_ in ("vline", "hline"):
            sel = self.inp[id_]
            rec[id_] = token(sel.selection(0).entity) if sel.selectionCount else None
        return rec

    def restore_area(self, data):
        """Select the stored area and symmetry lines again. Returns a problem to show, or ''."""
        self.loading = True
        problem = ""
        try:
            area = self.inp["area"]
            area.clearSelection()
            src_info = data.get("area") or {}
            if src_info.get("type") == "curves":
                toks = src_info.get("curves") or []
                found = [e for e in (entity(t) for t in toks) if e]
                for c in found:
                    area.addSelection(c)
                if len(found) < len(toks):
                    problem = "Some of the outline's curves are gone. Select the outline again."
            else:
                src = adsk.fusion.Sketch.cast(entity(data.get("src")))
                seeds = data.get("seeds") or []
                found = self.find_profiles(src, seeds) if src else []
                for p in found:
                    area.addSelection(p)
                if len(found) < len(seeds):
                    problem = ("Couldn't find the area in sketch '%s' (has it changed?). Select the area again."
                               % (src.name if src else "?"))
            for id_ in ("vline", "hline"):
                sel = self.inp[id_]
                sel.clearSelection()
                e = entity(data.get(id_))
                if e:
                    sel.addSelection(e)
                elif data.get(id_):
                    futil.log("the %s symmetry line is gone" % ("vertical" if id_ == "vline" else "horizontal"))
        finally:
            self.loading = False
        return problem

    @staticmethod
    def find_profiles(sketch, seeds):
        """The profiles of a sketch that contain the stored seed points."""
        profs = [sketch.profiles.item(k) for k in range(sketch.profiles.count)]
        loops, out = {}, []
        for seed in seeds:
            for k, p in enumerate(profs):
                if k not in loops:
                    try:
                        loops[k] = W.prepare_area(fusion_geom.profile_loops(p))
                    except Exception as e:
                        futil.debug("  profile %d unreadable: %s" % (k, e))
                        loops[k] = None
                if loops[k] and W.area_contains(loops[k], tuple(seed)):
                    if not any(p == q for q in out):
                        out.append(p)
                    break
        return out

    # ------------------------------------------------------------- visibility
    def refresh(self):
        shape = self.dd("shape", SHAPES)
        rect = shape == W.RECTANGLE
        by = self.dd("sizeBy", SIZE_BY)
        self.inp["sizeBy"].isVisible = not rect
        self.inp["side"].isVisible = not rect and by == W.SIDE
        self.inp["inscribed"].isVisible = not rect and by == W.INSCRIBED
        self.inp["length"].isVisible = self.inp["width"].isVisible = rect
        self.inp["rot90"].isVisible = shape in (W.TRIANGLE, W.HEXAGON)
        corner = self.inp["corner"]
        self.inp["cornerStyle"].isVisible = corner.isValidExpression and corner.value > 0
        vs, hs = W.allowed_phases(shape, self.rot90(shape))
        self.inp["vphase"].isVisible = len(vs) > 1
        self.inp["hphase"].isVisible = len(hs) > 1
        self.inp["editing"].isVisible = self.editing()
        say = {W.CELL: "through pocket centres", W.RIB: "along a rib"}
        v, h = self.phases(shape)
        self.inp["symInfo"].formattedText = (
            "Mirror axes: vertical %s, horizontal %s. Without lines they cross at the centre of the area." % (say[v], say[h]))
        self.refresh_more()

    def refresh_more(self):
        pass

    def editing(self):
        return False

    def rot90(self, shape):
        return self.inp["rot90"].value and shape in (W.TRIANGLE, W.HEXAGON)

    def phases(self, shape):
        vs, hs = W.allowed_phases(shape, self.rot90(shape))
        v, h = self.dd("vphase", PHASES), self.dd("hphase", PHASES)
        return (v if v in vs else vs[0]), (h if h in hs else hs[0])

    # ------------------------------------------------------------- events
    def on_activate(self, args):
        pass

    def on_pre_select(self, args):
        inp = getattr(args, "activeInput", None)
        if inp is not None and inp.id == "area" and not self.accept_area(args.selection.entity):
            args.isSelectable = False

    def accept_area(self, ent):
        """May this entity go in the Area box? (Regions and curves, but not a polygrid's own sketch.)"""
        if polygrid_of(ent)[0]:
            return False
        return bool(adsk.fusion.Profile.cast(ent) or adsk.fusion.SketchCurve.cast(ent))

    def on_input_changed(self, args):
        inp = args.input
        id_ = inp.id
        futil.debug("%s input '%s' = %s" % (self.name, id_, futil.describe_input(inp)))
        if self.loading:
            return
        if id_ == "area":
            self.area_changed(inp)
        elif id_ == "sizeBy" and self.dd("sizeBy", SIZE_BY) != self.size_by:
            # keep the pocket the same size, shown the other way
            self.size_by = self.dd("sizeBy", SIZE_BY)
            shape = self.dd("shape", SHAPES)
            if shape != W.RECTANGLE:
                if self.size_by == W.INSCRIBED:
                    self.inp["inscribed"].value = 2 * W.pocket_inradius(shape, W.SIDE, self.inp["side"].value)
                else:
                    r = W.pocket_inradius(shape, W.INSCRIBED, self.inp["inscribed"].value)
                    self.inp["side"].value = W.pocket_side(shape, r)
        elif id_ == "rib":
            if self.border_follows:
                self.loading = True
                try:
                    self.inp["border"].expression = inp.expression
                finally:
                    self.loading = False
        elif id_ == "border":
            self.border_follows = inp.expression == self.inp["rib"].expression
        self.input_changed(id_, inp)
        self.refresh()

    def area_changed(self, inp):
        """Drop anything from the Area box that can't be an area."""
        ents = [inp.selection(k).entity for k in range(inp.selectionCount)]
        if any(not self.accept_area(e) for e in ents):
            self.loading = True
            try:
                inp.clearSelection()
                for e in ents:
                    if self.accept_area(e):
                        inp.addSelection(e)
            finally:
                self.loading = False

    def input_changed(self, id_, inp):
        pass

    # ------------------------------------------------------------- the grid
    def line2d(self, id_, sk):
        """A symmetry line as two 2D points in sk's sketch space, or None."""
        sel = self.inp[id_]
        if not sel.selectionCount:
            return None
        ent = sel.selection(0).entity
        sl = adsk.fusion.SketchLine.cast(ent)
        if sl:
            g3 = sl.worldGeometry
            a, b = g3.startPoint, g3.endPoint
        else:
            ax = adsk.fusion.ConstructionAxis.cast(ent)
            if not ax:
                return None
            il = ax.geometry
            a = il.origin
            b = a.copy()
            b.translateBy(il.direction)
        a, b = sk.modelToSketchSpace(a), sk.modelToSketchSpace(b)
        if math.hypot(b.x - a.x, b.y - a.y) < 1e-9:
            raise ValueError("The %s symmetry line is square to the sketch plane."
                             % ("vertical" if id_ == "vline" else "horizontal"))
        return ((a.x, a.y), (b.x, b.y))

    def area_items(self):
        """(profiles, curves, sketch) picked in the Area box; raises ValueError."""
        sel = self.inp["area"]
        ents = [sel.selection(k).entity for k in range(sel.selectionCount)]
        profs = [p for p in (adsk.fusion.Profile.cast(e) for e in ents) if p]
        curves = [c for c in (adsk.fusion.SketchCurve.cast(e) for e in ents) if c]
        if not (profs or curves):
            raise ValueError("Select the area to fill: sketch regions, or the closed outline curves around it.")
        if profs and curves:
            raise ValueError("Pick either sketch regions or outline curves for the area, not both.")
        items = profs or curves
        src = items[0].parentSketch
        if any(x.parentSketch != src for x in items[1:]):
            raise ValueError("The area must all be in one sketch.")
        return profs, curves, src

    def value_errors(self):
        errs = []
        for id_ in self.values:
            inp = self.inp[id_]
            if not inp.isVisible:
                continue
            if not inp.isValidExpression:
                errs.append("%s: that isn't a valid value." % inp.name)
            elif id_ in self.can_be_negative:
                continue
            elif id_ in self.can_be_zero:
                if inp.value < 0:
                    errs.append("%s can't be negative." % inp.name)
            elif inp.value <= 0:
                errs.append("%s must be above zero." % inp.name)
        return errs

    def compute(self):
        """(result, error) for the dialog as it stands. Cached: validate and preview both ask."""
        try:
            profs, curves, src = self.area_items()
        except ValueError as e:
            return None, str(e)
        errs = self.value_errors()
        if errs:
            return None, "<br>".join(errs)
        shape = self.dd("shape", SHAPES)
        if shape == W.RECTANGLE:
            dims = (self.inp["length"].value, self.inp["width"].value)
        else:
            by = self.dd("sizeBy", SIZE_BY)
            dims = W.pocket_inradius(shape, by, self.inp["side" if by == W.SIDE else "inscribed"].value)
        vphase, hphase = self.phases(shape)
        try:
            vline, hline = self.line2d("vline", src), self.line2d("hline", src)
        except ValueError as e:
            return None, str(e)
        spec = dict(shape=shape, dims=dims, rib=self.inp["rib"].value, border=self.inp["border"].value,
                    vline=vline, hline=hline, rot90=self.rot90(shape), vphase=vphase, hphase=hphase,
                    min_partial=self.inp["minPartial"].valueOne / 100.0, corner=self.inp["corner"].value,
                    corner_style=self.dd("cornerStyle", STYLES))
        boxes = []
        for x in profs or curves:
            bb = x.boundingBox
            boxes.append([round(v, 9) for v in (bb.minPoint.x, bb.minPoint.y, bb.maxPoint.x, bb.maxPoint.y)])
        key = json.dumps([token(src), [token(c) for c in curves], boxes, sorted(spec.items())], default=str)
        if self.key == key:
            return self.res
        t0 = time.time()
        try:
            if profs:
                regions, source = [fusion_geom.profile_loops(p) for p in profs], dict(type="profiles")
            else:
                regions = W.areas_from_loops(W.chain_loops(fusion_geom.curve_prims(curves)))
                source = dict(type="curves", curves=[token(c) for c in curves])
            res = W.build(regions, **spec)
            res["src"], res["source"] = src, source
            out = (res, None)
            futil.debug("%s %s %s rib %.4g border %.4g: %d whole, %d cut, %d dropped (%.2f s)" % (
                self.name, shape, dims, spec["rib"], spec["border"], res["full"], res["partial"], res["dropped"],
                time.time() - t0))
        except ValueError as e:
            out = (None, str(e))
            futil.debug("%s rejected: %s" % (self.name, e))
        self.key, self.res = key, out
        return out

    def extra_errors(self):
        return []

    def on_validate(self, args):
        res, err = self.compute()
        if not err and res and not res["pockets"]:
            err = "No pockets fit: the cells are too big for the area, or the border fills it."
        if not err:
            err = "<br>".join(self.extra_errors())
        args.areInputsValid = not err
        if err != self.last_err:
            self.last_err = err
            futil.debug("%s validate: %s" % (self.name, err or "ok"))
        if err:
            self.inp["info"].formattedText = "<span style='color:#c33'>%s</span>" % err

    # ------------------------------------------------------------- output
    @staticmethod
    def fmt(v):
        d = design()
        try:
            return d.unitsManager.formatInternalValue(v, d.unitsManager.defaultLengthUnits, True)
        except Exception:
            return "%.4g cm" % v

    def info(self, res):
        shape = self.dd("shape", SHAPES)
        rib = self.inp["rib"].value
        n = res["full"] + res["partial"]
        lines = ["<b>%d pockets</b>: %d whole, %d cut by the edge%s" % (
            n, res["full"], res["partial"], (", %d slivers left out" % res["dropped"]) if res["dropped"] else "")]
        if shape == W.RECTANGLE:
            L, Wd = self.inp["length"].value, self.inp["width"].value
            lines.append("Pocket %s × %s; pitch %s × %s" % (self.fmt(L), self.fmt(Wd), self.fmt(L + rib),
                                                            self.fmt(Wd + rib)))
        else:
            by = self.dd("sizeBy", SIZE_BY)
            r = W.pocket_inradius(shape, by, self.inp["side" if by == W.SIDE else "inscribed"].value)
            lines.append("Pocket side %s, inscribed Ø %s; pitch %s" % (
                self.fmt(W.pocket_side(shape, r)), self.fmt(2 * r), self.fmt(2 * r + rib)))
        if res["region_area"] > 0:
            lines.append("Pockets take %.0f%% of the area" % (100.0 * res["pocket_area"] / res["region_area"]))
        notes = []
        if res.get("smaller_corners"):
            notes.append("%d made smaller to fit" % res["smaller_corners"])
        if res.get("arc_corners"):
            notes.append("%d left as arcs (no room, or next to a spline edge)" % res["arc_corners"])
        if notes:
            lines.append("Corners at the edge: " + "; ".join(notes))
        return "<br>".join(lines)

    def preview_outlines(self, res, outline):
        f = fusion_geom.model_map(res["src"])

        def segs(loops, steps):
            out = []
            for lp in loops:
                for p in lp:
                    pts = [f(q) for q in W.prim_polyline(p, steps)]
                    for a, b in zip(pts, pts[1:]):
                        out.extend(a + b)
            return out
        comb_graphics.draw_segments(PREVIEW_KEY, segs([lp for pk in res["pockets"] for lp in pk["loops"]], 12),
                                    POCKET_RGB, 2)
        if outline:
            comb_graphics.draw_segments(OUTLINE_KEY, segs(res["outlines"], 24), OUTLINE_RGB, 2)
        else:
            comb_graphics.clear(OUTLINE_KEY)

    @staticmethod
    def clear_preview():
        comb_graphics.clear(PREVIEW_KEY)
        comb_graphics.clear(OUTLINE_KEY)

    def draw(self, sk, res, outline, border, f):
        """Draw the pockets (and with outline, the area's outline) into sketch sk; f maps the
        area sketch's 2D space into sk's. Returns the curves drawn for each loop: the pockets'
        loops in order, then the outline's."""
        edges_only = outline and border <= 0  # the outline is drawn anyway; draw only the pockets' own edges
        jobs = [lp for pk in res["pockets"] for lp in (pk["edges"] if edges_only else pk["loops"])]
        if outline:
            jobs += res["outlines"]
        pd = None
        if len(jobs) > PROGRESS_FROM:
            try:
                pd = ui.createProgressDialog()
                pd.isCancelButtonShown = False
                pd.show(self.name, "Drawing pocket %v of %m", 0, len(jobs), 0)
            except Exception:
                pd = None
        made = []
        t0 = time.time()
        sk.isComputeDeferred = True
        try:
            for k, prims in enumerate(jobs):
                made.append(fusion_geom.draw_loop(sk, prims, f))
                if pd and k % 25 == 0:
                    pd.progressValue = k
                    adsk.doEvents()
        finally:
            sk.isComputeDeferred = False
            if pd:
                pd.hide()
        futil.debug("%s: drew %d curves in %d loops (%.1f s)" % (self.name, sum(len(m) for m in made), len(jobs),
                                                                time.time() - t0))
        return made

    def on_preview(self, args):
        res, err = self.compute()
        if err or not res:
            self.clear_preview()
            return
        self.preview(args, res)
        self.inp["info"].formattedText = self.info(res)
        args.isValidResult = False  # execute builds the real result

    def preview(self, args, res):
        raise NotImplementedError

    def on_execute(self, args):
        raise NotImplementedError

    def on_destroy(self, args):
        futil.debug("=== %s closed (reason: %s)" % (self.name, args.terminationReason))
        self.clear_preview()
        self.closed()

    def closed(self):
        pass


def marking_menu(edit_id, kind):
    """A markingMenuDisplaying handler showing command edit_id when a polygrid of this kind is selected."""
    def handler(args):
        try:
            ents = list(args.selectedEntities or [])
            show = any(polygrid_of(e)[0] == kind for e in ents)
            ctrls = args.linearMarkingMenu.controls
            c = ctrls.itemById(edit_id)
            if c:
                c.isVisible = show
            elif show:
                ctrls.addCommand(ui.commandDefinitions.itemById(edit_id))
        except Exception:
            futil.handle_error("polygrid marking menu", show=False)
    handler.__name__ = "marking_menu_" + kind
    return handler
