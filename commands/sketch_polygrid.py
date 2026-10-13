"""'Sketch Polygrid' command (Sketch > Create): draws a regular grid of polygonal pockets
(triangles, squares, hexagons or rectangles), separated by ribs, filling a sketch area.

The grid is drawn in a sketch of its own, "Sketch Polygrid", on the area's plane, and its
settings are stored on that sketch (attribute GX_tool/sketchPolygrid). To change them, run the
command with the grid sketch (or any of its curves) selected, pick the grid in the Area box, or
right-click it > Edit Sketch Polygrid: the dialog opens with the stored settings, and OK redraws
the sketch. The area is found again from a point stored inside each region (or from the outline
curves), so the grid follows edits to the area's sketch.

"Cutouts" draws only the pocket outlines (select them all and cut). "Ribs" also draws the area's
outline, so the web of ribs between the pockets is a single profile to extrude.
For a solid result in the timeline, see the Polygrid command (polygrid.py).
"""

import json
import os

import adsk.core
import adsk.fusion

from ..lib import fusion_geom, futil
from ..lib.futil import ATTR_GROUP, ui
from . import polygrid_dialog as D

CMD_ID = "GXTool_SketchPolygrid"
EDIT_ID = "GXTool_SketchPolygridEdit"
NAME = "Sketch Polygrid"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "sketch_polygrid")
MODES = {"Cutouts (pocket outlines)": "cutouts", "Ribs (pockets + area outline)": "ribs"}

_dlg = None  # the open dialog


class SketchDialog(D.Dialog):
    name = NAME
    dropdowns = D.GRID_DROPDOWNS + (("mode", MODES),)
    defaults = dict(D.GRID_DEFAULTS, mode="cutouts")
    last_key = "sketchPolygridLast"

    def __init__(self):
        super().__init__()
        self.edit = None          # the grid sketch being edited
        self.edit_visible = True  # ...and whether it was visible

    def created(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        d = D.design()
        units = d.unitsManager.defaultLengthUnits if d else "mm"
        futil.debug("=== %s opened" % NAME)
        pre = [ui.activeSelections.item(k).entity for k in range(ui.activeSelections.count)]
        edit = next((obj for kind, obj in (D.polygrid_of(e) for e in pre) if kind == "sketch"), None)

        self.add_area(inputs, "Pick the sketch regions or outline to fill, or a sketch polygrid to edit")
        self.inp["area"].addSelectionFilter("Sketches")
        self.keep(D.dropdown(inputs, "mode", "Make", MODES, self.defaults["mode"])).tooltip = (
            "Cutouts: just the pocket outlines, to cut. Ribs: the area's outline as well, so the ribs between "
            "the pockets form one profile to extrude.")
        self.add_grid(inputs, units)
        self.add_info(inputs)
        self.restore_last()
        if edit:
            self.load(edit)
        self.refresh()
        self.connect(cmd)

    def editing(self):
        return self.edit is not None

    def accept_area(self, ent):
        kind, obj = D.polygrid_of(ent)
        if kind == "sketch":
            return obj != self.edit  # another sketch polygrid: switch to editing it
        return super().accept_area(ent)

    def area_changed(self, inp):
        ents = [inp.selection(k).entity for k in range(inp.selectionCount)]
        grids = [obj for kind, obj in (D.polygrid_of(e) for e in ents) if kind == "sketch" and obj != self.edit]
        if grids:
            self.load(grids[0])
        else:
            super().area_changed(inp)

    def load(self, grid):
        """Switch the dialog to editing an existing sketch polygrid."""
        data = json.loads(D.attr(grid, D.SKETCH_ATTR))
        futil.debug("editing sketch polygrid '%s': %s" % (grid.name, data))
        if self.edit is not None and self.edit.isValid and self.edit != grid:
            self.edit.isVisible = self.edit_visible  # switching to another grid: put this one back
        self.apply(data)
        self.edit, self.edit_visible = grid, grid.isVisible
        problem = self.restore_area(data)
        self.inp["editing"].formattedText = problem or (
            "Editing <b>%s</b>. OK redraws it; Cancel leaves it as it was." % grid.name)

    def preview(self, args, res):
        if self.edit is not None and self.edit.isValid:
            self.edit.isVisible = False  # show the new grid, not the old one
        self.preview_outlines(res, self.dd("mode", MODES) == "ribs")

    def on_execute(self, args):
        self.clear_preview()
        self.key = None  # recompute: the preview may have changed what's visible
        res, err = self.compute()
        if err or not res:
            ui.messageBox(err or "Nothing to draw.")
            return
        src = res["src"]
        native = src.nativeObject or src
        if self.edit is not None and self.edit.isValid:
            sk = self.edit
            f, coplanar = fusion_geom.sketch_map(native, sk)
            if not coplanar:
                ui.messageBox("The area isn't in the plane of the grid's sketch, so the grid can't move there. "
                              "Make a new sketch polygrid for it instead.")
                return
            fusion_geom.clear_sketch(sk)
            sk.isVisible = True
        else:
            sk = D.new_sketch(native, NAME)
            f, _ = fusion_geom.sketch_map(native, sk)
        data = self.settings()
        self.draw(sk, res, data["mode"] == "ribs", self.inp["border"].value, f)
        data.update(v=1, **self.area_record(res))
        sk.attributes.add(ATTR_GROUP, D.SKETCH_ATTR, json.dumps(data))
        self.done = True
        self.save_last(data)
        futil.debug("sketch polygrid %s in sketch '%s'" % ("redrawn" if self.edit else "made", sk.name))

    def closed(self):
        if self.edit is not None and not self.done:
            try:
                if self.edit.isValid and self.edit.isVisible != self.edit_visible:
                    self.edit.isVisible = self.edit_visible
            except Exception:
                pass


def _on_created(args):
    global _dlg
    _dlg = SketchDialog()
    _dlg.created(args)


def start():
    futil.create_button(CMD_ID, NAME,
                        "Fill a sketch area with a grid of triangle, square, hexagon or rectangle pockets separated "
                        "by ribs, drawn in a sketch. Select one and run it again to edit it.",
                        RESOURCES, _on_created)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
    d = ui.commandDefinitions.addButtonDefinition(EDIT_ID, "Edit " + NAME, "Change this sketch polygrid's settings",
                                                  RESOURCES)
    futil.add_handler(d.commandCreated, _on_created, adsk.core.CommandCreatedEventHandler)
    futil.add_handler(ui.markingMenuDisplaying, D.marking_menu(EDIT_ID, "sketch"), adsk.core.MarkingMenuEventHandler)


def stop():
    D.Dialog.clear_preview()
    futil.delete_button(CMD_ID)
    d = ui.commandDefinitions.itemById(EDIT_ID)
    if d:
        d.deleteMe()
