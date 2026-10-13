"""'GX Smooth' command: fair a control-point spline (bending energy, optional tension, and an
optional exact curvature-flow polish). Works on GX curves and any other control-point spline.

The new shape is applied through the sketch: dimensioned points get new dimension values,
others are dragged, so existing constraints (points on lines, symmetry) are respected.
"""

import os

import adsk.core
import adsk.fusion

from ..lib import comb_graphics, fusion_geom, futil
from ..lib import gx_math as g
from . import gx_comb

CMD_ID = "GXTool_Smooth"
PREVIEW_KEY = "gx_smooth_preview"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gx_smooth")

M_BEND, M_CURV = "Bending energy (fast)", "Curvature flow (exact, slower)"

S = {}


def _i(inputs, id_):
    return inputs.itemById(id_)


def _on_created(args):
    S.clear()
    cmd = args.command
    inputs = cmd.commandInputs
    futil.debug("=== GX Smooth opened")

    sel = inputs.addSelectionInput("curve", "Spline", "Pick a control-point spline")
    sel.addSelectionFilter("SketchCurves")
    sel.setSelectionLimits(0, 1)
    if futil.ui.activeSelections.count == 1:
        pre = adsk.fusion.SketchControlPointSpline.cast(futil.ui.activeSelections.item(0).entity)
        if pre:
            sel.addSelection(pre)

    dd = inputs.addDropDownCommandInput("method", "Method", adsk.core.DropDownStyles.TextListDropDownStyle)
    dd.listItems.add(M_BEND, True, "")
    dd.listItems.add(M_CURV, False, "")
    a = inputs.addFloatSliderCommandInput("amount", "Amount", "", 0.0, 1.0, False)
    a.valueOne = 0.8
    t = inputs.addFloatSliderCommandInput("tension", "Tension", "", 0.0, 0.5, False)
    t.valueOne = 0.0
    inputs.addBoolValueInput("keepEnds", "Keep end continuity", True, "", True)
    inputs.addBoolValueInput("comb", "Show curvature comb", True, "", True)
    cs = inputs.addFloatSliderCommandInput("combScale", "Comb scale", "", 0.1, 5.0, False)
    cs.valueOne = gx_comb.get_setting("combScale", 1.0)
    inputs.addTextBoxCommandInput("info", "", "Select a control-point spline.", 4, True)

    futil.add_handler(cmd.inputChanged, _on_input_changed, adsk.core.InputChangedEventHandler)
    futil.add_handler(cmd.validateInputs, _on_validate, adsk.core.ValidateInputsEventHandler)
    futil.add_handler(cmd.executePreview, _on_preview, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.execute, _on_execute, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.destroy, _on_destroy, adsk.core.CommandEventHandler)


def _spline(inputs):
    sel = _i(inputs, "curve")
    if not sel.selectionCount:
        return None
    return adsk.fusion.SketchControlPointSpline.cast(sel.selection(0).entity)


def _error(inputs):
    sel = _i(inputs, "curve")
    if not sel.selectionCount:
        return "Select a control-point spline."
    if not _spline(inputs):
        return "That curve isn't a control-point spline. (Fit-point splines, lines and arcs can't be smoothed.)"
    return ""


def _on_input_changed(args):
    futil.debug("smooth input '%s' = %s" % (args.input.id, futil.describe_input(args.input)))


def _on_validate(args):
    inputs = args.firingEvent.sender.commandInputs
    err = _error(inputs)
    args.areInputsValid = not err
    if err:
        _i(inputs, "info").formattedText = "<span style='color:#c33'>%s</span>" % err


def _run(inputs):
    spline = _spline(inputs)
    method = g.METHOD_CURVATURE if _i(inputs, "method").selectedItem.name == M_CURV else g.METHOD_BENDING
    amount, tension = _i(inputs, "amount").valueOne, _i(inputs, "tension").valueOne
    keep = _i(inputs, "keepEnds").value
    futil.debug("smooth: method=%s amount=%.2f tension=%.2f keepEnds=%s" % (method, amount, tension, keep))
    cps, ctrl, degree, knots = fusion_geom.read_spline(spline)
    if not cps:
        return spline, "Couldn't read this spline's control points."
    new, warning = g.fair_spline(ctrl, degree, knots, amount=amount, keep_ends=keep, tension=tension, method=method)
    if warning:
        return spline, "<span style='color:#b60'>%s</span>" % warning
    f0 = g.fairness(g.Spline(ctrl, degree, knots))
    f1 = g.fairness(g.Spline(new, degree, knots))
    moved = max(g.length(g.sub(a, b)) for a, b in zip(ctrl, new))
    resid = fusion_geom.apply_control_points(spline.parentSketch, cps, new)
    lines = ["<b>Comb roughness: %.3g → %.3g (%+.0f%%)</b>" % (f0, f1, (f1 / f0 - 1) * 100 if f0 > 0 else 0),
             "Largest control-point move: %.4g cm" % moved]
    if keep:
        cs, ce = g.end_runs(ctrl)
        lines.append("Ends kept: %d point(s) slide on the start line, %d on the end line" % (cs, ce))
    if resid > 1e-5:
        lines.append("<span style='color:#b60'>Sketch constraints held some points back (off by up to %.3g cm).</span>"
                     % resid)
    futil.debug("smooth result: roughness %.4g -> %.4g, moved %.4g, residual %.3g" % (f0, f1, moved, resid))
    return spline, "<br>".join(lines)


def _on_preview(args):
    inputs = args.command.commandInputs
    if _error(inputs):
        return
    spline, info = _run(inputs)
    _i(inputs, "info").formattedText = info
    if _i(inputs, "comb").value and spline:
        comb_graphics.draw(PREVIEW_KEY, [spline], _i(inputs, "combScale").valueOne, 120)
    else:
        comb_graphics.clear(PREVIEW_KEY)
    args.isValidResult = True  # the preview is the result


def _on_execute(args):
    # only reached if the preview wasn't kept
    inputs = args.command.commandInputs
    comb_graphics.clear(PREVIEW_KEY)
    if not _error(inputs):
        _run(inputs)


def _on_destroy(args):
    futil.debug("=== GX Smooth closed (reason: %s)" % args.terminationReason)
    comb_graphics.clear(PREVIEW_KEY)
    S.clear()
    gx_comb.refresh(force=True)


def start():
    futil.create_button(CMD_ID, "GX Smooth",
                        "Smooth a control-point spline by minimising its bending (and optionally its length), "
                        "keeping its ends and constraints.",
                        RESOURCES, _on_created)


def stop():
    comb_graphics.clear(PREVIEW_KEY)
    futil.delete_button(CMD_ID)
