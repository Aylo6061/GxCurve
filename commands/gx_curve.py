"""'GX Curve' command: G2/G3/G4 corner blends (symmetric / asymmetric) and freeform splines.

Corner mode: every leg control point has an on-canvas drag handle that slides it
along its leg (so continuity can't break). In symmetric mode leg B mirrors leg A.
On OK the curve is locked with sketch constraints (and optional dimensions) so it
stays G2/G3 and symmetric when edited later.

Freeform mode: builds a spline between two sketch points (continuing a line
through either point when there is one) and leaves it free to drag.
"""

import os

import adsk.core
import adsk.fusion

from ..lib import comb_graphics, fusion_geom, futil
from ..lib import gx_math as g
from ..lib.futil import ui
from . import gx_comb

CMD_ID = "GXTool_Curve"
PREVIEW_KEY = "gx_curve_preview"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gx_curve")
MAX_LEG = 4  # most points per leg (9 control points)

MODE_CORNER, MODE_FREE = "Corner", "Freeform"
FIVE_CUBIC, FIVE_QUINTIC = "Cubic (5 points)", "Quintic (6 points)"
APEX_LOCKED, APEX_FREE_LBL = "Locked to corner", "Free (drag the apex)"
SYM, ASYM = "Symmetric", "Asymmetric"

S = {}  # per-command state


# --------------------------------------------------------------------------- helpers
def _i(inputs, id_):
    return inputs.itemById(id_)


def _dd(inputs, id_):
    item = _i(inputs, id_).selectedItem
    return item.name if item else ""


def _dropdown(inputs, id_, name, items, selected):
    dd = inputs.addDropDownCommandInput(id_, name, adsk.core.DropDownStyles.TextListDropDownStyle)
    for it in items:
        dd.listItems.add(it, it == selected, "")
    return dd


def _style(inputs):
    count = int(_dd(inputs, "count"))
    five = "quintic" if _dd(inputs, "fiveType") == FIVE_QUINTIC else "cubic"
    apex = g.APEX_FREE if _dd(inputs, "apex") == APEX_FREE_LBL else g.APEX_CORNER
    return count, five, apex


def _dist(inputs, side, m):
    return [_i(inputs, "d%s%d" % (side, i)).value for i in range(m)]


def read_params(inputs):
    count, five, apex = _style(inputs)
    m, apex_mode, n, degree = g.corner_style(count, five, apex)
    sym = _dd(inputs, "symmetry") == SYM
    dA = _dist(inputs, "A", m)
    dB = dA if sym else _dist(inputs, "B", m)
    return dict(
        mode=_dd(inputs, "mode"), count=count, five_type=five, apex=apex, symmetric=sym,
        distA=dA, distB=dB, apex_dist=_i(inputs, "apexDist").value,
        tension=_i(inputs, "tension").valueOne, trim=_i(inputs, "trim").value,
        dims=_i(inputs, "dims").value, match=_i(inputs, "match").value,
        reach=_i(inputs, "reach").valueOne, comb=_i(inputs, "comb").value,
        comb_scale=_i(inputs, "combScale").valueOne,
    )


def _sel_entity(inputs, id_):
    s = _i(inputs, id_)
    return s.selection(0) if s.selectionCount else None


def update_geo(inputs):
    S["geo"], S["geo_err"] = None, ""
    a, b = _sel_entity(inputs, "lineA"), _sel_entity(inputs, "lineB")
    if not (a and b):
        return
    try:
        geo = fusion_geom.CornerGeo(a.entity, a.point, b.entity, b.point)
        S["geo"] = geo
        futil.debug("corner: at %s, uA=%s availA=%.4f, uB=%s availB=%.4f, bisector=%s, sketch='%s'"
                    % (_f(geo.corner), _f(geo.uA), geo.availA, _f(geo.uB), geo.availB, _f(geo.bis), geo.sketch.name))
    except ValueError as e:
        S["geo_err"] = str(e)
        futil.debug("corner: rejected: %s" % e)


def _f(p):
    return "(%.4f, %.4f)" % (p[0], p[1])


def set_layout(inputs, sides="AB", d0=None):
    """Place the leg points from the tension slider, keeping each leg's length."""
    count, five, apex = _style(inputs)
    m, apex_mode, _, _ = g.corner_style(count, five, apex)
    fr = g.tension_fractions(m, _i(inputs, "tension").valueOne)
    for side in sides:
        L = d0 if d0 is not None else _i(inputs, "d%s0" % side).value
        for i in range(MAX_LEG):
            _i(inputs, "d%s%d" % (side, i)).value = L * fr[i] if i < m else 0.0
        S["prev" + side] = L


def update_visibility(inputs):
    corner = _dd(inputs, "mode") == MODE_CORNER
    count, five, apex = _style(inputs)
    m, apex_mode, _, _ = g.corner_style(count, five, apex)
    sym = _dd(inputs, "symmetry") == SYM
    has_geo = S.get("geo") is not None
    for id_ in ("lineA", "lineB", "symmetry", "tension", "scaleInner", "trim", "dims"):
        _i(inputs, id_).isVisible = corner
    for id_ in ("startPt", "endPt", "match", "reach"):
        _i(inputs, id_).isVisible = not corner
    _i(inputs, "fiveType").isVisible = count == 5
    _i(inputs, "apex").isVisible = corner and count in (7, 9)
    _i(inputs, "grpA").isVisible = corner and has_geo
    _i(inputs, "grpB").isVisible = corner and has_geo and not sym
    for side in "AB":
        for i in range(MAX_LEG):
            _i(inputs, "d%s%d" % (side, i)).isVisible = i < m
    _i(inputs, "apexDist").isVisible = corner and has_geo and apex_mode == g.APEX_FREE


def update_manipulators(inputs):
    geo = S.get("geo")
    if not geo:
        return
    sk, c = geo.sketch, geo.corner
    origin = fusion_geom.model_point(sk, c)
    for side, u in (("A", geo.uA), ("B", geo.uB)):
        d = fusion_geom.model_dir(sk, c, u)
        for i in range(MAX_LEG):
            _i(inputs, "d%s%d" % (side, i)).setManipulator(origin, d)
    _i(inputs, "apexDist").setManipulator(origin, fusion_geom.model_dir(sk, c, geo.bis))


def errors(inputs):
    P = read_params(inputs)
    errs = []
    if P["mode"] == MODE_CORNER:
        if S.get("geo_err"):
            return [S["geo_err"]]
        geo = S.get("geo")
        if not geo:
            return ["Select two sketch lines (click each on the side you want to keep)."]
        for side, dist, avail in (("A", P["distA"], geo.availA), ("B", P["distB"], geo.availB)):
            if P["symmetric"] and side == "B":
                avail = min(geo.availA, geo.availB)
                dist = P["distA"]
            if not g.check_distances(dist):
                errs.append("Leg %s: points must be above zero and ordered outer to inner." % side)
            elif P["trim"] and dist[0] >= avail - 1e-7:
                errs.append("Leg %s is longer than its line. Shorten the leg or turn off Trim." % side)
    else:
        a, b = _sel_entity(inputs, "startPt"), _sel_entity(inputs, "endPt")
        if not (a and b):
            return ["Select a start point and an end point (for example, the ends of two lines)."]
        if a.entity.parentSketch != b.entity.parentSketch:
            errs.append("Both points must be in the same sketch.")
        elif g.length(g.sub(fusion_geom.p2(a.entity.geometry), fusion_geom.p2(b.entity.geometry))) < 1e-7:
            errs.append("The start and end points are at the same place.")
    return errs


def set_info(inputs, text):
    _i(inputs, "info").formattedText = text


def describe(P, extra=""):
    if P["mode"] == MODE_CORNER:
        m, apex_mode, n, deg = g.corner_style(P["count"], P["five_type"], P["apex"])
        G = g.expected_corner_continuity(P["count"], P["five_type"], P["apex"])
        s = "<b>%d control points, degree %d: G%d to both lines</b>" % (n, deg, G)
    else:
        s = "<b>" + extra + "</b>"
    return s


# --------------------------------------------------------------------------- events
def _on_created(args):
    S.clear()
    cmd = args.command
    edit = futil.app.activeEditObject
    futil.debug("=== GX Curve opened. Editing: %s '%s'; %d item(s) preselected: %s" % (
        edit.objectType if edit else None, getattr(edit, "name", ""), ui.activeSelections.count,
        " | ".join(futil.describe_entity(ui.activeSelections.item(i).entity) for i in range(ui.activeSelections.count))))
    cmd.setDialogInitialSize(320, 600)
    inputs = cmd.commandInputs

    _dropdown(inputs, "mode", "Mode", [MODE_CORNER, MODE_FREE], MODE_CORNER)

    # All selections are optional (min 0): Fusion blocks preview/OK on any unmet
    # selection minimum, hidden inputs included. errors() enforces what each mode needs.
    sa = inputs.addSelectionInput("lineA", "Line A", "Pick the first line, on the side to keep")
    sa.addSelectionFilter("SketchLines")
    sa.setSelectionLimits(0, 1)
    sb = inputs.addSelectionInput("lineB", "Line B", "Pick the second line, on the side to keep")
    sb.addSelectionFilter("SketchLines")
    sb.setSelectionLimits(0, 1)
    for id_, name in (("startPt", "Start point"), ("endPt", "End point")):
        sp = inputs.addSelectionInput(id_, name, "Pick a sketch point or a line end")
        sp.addSelectionFilter("SketchPoints")
        sp.setSelectionLimits(0, 1)

    _dropdown(inputs, "count", "Control points", ["5", "7", "9"], "7")
    _dropdown(inputs, "fiveType", "5-point type", [FIVE_CUBIC, FIVE_QUINTIC], FIVE_CUBIC)
    _dropdown(inputs, "apex", "Apex point", [APEX_LOCKED, APEX_FREE_LBL], APEX_LOCKED)
    _dropdown(inputs, "symmetry", "Symmetry", [SYM, ASYM], SYM)

    t = inputs.addFloatSliderCommandInput("tension", "Tension", "", 0.0, 1.0, False)
    t.valueOne = 0.5
    inputs.addBoolValueInput("scaleInner", "Inner points follow leg length", True, "", True)

    zero = adsk.core.ValueInput.createByReal(0.0)
    for side in "AB":
        grp = inputs.addGroupCommandInput("grp" + side, "Leg %s points" % side)
        grp.isExpanded = True
        for i in range(MAX_LEG):
            name = "Leg length" if i == 0 else "Point %d" % (i + 1)
            grp.children.addDistanceValueCommandInput("d%s%d" % (side, i), name, zero)
    inputs.addDistanceValueCommandInput("apexDist", "Apex offset", zero)

    inputs.addBoolValueInput("match", "Continue picked lines", True, "", True)
    r = inputs.addFloatSliderCommandInput("reach", "Handle reach", "", 0.1, 0.8, False)
    r.valueOne = 0.35

    inputs.addBoolValueInput("trim", "Trim lines", True, "", True)
    inputs.addBoolValueInput("dims", "Dimension points", True, "", True)

    inputs.addBoolValueInput("comb", "Show curvature comb", True, "", True)
    cs = inputs.addFloatSliderCommandInput("combScale", "Comb scale", "", 0.1, 5.0, False)
    cs.valueOne = gx_comb.get_setting("combScale", 1.0)

    inputs.addTextBoxCommandInput("info", "", "", 4, True)

    update_visibility(inputs)
    set_info(inputs, errors(inputs)[0])

    futil.add_handler(cmd.inputChanged, _on_input_changed, adsk.core.InputChangedEventHandler)
    futil.add_handler(cmd.validateInputs, _on_validate, adsk.core.ValidateInputsEventHandler)
    futil.add_handler(cmd.executePreview, _on_preview, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.execute, _on_execute, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.destroy, _on_destroy, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.mouseClick, _on_mouse_click, adsk.core.MouseEventHandler)


def _on_mouse_click(args):
    vp = args.viewportPosition
    try:
        model = futil.app.activeViewport.viewToModelSpace(vp)
    except Exception:
        model = None
    futil.debug("click: button %s, screen (%d, %d), model %s, modifiers %s"
                % (args.button, vp.x, vp.y, futil.fmt_pt(model) if model else "?", args.keyboardModifiers))


def _on_input_changed(args):
    if S.get("busy"):
        return
    S["busy"] = True
    try:
        # args.inputs is only the group's children when a grouped input fired; use the command's.
        inputs = args.firingEvent.sender.commandInputs
        id_ = args.input.id
        futil.debug("input '%s' = %s" % (id_, futil.describe_input(args.input)))
        if id_ in ("lineA", "lineB"):
            had_geo = S.get("geo") is not None
            update_geo(inputs)
            if id_ == "lineA" and _i(inputs, "lineA").selectionCount and not _i(inputs, "lineB").selectionCount:
                _i(inputs, "lineB").hasFocus = True
            geo = S.get("geo")
            if geo:
                if not had_geo:
                    set_layout(inputs, "AB", 0.4 * min(geo.availA, geo.availB))
                update_manipulators(inputs)
        elif id_ == "startPt":
            if _i(inputs, "startPt").selectionCount and not _i(inputs, "endPt").selectionCount:
                _i(inputs, "endPt").hasFocus = True
        elif id_ in ("count", "fiveType", "tension"):
            set_layout(inputs, "AB")
        elif id_ == "symmetry" and _dd(inputs, "symmetry") == ASYM:
            for i in range(MAX_LEG):
                _i(inputs, "dB%d" % i).value = _i(inputs, "dA%d" % i).value
            S["prevB"] = _i(inputs, "dB0").value
        elif id_ in ("dA0", "dB0"):
            side = id_[1]
            new, old = args.input.value, S.get("prev" + side)
            if _i(inputs, "scaleInner").value and old and old > 1e-9 and new > 1e-9:
                k = new / old
                for i in range(1, MAX_LEG):
                    inp = _i(inputs, "d%s%d" % (side, i))
                    inp.value = inp.value * k
            S["prev" + side] = new
        update_visibility(inputs)
    finally:
        S["busy"] = False


def _on_validate(args):
    inputs = args.firingEvent.sender.commandInputs
    errs = errors(inputs)
    args.areInputsValid = not errs
    if errs != S.get("last_errs"):
        S["last_errs"] = errs
        futil.log("validate: " + ("; ".join(errs) if errs else "ok"))
    if errs:
        set_info(inputs, "<span style='color:#c33'>%s</span>" % "<br>".join(errs))


def _on_preview(args):
    inputs = args.command.commandInputs
    errs = errors(inputs)
    if errs:
        futil.debug("preview skipped: %s" % "; ".join(errs))
        comb_graphics.clear(PREVIEW_KEY)
        return
    spline, info = _build(inputs, final=False)
    P = read_params(inputs)
    set_info(inputs, info)
    if P["comb"] and spline:
        comb_graphics.draw(PREVIEW_KEY, [spline], P["comb_scale"], 120)
    else:
        comb_graphics.clear(PREVIEW_KEY)
    args.isValidResult = False  # always run execute, which adds the constraints


def _build(inputs, final):
    P = read_params(inputs)
    futil.debug("%s: %s" % ("EXECUTE" if final else "preview", {k: (["%.4f" % v for v in x] if isinstance(x, list) else x)
                                                               for k, x in P.items()}))
    if P["mode"] == MODE_CORNER:
        spline, failed = fusion_geom.build_corner(S["geo"], P, final)
        info = describe(P)
    else:
        a, b = _sel_entity(inputs, "startPt").entity, _sel_entity(inputs, "endPt").entity
        spline, failed, (gs, ge, warning, n, deg) = fusion_geom.build_freeform(a.parentSketch, a, b, P, final)
        ends = []
        for name, G in (("start", gs), ("end", ge)):
            ends.append("%s G%d to its line" % (name, G) if G is not None else "%s free" % name)
        info = describe(P, "%d control points, degree %d: %s" % (n, deg, ", ".join(ends)))
        if warning:
            info += "<br><span style='color:#b60'>%s</span>" % warning
    futil.debug("  -> spline created: %s; %s" % (spline is not None, info))
    if final and failed:
        futil.log("  -> constraints that failed: %s" % failed)
        ui.messageBox("The curve was created, but some constraints couldn't be added:\n  - "
                      + "\n  - ".join(failed) + "\n\nThe details are in logs/gx_tool.log.")
    return spline, info


def _on_execute(args):
    inputs = args.command.commandInputs
    comb_graphics.clear(PREVIEW_KEY)
    P = read_params(inputs)
    _build(inputs, final=True)
    gx_comb.set_setting("combScale", P["comb_scale"])
    if P["comb"]:
        gx_comb.set_setting("combVisible", True)


def _on_destroy(args):
    futil.debug("=== GX Curve closed (reason: %s)" % args.terminationReason)
    comb_graphics.clear(PREVIEW_KEY)
    S.clear()
    gx_comb.refresh(force=True)


def start():
    futil.create_button(CMD_ID, "GX Curve",
                        "G2/G3 continuous curve: a fillet-like corner blend (symmetric or asymmetric) or a freeform spline.",
                        RESOURCES, _on_created)


def stop():
    comb_graphics.clear(PREVIEW_KEY)
    futil.delete_button(CMD_ID)
