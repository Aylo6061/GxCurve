"""'GX G3 Fillet' command: like Sketch Fillet, but makes a G3 (7 points) or G4 (9 points)
curve that stands in for a radius-R arc.

By default the leg-point spacing gives the smoothest possible comb (gx_math.fillet_smoothest,
the same thing GX Smooth's curvature flow aims for). "Emulate fillet" instead uses the spacing
that hugs the true arc most closely (gx_math.fillet_shape). "Match" picks the size. The curve
is built and constrained exactly like a symmetric GX Curve corner.

If either pick is a curve (arc, circle, ellipse, conic, spline) the blend is G3 to that curve's
own curvature (gx_math.curve_fillet). Sketch constraints can't hold G3 to a curve, so such
blends are made fixed, with coincident (and where Fusion allows, G2 smooth) constraints.
"""

import math
import os

import adsk.core
import adsk.fusion

from ..lib import comb_graphics, fusion_geom, futil
from ..lib import gx_math as g
from . import gx_comb

CMD_ID = "GXTool_Fillet"
PREVIEW_KEY = "gx_fillet_preview"
ARC_KEY = "gx_fillet_arc"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gx_fillet")

COUNTS = {"7 points (G3)": 7, "9 points (G4)": 9}
MATCHES = {
    "Best fit to the fillet": g.FILLET_BEST,
    "Same depth into the corner": g.FILLET_DEPTH,
    "Same tangent points": g.FILLET_TANGENT,
    "Never tighter than R": g.FILLET_MINRADIUS,
}

S = {}


def _i(inputs, id_):
    return inputs.itemById(id_)


def _dd(inputs, id_, table):
    return table[_i(inputs, id_).selectedItem.name]


def _on_created(args):
    S.clear()
    cmd = args.command
    inputs = cmd.commandInputs
    futil.debug("=== GX G3 Fillet opened")

    for id_, name in (("lineA", "Curve A"), ("lineB", "Curve B")):
        s = inputs.addSelectionInput(id_, name, "Pick a line or curve, on the side to keep")
        s.addSelectionFilter("SketchCurves")
        s.setSelectionLimits(0, 1)
    inputs.addDistanceValueCommandInput("radius", "Radius", adsk.core.ValueInput.createByReal(1.0))
    for id_, name, table in (("count", "Curve", COUNTS), ("match", "Match", MATCHES)):
        dd = inputs.addDropDownCommandInput(id_, name, adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, label in enumerate(table):
            dd.listItems.add(label, k == 0, "")
    inputs.addBoolValueInput("emulate", "Emulate fillet", True, "", False)
    inputs.addBoolValueInput("trim", "Trim", True, "", True)
    inputs.addBoolValueInput("dims", "Dimension points", True, "", True)
    inputs.addBoolValueInput("arc", "Show true fillet arc", True, "", True)
    inputs.addBoolValueInput("comb", "Show curvature comb", True, "", True)
    cs = inputs.addFloatSliderCommandInput("combScale", "Comb scale", "", 0.1, 5.0, False)
    cs.valueOne = gx_comb.get_setting("combScale", 1.0)
    inputs.addTextBoxCommandInput("info", "", "Select two lines.", 5, True)
    _i(inputs, "radius").isVisible = False

    futil.add_handler(cmd.inputChanged, _on_input_changed, adsk.core.InputChangedEventHandler)
    futil.add_handler(cmd.validateInputs, _on_validate, adsk.core.ValidateInputsEventHandler)
    futil.add_handler(cmd.executePreview, _on_preview, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.execute, _on_execute, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.destroy, _on_destroy, adsk.core.CommandEventHandler)


def _sel(inputs, id_):
    s = _i(inputs, id_)
    return s.selection(0) if s.selectionCount else None


def _phi(geo):
    return math.acos(max(-1.0, min(1.0, g.dot(geo.uA, geo.uB))))


def _update_geo(inputs):
    S["geo"], S["cgeo"], S["geo_err"], S["cache"] = None, None, "", {}
    a, b = _sel(inputs, "lineA"), _sel(inputs, "lineB")
    if not (a and b):
        return
    try:
        if adsk.fusion.SketchLine.cast(a.entity) and adsk.fusion.SketchLine.cast(b.entity):
            geo = fusion_geom.CornerGeo(a.entity, a.point, b.entity, b.point)
            S["geo"] = geo
            futil.debug("fillet corner: at (%.4f, %.4f), angle %.2f deg, availA %.4f availB %.4f"
                        % (geo.corner[0], geo.corner[1], math.degrees(_phi(geo)), geo.availA, geo.availB))
        else:
            S["cgeo"] = fusion_geom.CurveCornerGeo(a.entity, a.point, b.entity, b.point)
    except ValueError as e:
        S["geo_err"] = str(e)
        futil.debug("fillet corner rejected: %s" % e)


def _curves():
    return S.get("cgeo") is not None


def _curve_result(inputs):
    """Cached curve_fillet result (each solve takes ~1-7 s)."""
    key = (round(_i(inputs, "radius").value, 9), _dd(inputs, "count", COUNTS), _dd(inputs, "match", MATCHES),
           _i(inputs, "emulate").value)
    cache = S.setdefault("cache", {})
    if key not in cache:
        try:
            cache[key] = S["cgeo"].fillet(key[0], key[1], key[2], key[3])
            r = cache[key]
            futil.debug("curve fillet R=%.5f: contacts u=%.5f v=%.5f, gap %.4g, tightest radius %.4g"
                        % (key[0], r["ua"], r["ub"], r["max_gap"], r["min_radius"]))
        except ValueError as e:
            cache[key] = str(e)
            futil.debug("curve fillet R=%.5f failed: %s" % (key[0], e))
    return cache[key]


def _on_input_changed(args):
    inputs = args.firingEvent.sender.commandInputs
    id_ = args.input.id
    futil.debug("fillet input '%s' = %s" % (id_, futil.describe_input(args.input)))
    if id_ in ("lineA", "lineB"):
        had = S.get("geo") is not None or _curves()
        _update_geo(inputs)
        if id_ == "lineA" and _i(inputs, "lineA").selectionCount and not _i(inputs, "lineB").selectionCount:
            _i(inputs, "lineB").hasFocus = True
        geo = S.get("geo")
        r = _i(inputs, "radius")
        r.isVisible = geo is not None or _curves()
        _i(inputs, "dims").isVisible = not _curves()
        if _curves():
            cg = S["cgeo"]
            if not had:
                ref = [p for p in (_sel(inputs, "lineA").point, _sel(inputs, "lineB").point) if p]
                d = min(g.length(g.sub(fusion_geom.p2(cg.sketch.modelToSketchSpace(p)), cg.X)) for p in ref)
                r.value = 0.3 * d
            r.setManipulator(fusion_geom.model_point(cg.sketch, cg.X), fusion_geom.model_dir(cg.sketch, cg.X, cg.bis))
        elif geo:
            if not had:
                # default radius: legs use about half of the shorter line
                shape = g.fillet_shape if _i(inputs, "emulate").value else g.fillet_smoothest
                fr, L_fit, _, _, _ = shape(_phi(geo), _dd(inputs, "count", COUNTS))
                r.value = 0.5 * min(geo.availA, geo.availB) / L_fit
            sk = geo.sketch
            r.setManipulator(fusion_geom.model_point(sk, geo.corner), fusion_geom.model_dir(sk, geo.corner, geo.bis))


def _layout(inputs):
    geo = S["geo"]
    return g.fillet_layout(_phi(geo), _i(inputs, "radius").value, _dd(inputs, "count", COUNTS),
                           _dd(inputs, "match", MATCHES), emulate=_i(inputs, "emulate").value)


def _errors(inputs):
    if S.get("geo_err"):
        return [S["geo_err"]]
    if not (S.get("geo") or _curves()):
        return ["Select two sketch lines or curves (click each on the side you want to keep)."]
    if _i(inputs, "radius").value <= 0:
        return ["The radius must be above zero."]
    if _curves():
        res = _curve_result(inputs)
        return [res] if isinstance(res, str) else []
    geo = S["geo"]
    lay = _layout(inputs)
    if _i(inputs, "trim").value and lay["L"] >= min(geo.availA, geo.availB) - 1e-7:
        return ["This curve needs %.4g cm along each line, but a line is only %.4g cm long. "
                "Use a smaller radius or turn off Trim." % (lay["L"], min(geo.availA, geo.availB))]
    return []


def _on_validate(args):
    inputs = args.firingEvent.sender.commandInputs
    errs = _errors(inputs)
    args.areInputsValid = not errs
    if errs != S.get("last_errs"):
        S["last_errs"] = errs
        futil.debug("fillet validate: " + ("; ".join(errs) if errs else "ok"))
    if errs:
        _i(inputs, "info").formattedText = "<span style='color:#c33'>%s</span>" % "<br>".join(errs)


def _params(inputs, lay):
    count = _dd(inputs, "count", COUNTS)
    return dict(mode="fillet", count=count, five_type="cubic", apex=g.APEX_CORNER, symmetric=True,
                distA=lay["dist"], distB=lay["dist"], apex_dist=0.0,
                trim=_i(inputs, "trim").value, dims=_i(inputs, "dims").value,
                comb=_i(inputs, "comb").value, fillet_radius=_i(inputs, "radius").value,
                fillet_match=_dd(inputs, "match", MATCHES), fillet_emulate=_i(inputs, "emulate").value)


def _arc_polylines(sk, c, R, ta, tb, ua, ub):
    """Green reference arc from tangent point ta to tb (2D) plus tick marks; ua/ub = tangents."""
    a0 = math.atan2(ta[1] - c[1], ta[0] - c[0])
    a1 = math.atan2(tb[1] - c[1], tb[0] - c[0])
    sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi  # the short way round
    pts = [(c[0] + R * math.cos(a0 + sweep * i / 64), c[1] + R * math.sin(a0 + sweep * i / 64)) for i in range(65)]
    out = [[fusion_geom.model_point(sk, p) for p in pts]]
    tick = 0.04 * R
    for t, u in ((ta, ua), (tb, ub)):
        n = (-u[1], u[0])
        out.append([fusion_geom.model_point(sk, g.add(t, g.mul(n, tick))),
                    fusion_geom.model_point(sk, g.sub(t, g.mul(n, tick)))])
    return out


def _draw_arc(inputs):
    if not _i(inputs, "arc").value:
        comb_graphics.clear(ARC_KEY)
        return
    R = _i(inputs, "radius").value
    if _curves():
        cg, res = S["cgeo"], _curve_result(inputs)
        arc = res["arc"]
        ta, tb = cg.A.point(arc["ua"]), cg.B.point(arc["ub"])
        comb_graphics.draw_polylines(ARC_KEY, _arc_polylines(cg.sketch, arc["centre"], R, ta, tb,
                                                             cg.A.frame(arc["ua"], 1)["T"], cg.B.frame(arc["ub"], 1)["T"]),
                                     (90, 160, 90), 2)
        return
    geo = S["geo"]
    phi = _phi(geo)
    c = g.add(geo.corner, g.mul(geo.bis, R / math.sin(phi / 2)))
    ta = g.add(geo.corner, g.mul(geo.uA, R / math.tan(phi / 2)))
    tb = g.add(geo.corner, g.mul(geo.uB, R / math.tan(phi / 2)))
    a0 = math.atan2(ta[1] - c[1], ta[0] - c[0])
    a1 = math.atan2(tb[1] - c[1], tb[0] - c[0])
    sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi  # the short way round
    pts = [(c[0] + R * math.cos(a0 + sweep * i / 64), c[1] + R * math.sin(a0 + sweep * i / 64)) for i in range(65)]
    sk = geo.sketch
    arc = [fusion_geom.model_point(sk, p) for p in pts]
    tick = 0.04 * R
    ticks = []
    for t, u in ((ta, geo.uA), (tb, geo.uB)):  # small marks at the arc's tangent points
        n = (-u[1], u[0])
        ticks.append([fusion_geom.model_point(sk, g.add(t, g.mul(n, tick))),
                      fusion_geom.model_point(sk, g.sub(t, g.mul(n, tick)))])
    comb_graphics.draw_polylines(ARC_KEY, [arc] + ticks, (90, 160, 90), 2)


def _info(lay, R, count, emulate=False):
    G = 3 if count == 7 else 4
    return ("<b>G%d curve, %s, standing in for an R %.4g fillet</b><br>"
            "Legs: %.4g cm (%.2f× the fillet's %.4g)<br>"
            "Tightest radius: %.4g (%.0f%% of R)<br>"
            "Largest gap to the true fillet: %.3g cm (%.1f%% of R)"
            % (G, "closest to the arc" if emulate else "smoothest shape", R, lay["L"], lay["L"] / lay["arc_tangent"], lay["arc_tangent"],
               lay["min_radius"], 100 * lay["min_radius"] / R, lay["max_gap"], 100 * lay["max_gap"] / R))


def _curve_info(res, R, count, emulate):
    return ("<b>G%d blend into the curves' own curvature, %s, standing in for an R %.4g fillet</b><br>"
            "Tightest radius: %.4g (%.0f%% of R)<br>"
            "Largest gap to the true fillet: %.3g cm (%.1f%% of R)<br>"
            "The blend is fixed: sketch constraints can't hold G3 to a curve."
            % (res["G"], "closest to the arc" if emulate else "smoothest shape", R,
               res["min_radius"], 100 * res["min_radius"] / R, res["max_gap"], 100 * res["max_gap"] / R))


def _build_curves(inputs, final):
    res = _curve_result(inputs)
    R = _i(inputs, "radius").value
    P = dict(mode="curve_fillet", count=_dd(inputs, "count", COUNTS), trim=_i(inputs, "trim").value,
             comb=_i(inputs, "comb").value, fillet_radius=R, fillet_match=_dd(inputs, "match", MATCHES),
             fillet_emulate=_i(inputs, "emulate").value)
    futil.debug("curve fillet %s: R=%.5f count=%d" % ("EXECUTE" if final else "preview", R, P["count"]))
    spline, failed = fusion_geom.build_curve_blend(S["cgeo"], res, P, final)
    if final and failed:
        futil.log("  -> problems: %s" % failed)
        futil.ui.messageBox("The blend was created, but:\n  - " + "\n  - ".join(failed)
                            + "\n\nThe details are in logs/gx_tool.log.")
    return spline, res, P


def _build(inputs, final):
    lay = _layout(inputs)
    P = _params(inputs, lay)
    futil.debug("fillet %s: R=%.5f count=%d match=%s L=%.5f dist=%s" % (
        "EXECUTE" if final else "preview", P["fillet_radius"], P["count"], P["fillet_match"], lay["L"],
        ["%.5f" % d for d in lay["dist"]]))
    spline, failed = fusion_geom.build_corner(S["geo"], P, final)
    if final and failed:
        futil.log("  -> constraints that failed: %s" % failed)
        futil.ui.messageBox("The curve was created, but some constraints couldn't be added:\n  - "
                            + "\n  - ".join(failed) + "\n\nThe details are in logs/gx_tool.log.")
    return spline, lay, P


def _on_preview(args):
    inputs = args.command.commandInputs
    if _errors(inputs):
        comb_graphics.clear(PREVIEW_KEY)
        comb_graphics.clear(ARC_KEY)
        return
    if _curves():
        spline, res, P = _build_curves(inputs, final=False)
        _i(inputs, "info").formattedText = _curve_info(res, P["fillet_radius"], P["count"], P["fillet_emulate"])
    else:
        spline, lay, P = _build(inputs, final=False)
        _i(inputs, "info").formattedText = _info(lay, P["fillet_radius"], P["count"], P["fillet_emulate"])
    _draw_arc(inputs)
    if P["comb"] and spline:
        comb_graphics.draw(PREVIEW_KEY, [spline], _i(inputs, "combScale").valueOne, 120)
    else:
        comb_graphics.clear(PREVIEW_KEY)
    args.isValidResult = False  # execute adds the constraints


def _on_execute(args):
    inputs = args.command.commandInputs
    comb_graphics.clear(PREVIEW_KEY)
    comb_graphics.clear(ARC_KEY)
    _, _, P = (_build_curves if _curves() else _build)(inputs, final=True)
    gx_comb.set_setting("combScale", _i(inputs, "combScale").valueOne)
    if P["comb"]:
        gx_comb.set_setting("combVisible", True)


def _on_destroy(args):
    futil.debug("=== GX G3 Fillet closed (reason: %s)" % args.terminationReason)
    comb_graphics.clear(PREVIEW_KEY)
    comb_graphics.clear(ARC_KEY)
    S.clear()
    gx_comb.refresh(force=True)


def start():
    futil.create_button(CMD_ID, "GX G3 Fillet",
                        "Like Fillet, but makes a G3 (or G4) curve that approximates a radius-R arc.",
                        RESOURCES, _on_created)


def stop():
    comb_graphics.clear(PREVIEW_KEY)
    comb_graphics.clear(ARC_KEY)
    futil.delete_button(CMD_ID)
