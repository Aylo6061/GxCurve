"""'GX Curvature Comb' command: choose which sketch curves show a comb, and show/hide them.

Combed curves are remembered with an attribute on the curve. Combs are redrawn
after every command and polled a few times a second, so they follow the curve
while you drag control points or edit dimensions.
"""

import os
import threading

import adsk.core
import adsk.fusion

from ..lib import comb_graphics, futil
from ..lib.futil import ATTR_GROUP, app, ui

CMD_ID = "GXTool_Comb"
TICK_EVENT = "GXTool_CombTick"
KEY = "persistent"
PREVIEW_KEY = "comb_cmd_preview"
RESOURCES = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "gx_comb")

_ticker = None
_tick_event = None
_last_sig = None
_suspended = False


# --------------------------------------------------------------------------- settings
def _design():
    return adsk.fusion.Design.cast(app.activeProduct)


def get_setting(name, default):
    d = _design()
    a = d.attributes.itemByName(ATTR_GROUP, name) if d else None
    if not a:
        return default
    try:
        return type(default)(a.value) if not isinstance(default, bool) else a.value == "1"
    except Exception:
        return default


def set_setting(name, value):
    d = _design()
    if d:
        d.attributes.add(ATTR_GROUP, name, ("1" if value else "0") if isinstance(value, bool) else str(value))


def settings():
    return (get_setting("combVisible", True), get_setting("combScale", 1.0), get_setting("combDensity", 80))


def combed_curves():
    d = _design()
    if not d:
        return []
    out = []
    for a in d.findAttributes(ATTR_GROUP, "comb"):
        c = adsk.fusion.SketchCurve.cast(a.parent) if a.parent else None
        if c and c.isValid:
            out.append(c)
    return out


def set_combed(curve, on):
    a = curve.attributes.itemByName(ATTR_GROUP, "comb")
    if on and not a:
        curve.attributes.add(ATTR_GROUP, "comb", "1")
    elif not on and a:
        a.deleteMe()


# --------------------------------------------------------------------------- refresh
def _signature(curves, sets):
    sig = [sets]
    for c in curves:
        try:
            ev = c.worldGeometry.evaluator
            ok, t0, t1 = ev.getParameterExtents()
            ok, pts = ev.getPointsAtParameters([t0 + (t1 - t0) * i / 6 for i in range(7)])
            sig.append(tuple(round(v, 9) for p in pts for v in (p.x, p.y, p.z)))
        except Exception:
            sig.append(None)
    return tuple(sig)


def refresh(force=False):
    global _last_sig
    if _suspended:
        return
    if not _design():
        comb_graphics.clear(KEY)
        _last_sig = None
        return
    visible, scale, density = settings()
    curves = combed_curves() if visible else []
    curves = [c for c in curves if c.parentSketch.isVisible]
    sig = _signature(curves, (visible, scale, density))
    if not force and sig == _last_sig:
        return
    _last_sig = sig
    if curves:
        comb_graphics.draw(KEY, curves, scale, density)
    else:
        comb_graphics.clear(KEY)
        app.activeViewport.refresh()


class _Ticker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.stopped = threading.Event()

    def run(self):
        while not self.stopped.wait(0.4):
            try:
                app.fireCustomEvent(TICK_EVENT, "")
            except Exception:
                pass


# --------------------------------------------------------------------------- command
def _on_created(args):
    global _suspended
    cmd = args.command
    inputs = cmd.commandInputs
    visible, scale, density = settings()

    sel = inputs.addSelectionInput("curves", "Curves", "Sketch curves to show a comb on")
    sel.addSelectionFilter("SketchCurves")
    sel.setSelectionLimits(0, 0)
    for c in combed_curves():
        sel.addSelection(c)
    inputs.addBoolValueInput("visible", "Show combs", True, "", visible)
    s = inputs.addFloatSliderCommandInput("scale", "Comb scale", "", 0.1, 5.0, False)
    s.valueOne = scale
    d = inputs.addIntegerSliderCommandInput("density", "Comb density", 10, 300, False)
    d.valueOne = density
    inputs.addTextBoxCommandInput("help", "", "Select the curves to comb. Clear the selection to remove all combs.", 2, True)

    futil.debug("=== Comb command opened; %d curve(s) currently combed; visible=%s scale=%s density=%s"
                % (sel.selectionCount, visible, scale, density))
    _suspended = True
    comb_graphics.clear(KEY)
    futil.add_handler(cmd.inputChanged, _on_input_changed, adsk.core.InputChangedEventHandler)
    futil.add_handler(cmd.executePreview, _on_preview, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.execute, _on_execute, adsk.core.CommandEventHandler)
    futil.add_handler(cmd.destroy, _on_destroy, adsk.core.CommandEventHandler)


def _read(inputs):
    sel = inputs.itemById("curves")
    curves = [sel.selection(i).entity for i in range(sel.selectionCount)]
    return (curves, inputs.itemById("visible").value,
            inputs.itemById("scale").valueOne, inputs.itemById("density").valueOne)


def _on_input_changed(args):
    futil.debug("comb input '%s' = %s" % (args.input.id, futil.describe_input(args.input)))


def _on_preview(args):
    curves, visible, scale, density = _read(args.command.commandInputs)
    if visible:
        comb_graphics.draw(PREVIEW_KEY, curves, scale, density)
    else:
        comb_graphics.clear(PREVIEW_KEY)


def _on_execute(args):
    curves, visible, scale, density = _read(args.command.commandInputs)
    futil.debug("comb EXECUTE: %d curve(s), visible=%s scale=%.2f density=%d"
                % (len(curves), visible, scale, density))
    keep = set()
    for c in curves:
        set_combed(c, True)
        keep.add(c.entityToken)
    for c in combed_curves():
        if c.entityToken not in keep:
            set_combed(c, False)
    set_setting("combVisible", visible)
    set_setting("combScale", scale)
    set_setting("combDensity", density)


def _on_destroy(args):
    global _suspended
    _suspended = False
    comb_graphics.clear(PREVIEW_KEY)
    refresh(force=True)


def _on_terminated(args):
    if args.commandId != CMD_ID:
        refresh()


def _on_doc(args):
    refresh(force=True)


def _on_tick(args):
    refresh()


def start():
    global _ticker, _tick_event
    futil.create_button(CMD_ID, "GX Curvature Comb",
                        "Show or hide curvature combs on sketch curves (updates live while you drag).",
                        RESOURCES, _on_created)
    futil.add_handler(ui.commandTerminated, _on_terminated, adsk.core.ApplicationCommandEventHandler)
    futil.add_handler(app.documentActivated, _on_doc, adsk.core.DocumentEventHandler)
    try:
        app.unregisterCustomEvent(TICK_EVENT)
    except Exception:
        pass
    _tick_event = app.registerCustomEvent(TICK_EVENT)
    futil.add_handler(_tick_event, _on_tick, adsk.core.CustomEventHandler)
    _ticker = _Ticker()
    _ticker.start()
    refresh(force=True)


def stop():
    global _ticker
    if _ticker:
        _ticker.stopped.set()
        _ticker = None
    try:
        app.unregisterCustomEvent(TICK_EVENT)
    except Exception:
        pass
    comb_graphics.clear_all()
    futil.delete_button(CMD_ID)
