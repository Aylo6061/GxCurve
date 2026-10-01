"""Small helpers: event-handler plumbing, logging, error reporting."""

import datetime
import os
import traceback

import adsk.core

app = adsk.core.Application.get()
ui = app.userInterface

ATTR_GROUP = "GX_tool"
WORKSPACE_ID = "FusionSolidEnvironment"
PANEL_ID = "SketchCreatePanel"

# Fusion only keeps weak references to handlers; keep them alive here.
_handlers = []


LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
LOG_FILE = os.path.join(LOG_DIR, "gx_tool.log")
LOG_MAX_BYTES = 1_000_000  # when exceeded, the log is moved to gx_tool.log.1 and restarted


def _log_to_file(msg):
    os.makedirs(LOG_DIR, exist_ok=True)
    if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > LOG_MAX_BYTES:
        os.replace(LOG_FILE, LOG_FILE + ".1")
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write("%s  %s\n" % (stamp, msg))


def log(msg):
    try:
        app.log("[GX_tool] " + str(msg))
    except Exception:
        pass
    try:
        _log_to_file(msg)
    except Exception:
        pass


VERBOSE = True  # detailed logging of inputs, selections and geometry


def debug(msg):
    if VERBOSE:
        log(msg)


def fmt_pt(p):
    try:
        return "(%.4f, %.4f, %.4f)" % (p.x, p.y, p.z)
    except Exception:
        return str(p)


def describe_entity(e):
    """One-line description of a selected Fusion entity (cm, sketch space for sketch items)."""
    if e is None:
        return "None"
    t = getattr(e, "objectType", type(e).__name__)
    try:
        if t.endswith("SketchLine"):
            s = "SketchLine %s -> %s%s" % (fmt_pt(e.startSketchPoint.geometry), fmt_pt(e.endSketchPoint.geometry),
                                          " [construction]" if e.isConstruction else "")
        elif t.endswith("SketchPoint"):
            n = e.connectedEntities.count if e.connectedEntities else 0
            s = "SketchPoint %s, %d connected" % (fmt_pt(e.geometry), n)
        else:
            s = t
        sk = getattr(e, "parentSketch", None)
        if sk:
            s += " in sketch '%s'" % sk.name
        return s
    except Exception as ex:
        return "%s (describe failed: %s)" % (t, ex)


def describe_input(inp):
    """Value of a command input, for the log."""
    t = inp.objectType.split("::")[-1]
    try:
        if t == "SelectionCommandInput":
            parts = []
            for i in range(inp.selectionCount):
                sel = inp.selection(i)
                parts.append("%s, clicked at %s" % (describe_entity(sel.entity), fmt_pt(sel.point)))
            return "%d selected%s" % (inp.selectionCount, (": " + " | ".join(parts)) if parts else "")
        if t == "DropDownCommandInput":
            return inp.selectedItem.name if inp.selectedItem else "(none)"
        if t in ("FloatSliderCommandInput", "IntegerSliderCommandInput"):
            return str(inp.valueOne)
        if t in ("DistanceValueCommandInput", "ValueCommandInput", "BoolValueCommandInput"):
            return str(inp.value)
    except Exception as ex:
        return "(describe failed: %s)" % ex
    return t


def handle_error(where, show=True):
    msg = "GX_tool error in %s:\n%s" % (where, traceback.format_exc())
    log(msg)
    if show:
        ui.messageBox(msg)


def add_handler(event, callback, handler_cls):
    """Wrap a plain function as a Fusion event handler and attach it."""

    class _H(handler_cls):
        def __init__(self):
            super().__init__()

        def notify(self, args):
            try:
                callback(args)
            except Exception:
                handle_error(getattr(callback, "__name__", "handler"))

    h = _H()
    event.add(h)
    _handlers.append((event, h))
    return h


def clear_handlers():
    for event, h in _handlers:
        try:
            event.remove(h)
        except Exception:
            pass
    _handlers.clear()


def create_button(cmd_id, name, tooltip, resources, on_created, after_id=""):
    cmd_def = ui.commandDefinitions.itemById(cmd_id)
    if cmd_def:
        cmd_def.deleteMe()
    cmd_def = ui.commandDefinitions.addButtonDefinition(cmd_id, name, tooltip, resources)
    add_handler(cmd_def.commandCreated, on_created, adsk.core.CommandCreatedEventHandler)
    panel = ui.workspaces.itemById(WORKSPACE_ID).toolbarPanels.itemById(PANEL_ID)
    ctrl = panel.controls.itemById(cmd_id)
    if ctrl:
        ctrl.deleteMe()
    ctrl = panel.controls.addCommand(cmd_def, after_id, False)
    ctrl.isPromoted = False
    return cmd_def


def delete_button(cmd_id):
    ws = ui.workspaces.itemById(WORKSPACE_ID)
    panel = ws.toolbarPanels.itemById(PANEL_ID) if ws else None
    ctrl = panel.controls.itemById(cmd_id) if panel else None
    if ctrl:
        ctrl.deleteMe()
    cmd_def = ui.commandDefinitions.itemById(cmd_id)
    if cmd_def:
        cmd_def.deleteMe()
