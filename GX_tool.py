"""GX_tool add-in entry point."""

from .lib import futil
from .commands import gx_curve, gx_comb, gx_edge_fillet, gx_fillet, gx_smooth, lip, polygrid, screw_boss, sketch_polygrid


def run(context):
    try:
        gx_comb.start()   # comb first: the others use its drawing helpers
        gx_curve.start()
        gx_fillet.start()
        gx_smooth.start()
        gx_edge_fillet.start()
        sketch_polygrid.start()
        polygrid.start()
        screw_boss.start()
        lip.start()
        futil.log("add-in started")
    except Exception:
        futil.handle_error("run")


def stop(context):
    try:
        lip.stop()
        screw_boss.stop()
        polygrid.stop()
        sketch_polygrid.stop()
        gx_edge_fillet.stop()
        gx_smooth.stop()
        gx_fillet.stop()
        gx_curve.stop()
        gx_comb.stop()
        futil.clear_handlers()
    except Exception:
        futil.handle_error("stop")
