"""GX_tool add-in entry point."""

from .lib import futil
from .commands import gx_curve, gx_comb


def run(context):
    try:
        gx_comb.start()   # comb first: gx_curve uses its drawing helpers
        gx_curve.start()
        futil.log("add-in started")
    except Exception:
        futil.handle_error("run")


def stop(context):
    try:
        gx_curve.stop()
        gx_comb.stop()
        futil.clear_handlers()
    except Exception:
        futil.handle_error("stop")
