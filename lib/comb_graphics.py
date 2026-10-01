"""Curvature combs drawn as custom graphics (display only, never sketch geometry)."""

import adsk.core
import adsk.fusion

from .futil import app, debug, log

_groups = {}  # key -> CustomGraphicsGroup

TEETH_RGB = (230, 60, 160)
ENVELOPE_RGB = (120, 20, 90)


def _design():
    return adsk.fusion.Design.cast(app.activeProduct)


def clear(key):
    grp = _groups.pop(key, None)
    if grp is not None:
        try:
            if grp.isValid:
                grp.deleteMe()
        except Exception:
            pass


def clear_all():
    for k in list(_groups):
        clear(k)


def _effect(rgb):
    return adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255))


def sample(curve, density):
    """[(points, curvature_dirs, curvatures, length)] for a sketch curve, in model space."""
    ev = curve.worldGeometry.evaluator
    ok, t0, t1 = ev.getParameterExtents()
    if not ok:
        return None
    params = [t0 + (t1 - t0) * i / density for i in range(density + 1)]
    ok1, pts = ev.getPointsAtParameters(params)
    ok2, dirs, ks = ev.getCurvatures(params)
    ok3, length = ev.getLengthAtParameter(t0, t1)
    if not (ok1 and ok2):
        return None
    return pts, dirs, ks, (length if ok3 else 1.0)


def draw(key, curves, scale=1.0, density=60):
    """Draw combs for the given sketch curves. Tooth lengths share one scale across all
    curves, so a curvature jump at a joint between two curves is visible."""
    clear(key)
    design = _design()
    if not design or not curves:
        return
    data = []
    for c in curves:
        try:
            s = sample(c, density)
            if s:
                data.append(s)
        except Exception as e:
            log("comb sample failed: %s" % e)
    kmax = max((abs(k) for _, _, ks, _ in data for k in ks), default=0.0)
    lref = max((d[3] for d in data), default=0.0)
    debug("comb '%s': %d curve(s), %d sampled, max curvature %.6g, ref length %.4f, scale %.2f"
          % (key, len(curves), len(data), kmax, lref, scale))
    if not data or kmax < 1e-12:
        return
    factor = 0.25 * lref * scale / kmax

    grp = design.rootComponent.customGraphicsGroups.add()
    _groups[key] = grp
    for pts, dirs, ks, _ in data:
        teeth, env = [], []
        for p, d, k in zip(pts, dirs, ks):
            if d is None or d.length < 1e-12 or abs(k) < 1e-12:
                tip = p
            else:
                v = d.copy()
                v.normalize()
                v.scaleBy(-k * factor)  # away from the centre of curvature
                tip = p.copy()
                tip.translateBy(v)
            teeth += [p.x, p.y, p.z, tip.x, tip.y, tip.z]
            env += [tip.x, tip.y, tip.z]
        tl = grp.addLines(adsk.fusion.CustomGraphicsCoordinates.create(teeth), [], False, [])
        tl.color = _effect(TEETH_RGB)
        el = grp.addLines(adsk.fusion.CustomGraphicsCoordinates.create(env), [], True, [])
        el.color = _effect(ENVELOPE_RGB)
        el.weight = 2
    for i in range(grp.count):
        try:
            grp.item(i).isSelectable = False
        except Exception:
            pass
    app.activeViewport.refresh()
