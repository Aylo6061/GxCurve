"""Pure-Python geometry for GX_tool (no Fusion imports, so it is unit-testable).

All points/vectors are 2D tuples (x, y) in sketch space.

Continuity rule used throughout: for a clamped B-spline, the k-th derivative at
an end depends only on the first k+1 control points.  If the first c control
points are collinear with a straight leg, derivatives 1..c-1 are all parallel to
the leg, so the curve meets the leg with G(c-1) continuity.
"""

import math

EPS = 1e-12


# --------------------------------------------------------------------------- vectors
def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def length(a):
    return math.hypot(a[0], a[1])


def normalize(a):
    n = length(a)
    if n < EPS:
        raise ValueError("zero-length vector")
    return (a[0] / n, a[1] / n)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def line_intersection(p1, d1, p2, d2):
    """Intersection of infinite lines p1 + s*d1 and p2 + t*d2. Returns (point, s, t) or None."""
    den = cross(d1, d2)
    if abs(den) < 1e-9 * max(length(d1) * length(d2), EPS):
        return None
    w = sub(p2, p1)
    s = cross(w, d2) / den
    t = cross(w, d1) / den
    return add(p1, mul(d1, s)), s, t


# --------------------------------------------------------------------------- B-splines
def degree_for_count(n):
    """Fusion control-point splines are degree 3 or 5; degree 5 needs >= 6 points."""
    return 3 if n < 6 else 5


def clamped_knots(n, p):
    inner = n - p - 1
    return [0.0] * (p + 1) + [i / (inner + 1) for i in range(1, inner + 1)] + [1.0] * (p + 1)


def _find_span(knots, p, n, t):
    if t >= knots[n]:
        return n - 1
    lo, hi = p, n
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if t < knots[mid]:
            hi = mid
        else:
            lo = mid
    return lo


def bspline_point(ctrl, p, knots, t):
    """De Boor evaluation."""
    n = len(ctrl)
    k = _find_span(knots, p, n, t)
    d = [ctrl[j + k - p] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = j + k - p
            den = knots[i + p - r + 1] - knots[i]
            a = 0.0 if den < EPS else (t - knots[i]) / den
            d[j] = lerp(d[j - 1], d[j], a)
    return d[p]


def bspline_derivative(ctrl, p, knots):
    """Hodograph: returns (ctrl', p-1, knots') of the derivative curve."""
    q = []
    for i in range(len(ctrl) - 1):
        den = knots[i + p + 1] - knots[i + 1]
        q.append(mul(sub(ctrl[i + 1], ctrl[i]), p / den) if den > EPS else (0.0, 0.0))
    return q, p - 1, knots[1:-1]


class Spline:
    """Clamped uniform B-spline with cached derivative curves."""

    def __init__(self, ctrl, degree=None):
        self.ctrl = [tuple(c) for c in ctrl]
        self.p = degree if degree is not None else degree_for_count(len(ctrl))
        self.knots = clamped_knots(len(ctrl), self.p)
        self._derivs = [(self.ctrl, self.p, self.knots)]
        while len(self._derivs) <= 4 and self._derivs[-1][1] > 0:
            self._derivs.append(bspline_derivative(*self._derivs[-1]))

    def d(self, t, order=0):
        if order >= len(self._derivs):
            return (0.0, 0.0)
        c, p, k = self._derivs[order]
        if p == 0:
            # piecewise constant
            n = len(c)
            return c[_find_span(k, 0, n, t)]
        return bspline_point(c, p, k, t)

    def point(self, t):
        return self.d(t, 0)

    def curvature(self, t):
        """Signed curvature (positive = turning left)."""
        d1, d2 = self.d(t, 1), self.d(t, 2)
        s = length(d1)
        return cross(d1, d2) / (s ** 3) if s > EPS else 0.0

    def curvature_rate(self, t):
        """d(kappa)/ds, the quantity that must vanish at a G3 join with a line."""
        d1, d2, d3 = self.d(t, 1), self.d(t, 2), self.d(t, 3)
        s2 = dot(d1, d1)
        if s2 < EPS:
            return 0.0
        dk_dt = (cross(d1, d3) * s2 - 3.0 * cross(d1, d2) * dot(d1, d2)) / (s2 ** 2.5)
        return dk_dt / math.sqrt(s2)

    def comb(self, samples=60, scale=1.0):
        """[(point, tooth_end, kappa)]; teeth point away from the centre of curvature."""
        out = []
        for i in range(samples + 1):
            t = i / samples
            pt = self.point(t)
            tan = self.d(t, 1)
            if length(tan) < EPS:
                continue
            nrm = normalize((-tan[1], tan[0]))  # left normal
            k = self.curvature(t)
            out.append((pt, sub(pt, mul(nrm, k * scale)), k))
        return out


def collinear_count(ctrl, from_end=False, tol=1e-9):
    """How many control points, counted from one end, lie on the line of the first two."""
    pts = list(reversed(ctrl)) if from_end else list(ctrl)
    if len(pts) < 2:
        return len(pts)
    base = pts[0]
    u = sub(pts[1], base)
    if length(u) < EPS:
        return 1
    u = normalize(u)
    c = 2
    for q in pts[2:]:
        if abs(cross(u, sub(q, base))) > tol * max(1.0, length(sub(q, base))):
            break
        c += 1
    return c


def end_continuity_to_line(ctrl, degree=None, from_end=False):
    """G-order at an end when joined to a straight line along the end tangent."""
    p = degree if degree is not None else degree_for_count(len(ctrl))
    return min(collinear_count(ctrl, from_end) - 1, p)


# --------------------------------------------------------------------------- corner layout
# Corner "styles":
#   count 5, 'cubic'   : A A C B B            degree 3   G2 ends
#   count 5, 'quintic' : A A A B B B (6 pts)  degree 5   G2 ends, single span
#   count 7, apex corner: A A A C B B B       degree 5   G3 ends
#   count 7, apex free  : A A A X B B B       degree 5   G2 ends, apex pull
#   count 9, apex corner: A A A A C B B B B   degree 5   G4 ends
#   count 9, apex free  : A A A A X B B B B   degree 5   G3 ends, apex pull

APEX_CORNER = "corner"
APEX_FREE = "free"
APEX_NONE = "none"


def corner_style(count, five_type="cubic", apex=APEX_CORNER):
    """Returns (points_per_leg, apex_mode, total_cp_count, degree)."""
    if count == 5:
        if five_type == "quintic":
            return 3, APEX_NONE, 6, 5
        return 2, APEX_CORNER, 5, 3
    if count in (7, 9):
        m = (count - 1) // 2
        return m, (APEX_FREE if apex == APEX_FREE else APEX_CORNER), count, 5
    raise ValueError("control point count must be 5, 7 or 9")


def expected_corner_continuity(count, five_type="cubic", apex=APEX_CORNER):
    m, apex_mode, _, p = corner_style(count, five_type, apex)
    c = m + (1 if apex_mode == APEX_CORNER else 0)
    return min(c - 1, p)


def tension_fractions(m, tension=0.5):
    """Fractions of the leg length for the m leg points, outer (1.0) to inner.

    tension 0.5 = even spacing; higher pulls the inner points toward the corner
    (tighter curve), lower pushes them outward (fuller, flatter curve).
    """
    tension = min(max(tension, 0.0), 1.0)
    gamma = 2.0 ** ((tension - 0.5) * 2.0)  # 0.5 .. 2
    return [((m - j) / m) ** gamma for j in range(m)]


def bisector(uA, uB):
    b = add(uA, uB)
    if length(b) < 1e-9:
        return (-uA[1], uA[0])
    return normalize(b)


def corner_control_points(corner, uA, uB, distA, distB, apex_mode, apex_dist=0.0):
    """Control points A-outer .. A-inner, [apex], B-inner .. B-outer.

    distA/distB: distances from the corner, outer to inner (descending).
    uA/uB: unit vectors from the corner along each leg.
    """
    pts = [add(corner, mul(uA, d)) for d in distA]
    if apex_mode == APEX_CORNER:
        pts.append(tuple(corner))
    elif apex_mode == APEX_FREE:
        pts.append(add(corner, mul(bisector(uA, uB), apex_dist)))
    pts += [add(corner, mul(uB, d)) for d in reversed(distB)]
    return pts


def check_distances(dist):
    """True if distances are positive and strictly descending (outer to inner)."""
    if not dist or dist[-1] <= 0:
        return False
    return all(dist[i] > dist[i + 1] for i in range(len(dist) - 1))


# --------------------------------------------------------------------------- freeform layout
def freeform_control_points(p_start, p_end, count, dir_start=None, dir_end=None, reach=0.35):
    """Initial control points for a freeform curve.

    dir_start / dir_end: unit vectors pointing from the picked point *into* the new
    curve, continuing the existing line through that point; None = free end.
    Returns (ctrl, degree, g_start, g_end, warning)
    """
    n = count
    p = degree_for_count(n)
    target = 3 if n == 5 else 4  # points on the ray -> G2 / G3
    D = length(sub(p_end, p_start))
    if D < EPS:
        raise ValueError("start and end points coincide")
    warning = ""

    def ray(p0, u, c, span):
        return [add(p0, mul(u, span * j / (c - 1))) for j in range(c)]

    if dir_start is not None and dir_end is not None:
        c = target
        if 2 * c <= n:
            a = ray(p_start, dir_start, c, D * reach)
            b = ray(p_end, dir_end, c, D * reach)
            free = n - 2 * c
            mids = [lerp(a[-1], b[-1], (i + 1) / (free + 1)) for i in range(free)]
            ctrl = a + mids + list(reversed(b))
        else:
            hit = line_intersection(p_start, dir_start, p_end, dir_end)
            if hit and hit[1] > EPS and hit[2] > EPS:
                x = hit[0]
                k = c - 1
                a = [lerp(p_start, x, j / k) for j in range(k)]
                b = [lerp(p_end, x, j / k) for j in range(k)]
                ctrl = a + [x] + list(reversed(b))
            else:
                c = n // 2
                warning = ("The two line directions don't meet in front of the points, so this "
                           "count can only reach G%d. Use more control points for G%d." % (c - 1, target - 1))
                a = ray(p_start, dir_start, c, D * reach)
                b = ray(p_end, dir_end, c, D * reach)
                free = n - 2 * c
                mids = [lerp(a[-1], b[-1], (i + 1) / (free + 1)) for i in range(free)]
                ctrl = a + mids + list(reversed(b))
    elif dir_start is not None or dir_end is not None:
        flip = dir_start is None
        s, e, u = (p_end, p_start, dir_end) if flip else (p_start, p_end, dir_start)
        a = ray(s, u, target, D * reach)
        rest = n - target
        tail = [lerp(a[-1], e, (i + 1) / rest) for i in range(rest)]
        ctrl = a + tail
        if flip:
            ctrl = list(reversed(ctrl))
    else:
        ctrl = [lerp(p_start, p_end, i / (n - 1)) for i in range(n)]
        # give it a gentle bow so there is something to drag
        nrm = normalize((-(p_end[1] - p_start[1]), p_end[0] - p_start[0]))
        ctrl = [add(q, mul(nrm, D * 0.15 * math.sin(math.pi * i / (n - 1)))) for i, q in enumerate(ctrl)]
        ctrl[0], ctrl[-1] = tuple(p_start), tuple(p_end)

    g_start = end_continuity_to_line(ctrl, p) if dir_start is not None else None
    g_end = end_continuity_to_line(ctrl, p, from_end=True) if dir_end is not None else None
    return ctrl, p, g_start, g_end, warning
