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
    """Clamped B-spline with cached derivative curves (uniform knots unless given)."""

    def __init__(self, ctrl, degree=None, knots=None):
        self.ctrl = [tuple(c) for c in ctrl]
        self.p = degree if degree is not None else degree_for_count(len(ctrl))
        if knots is None:
            self.knots = clamped_knots(len(ctrl), self.p)
        else:  # normalise to [0, 1]
            k0, k1 = knots[0], knots[-1]
            self.knots = [(k - k0) / (k1 - k0) for k in knots]
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


# --------------------------------------------------------------------------- fairing (smoothing)
# A spline is C(t) = sum_i N_i(t) P_i, linear in the control points, so any energy of the
# form  integral |C^(k)(t)|^2 dt  is a quadratic form in the control points:
#     E = sum_ij G_ij (P_i . P_j),   G_ij = integral N_i^(k)(t) N_j^(k)(t) dt
# Writing the movable control points as affine functions of a few free variables x
# (fixed / sliding along a line / free in the plane), E plus a "stay near the original"
# term is quadratic in x, so the smoothest shape is one linear solve.

ENERGY_VELOCITY, ENERGY_ACCEL, ENERGY_JERK = 1, 2, 3

_GAUSS = [(-0.9324695142, 0.1713244924), (-0.6612093865, 0.3607615730), (-0.2386191861, 0.4679139346),
          (0.2386191861, 0.4679139346), (0.6612093865, 0.3607615730), (0.9324695142, 0.1713244924)]


def gram_matrix(n, p, knots, order):
    """G_ij = integral over [0,1] of N_i^(order) * N_j^(order). Exact for degree <= 5 (6-point Gauss)."""
    basis = [Spline([(1.0 if j == i else 0.0, 0.0) for j in range(n)], p, knots) for i in range(n)]
    ks = basis[0].knots
    G = [[0.0] * n for _ in range(n)]
    breaks = sorted(set(ks))
    for a, b in zip(breaks, breaks[1:]):
        h = (b - a) / 2
        for x, w in _GAUSS:
            t = a + h * (x + 1)
            v = [bs.d(t, order)[0] for bs in basis]
            for i in range(n):
                if v[i] == 0.0:
                    continue
                for j in range(n):
                    G[i][j] += w * h * v[i] * v[j]
    return G


def solve(A, b):
    """Gaussian elimination with partial pivoting (small dense systems)."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-14:
            raise ValueError("singular system")
        M[c], M[piv] = M[piv], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


class Param:
    """Control points as affine functions of free variables: P_i = a_i + sum_j x_j * B_i[j]."""

    def __init__(self, n):
        self.a = [(0.0, 0.0)] * n
        self.B = [dict() for _ in range(n)]
        self.x0 = []

    def _var(self, v0):
        self.x0.append(v0)
        return len(self.x0) - 1

    def fixed(self, i, p):
        self.a[i] = tuple(p)

    def on_ray(self, i, origin, u, p):
        """Point i slides along origin + x*u; starts where p projects."""
        self.a[i] = tuple(origin)
        self.B[i][self._var(dot(sub(p, origin), u))] = u

    def free(self, i, p):
        self.B[i][self._var(p[0])] = (1.0, 0.0)
        self.B[i][self._var(p[1])] = (0.0, 1.0)

    def points(self, x):
        out = []
        for a, Bi in zip(self.a, self.B):
            q = a
            for j, v in Bi.items():
                q = add(q, mul(v, x[j]))
            out.append(q)
        return out


def amount_to_lambda(amount):
    """Smoothing amount 0..1 -> weight of 'stay near the original' (1e3 .. 1e-3, log scale)."""
    amount = min(max(amount, 0.0), 1.0)
    return 10.0 ** (3.0 - 6.0 * amount)


def energy_matrix(n, p, knots, weights):
    """Sum of trace-normalised Gram matrices, weights = {order: weight}."""
    G = [[0.0] * n for _ in range(n)]
    for order, w in weights.items():
        if w <= 0:
            continue
        Gk = gram_matrix(n, p, knots, order)
        tr = sum(Gk[i][i] for i in range(n)) / n
        if tr <= 0:
            continue
        for i in range(n):
            for j in range(n):
                G[i][j] += w * Gk[i][j] / tr
    return G


def fair(param, orig, p, knots, weights, amount):
    """Minimise  sum_k w_k * E_k(P)  +  lambda(amount) * sum_i |P_i - orig_i|^2  (one linear solve)."""
    n = len(orig)
    nv = len(param.x0)
    if nv == 0:
        return list(orig)
    if isinstance(weights, int):
        weights = {weights: 1.0}
    G = energy_matrix(n, p, knots, weights)
    lam = amount_to_lambda(amount)
    H = [[0.0] * nv for _ in range(nv)]
    rhs = [0.0] * nv
    a, B = param.a, param.B
    for i in range(n):
        for l in range(n):
            gil = G[i][l]
            if gil == 0.0:
                continue
            for j, bij in B[i].items():
                rhs[j] -= gil * dot(bij, a[l])
                for k, blk in B[l].items():
                    H[j][k] += gil * dot(bij, blk)
        for j, bij in B[i].items():
            rhs[j] -= lam * dot(bij, sub(a[i], orig[i]))
            for k, bik in B[i].items():
                H[j][k] += lam * dot(bij, bik)
    return param.points(solve(H, rhs))


def end_runs(ctrl):
    """(start_run, end_run): how many control points from each end lie on that end's line."""
    return collinear_count(ctrl), collinear_count(ctrl, from_end=True)


METHOD_BENDING, METHOD_CURVATURE = "bending", "curvature"


def smooth_param(ctrl, keep_ends=True):
    """How each control point may move. Ends stay put; with keep_ends, the points that lie on
    each end's tangent line slide only along it (so G1/G2/G3 to neighbouring lines is kept),
    and a point on both lines (a corner apex) stays fixed. Returns (param, runs) or (None, msg)."""
    n = len(ctrl)
    cs, ce = end_runs(ctrl) if keep_ends else (1, 1)
    if cs >= n:
        return None, "The curve is straight; nothing to smooth."
    param = Param(n)
    u0 = normalize(sub(ctrl[1], ctrl[0]))
    u1 = normalize(sub(ctrl[-2], ctrl[-1]))
    for i, q in enumerate(ctrl):
        in_s, in_e = i < cs, i >= n - ce
        if i in (0, n - 1) or (in_s and in_e):
            param.fixed(i, q)
        elif in_s:
            param.on_ray(i, ctrl[0], u0, q)
        elif in_e:
            param.on_ray(i, ctrl[-1], u1, q)
        else:
            param.free(i, q)
    return param, (cs, ce, u0, u1)


def runs_in_order(new, ctrl, runs):
    """True if the points sliding along each end line keep their order."""
    cs, ce, u0, u1 = runs
    n = len(ctrl)
    for origin, u, idx in ((ctrl[0], u0, range(cs)), (ctrl[-1], u1, [n - 1 - k for k in range(ce)])):
        xs = [dot(sub(new[i], origin), u) for i in idx]
        if any(b <= a + 1e-12 for a, b in zip(xs, xs[1:])):
            return False
    return True


def fair_spline(ctrl, degree, knots=None, order=None, amount=0.7, keep_ends=True,
                tension=0.0, method=METHOD_BENDING):
    """Smooth a control-point spline. Returns (new_ctrl, warning).

    method BENDING:   minimise bending energy integral |C''|^2, blended with `tension` (0..1) of
                      integral |C'|^2 ("spline under tension": shorter, tighter). One linear solve.
    method CURVATURE: then refine to directly minimise integral (d kappa/ds)^2 ds, the curvature-
                      comb roughness, which is nonlinear (Nelder-Mead; slower).
    order: legacy single-energy override (ENERGY_* constant).
    amount 0..1: how far to go from the original toward the smoothest shape.
    """
    ctrl = [tuple(c) for c in ctrl]
    param, runs = smooth_param(ctrl, keep_ends)
    if param is None:
        return ctrl, runs
    if order is not None:
        weights = {order: 1.0}
    else:
        weights = {ENERGY_ACCEL: 1.0 - tension, ENERGY_VELOCITY: tension}
    new = fair(param, ctrl, degree, knots, weights, amount)
    if method == METHOD_CURVATURE and amount > 0.01:
        new = _curvature_refine(param, ctrl, new, degree, knots, runs)
    warning = ""
    if not runs_in_order(new, ctrl, runs):
        warning = "Points on a line would pass each other; use less smoothing or less tension."
    return new, warning


def _curvature_refine(param, orig, start, p, knots, runs, samples=120, tie=100.0):
    """Polish the bending result: minimise the comb acceleration  integral (d^2k/ds^2)^2 ds
    (relative to the start shape's) while staying close to the start shape. (Minimising the
    comb *slope* integral (dk/ds)^2 instead gives clothoid-like ramps with a sharp knee at
    each end.) Neighbouring control points must stay 2% of the polygon length apart, which
    rules out cusps whose spike would fall between samples."""
    lref = sum(length(sub(b, a)) for a, b in zip(orig, orig[1:]))
    gap = 0.02 * lref

    def roughness(pts):
        v = comb_acceleration(Spline(pts, p, knots), samples)
        return None if v == float("inf") else v

    f0 = roughness(start) or 1.0
    if f0 < 1e-14:
        return start

    def cost(x):
        pts = param.points(x)
        if not runs_in_order(pts, orig, runs) or any(length(sub(b, a)) < gap for a, b in zip(pts, pts[1:])):
            return 1e30
        f = roughness(pts)
        if f is None:
            return 1e30
        dev = sum(dot(sub(a, b), sub(a, b)) for a, b in zip(pts, start))
        return f / f0 + tie * dev / lref ** 2

    x0 = _fit_vars(param, start)
    x, _ = nelder_mead(cost, x0, step=0.02 * lref, iters=40 * len(x0) + 80, tol=1e-12)
    return param.points(x)


def _fit_vars(param, pts):
    """Variables x with param.points(x) ~ pts (each variable belongs to one point)."""
    x = list(param.x0)
    for i, Bi in enumerate(param.B):
        if not Bi:
            continue
        r = sub(pts[i], param.a[i])
        for j, v in Bi.items():
            x[j] = dot(r, v) / dot(v, v)
    return x


def fairness(spline, samples=400):
    """integral (d kappa / ds)^2 ds: lower = curvature changes more gently."""
    tot = 0.0
    for i in range(samples):
        t = (i + 0.5) / samples
        tot += spline.curvature_rate(t) ** 2 * length(spline.d(t, 1)) / samples
    return tot


def comb_acceleration(spline, samples=200):
    """integral (d^2 kappa / ds^2)^2 ds: how abruptly the comb's slope changes. Minimising
    integral (dk/ds)^2 alone gives clothoid-like curves whose comb ramps up linearly from the
    ends (a sharp knee next to each join); penalising this instead makes the slope itself
    ease in, which is what G3 continuity is for."""
    ts = [i / samples for i in range(samples + 1)]
    pts = [spline.point(t) for t in ts]
    rates = [spline.curvature_rate(t) for t in ts]
    tot = 0.0
    for i in range(samples):
        ds = length(sub(pts[i + 1], pts[i]))
        if ds < EPS:
            return float("inf")
        tot += ((rates[i + 1] - rates[i]) / ds) ** 2 * ds
    return tot


def max_curvature(spline, samples=600):
    return max(abs(spline.curvature(i / samples)) for i in range(samples + 1))


# --------------------------------------------------------------------------- G3 fillet
FILLET_DEPTH, FILLET_TANGENT, FILLET_MINRADIUS = "depth", "tangent", "minradius"


def max_curvature_fast(spline, coarse=80):
    """Max |curvature|: coarse scan, then golden-section refinement around the peak."""
    ts = [i / coarse for i in range(coarse + 1)]
    ks = [abs(spline.curvature(t)) for t in ts]
    i = max(range(len(ks)), key=ks.__getitem__)
    a, b = ts[max(i - 1, 0)], ts[min(i + 1, coarse)]
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(25):
        c, d = b - gr * (b - a), a + gr * (b - a)
        if abs(spline.curvature(c)) > abs(spline.curvature(d)):
            b = d
        else:
            a = c
    return max(ks[i], abs(spline.curvature((a + b) / 2)))


def nelder_mead(f, x0, step=0.5, iters=120, tol=1e-7):
    n = len(x0)
    pts = [list(x0)] + [[x0[j] + (step if j == i else 0.0) for j in range(n)] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=vals.__getitem__)
        pts, vals = [pts[i] for i in order], [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        cen = [sum(p[j] for p in pts[:-1]) / n for j in range(n)]
        refl = [cen[j] + (cen[j] - pts[-1][j]) for j in range(n)]
        fr_ = f(refl)
        if fr_ < vals[0]:
            exp = [cen[j] + 2 * (cen[j] - pts[-1][j]) for j in range(n)]
            fe = f(exp)
            pts[-1], vals[-1] = (exp, fe) if fe < fr_ else (refl, fr_)
        elif fr_ < vals[-2]:
            pts[-1], vals[-1] = refl, fr_
        else:
            con = [cen[j] + 0.5 * (pts[-1][j] - cen[j]) for j in range(n)]
            fc = f(con)
            if fc < vals[-1]:
                pts[-1], vals[-1] = con, fc
            else:
                for i in range(1, n + 1):
                    pts[i] = [pts[0][j] + 0.5 * (pts[i][j] - pts[0][j]) for j in range(n)]
                    vals[i] = f(pts[i])
    i = min(range(n + 1), key=vals.__getitem__)
    return pts[i], vals[i]


_fillet_cache = {}


def fillet_profile_distance(p, phi):
    """Distance from p to the radius-1 fillet profile (line A - arc - line B) of a corner at the
    origin with legs along (1, 0) and (cos phi, sin phi)."""
    half = phi / 2
    tdist = 1.0 / math.tan(half)                     # corner -> tangent points
    bis = (math.cos(half), math.sin(half))
    centre = mul(bis, 1.0 / math.sin(half))
    uA, uB = (1.0, 0.0), (math.cos(phi), math.sin(phi))
    best = float("inf")
    for u in (uA, uB):                                # the kept part of each line
        t = max(dot(p, u), tdist)
        best = min(best, length(sub(p, mul(u, t))))
    # arc: between the two tangent points, on the side facing the corner
    v = sub(p, centre)
    ta, tb = sub(mul(uA, tdist), centre), sub(mul(uB, tdist), centre)
    if cross(ta, v) * cross(ta, tb) >= 0 and cross(tb, v) * cross(tb, ta) >= 0:
        best = min(best, abs(length(v) - 1.0))
    else:
        best = min(best, length(sub(p, mul(uA, tdist))), length(sub(p, mul(uB, tdist))))
    return best


def fillet_shape(phi, count=7):
    """Best-fit G3/G4 stand-in for a radius-1 fillet of a corner with legs at angle phi
    (0 < phi < pi), corner at the origin.

    Leg length and inner-point spacing are chosen together to minimise the largest distance
    between the curve and the true fillet profile (line, arc, line). A G3 curve can't follow
    the arc exactly: it has to ease in from zero curvature, so it starts further out along
    the lines and bends a little tighter than the arc in the middle.
    Starts from the acceleration-faired layout, then refines with Nelder-Mead.
    Returns (fractions, L_fit, spline_at_L1, peak_curvature_at_L1, depth_at_L1)."""
    key = (round(phi, 6), count)
    if key in _fillet_cache:
        return _fillet_cache[key]
    uA, uB = (1.0, 0.0), (math.cos(phi), math.sin(phi))
    m, apex_mode, n, deg = corner_style(count)

    def spline_for(fr):
        return Spline(corner_control_points((0.0, 0.0), uA, uB, fr, fr, apex_mode), deg)

    fr0 = tension_fractions(m, 0.5)
    faired, warn = fair_spline(spline_for(fr0).ctrl, deg, order=ENERGY_ACCEL, amount=1.0)
    start = [length(q) for q in faired[:m]]
    if warn or not check_distances(start):
        start = fr0

    # descending fractions via bounded ratios: f_j = f_{j-1} * r_j, r_j in (RMIN, RMAX),
    # which keeps neighbouring control points from collapsing onto each other.
    RMIN, RMAX = 0.25, 0.95

    def to_fr(z):
        fr = [1.0]
        for zj in z:
            fr.append(fr[-1] * (RMIN + (RMAX - RMIN) / (1.0 + math.exp(-zj))))
        return fr

    def to_z(r):
        q = min(max((r - RMIN) / (RMAX - RMIN), 0.02), 0.98)
        return math.log(q / (1 - q))

    # initial leg length: match the arc's depth into the corner
    arc_depth = 1.0 / math.sin(phi / 2) - 1.0
    L0 = arc_depth / length(spline_for(start).point(0.5))
    z0 = [math.log(L0)] + [to_z(start[j] / start[j - 1]) for j in range(1, m)]
    ts = [(i + 0.5) / 120 for i in range(120)]

    def cost(z):
        L = math.exp(z[0])
        s = spline_for([L * f for f in to_fr(z[1:])])
        return max(fillet_profile_distance(s.point(t), phi) for t in ts)

    z, _ = nelder_mead(cost, z0, step=0.3, iters=200, tol=1e-10)
    fr, L_fit = to_fr(z[1:]), math.exp(z[0])
    s = spline_for(fr)
    out = (fr, L_fit, s, max_curvature_fast(s), length(s.point(0.5)))
    _fillet_cache[key] = out
    return out


FILLET_BEST = "best"


_fillet_smooth_cache = {}


def fillet_smoothest(phi, count):
    """Smoothest corner curve for this angle: the inner-point spacing that minimises the comb
    roughness  integral (dk/ds)^2 ds  (GX Smooth's 'curvature flow' at full strength). Roughness
    times size^3 doesn't depend on size, so the shape is found once per (angle, count); the
    leg length that brings this shape closest to a radius-1 fillet is found after.
    Returns (fractions, L_fit, spline_at_L1, peak_curvature_at_L1, depth_at_L1)."""
    key = (round(phi, 6), count)
    if key in _fillet_smooth_cache:
        return _fillet_smooth_cache[key]
    uA, uB = (1.0, 0.0), (math.cos(phi), math.sin(phi))
    m, apex_mode, n, deg = corner_style(count)
    RMIN, RMAX = 0.1, 0.98
    ts = [(i + 0.5) / 160 for i in range(160)]

    def to_fr(z):
        fr = [1.0]
        for zj in z:
            fr.append(fr[-1] * (RMIN + (RMAX - RMIN) / (1.0 + math.exp(-zj))))
        return fr

    def to_z(r):
        q = min(max((r - RMIN) / (RMAX - RMIN), 1e-4), 1 - 1e-4)
        return math.log(q / (1 - q))

    def spline_for(fr):
        return Spline(corner_control_points((0.0, 0.0), uA, uB, fr, fr, apex_mode), deg)

    def rough(z):
        v = comb_acceleration(spline_for(to_fr(z)), 160)
        return v if v < float("inf") else 1e30

    best = None
    starts = [fillet_shape(phi, count)[0], tension_fractions(m, 0.5), tension_fractions(m, 0.3)]
    for fr0 in starts:
        z0 = [to_z(fr0[j] / fr0[j - 1]) for j in range(1, m)]
        z, v = nelder_mead(rough, z0, step=0.4, iters=200, tol=1e-12)
        if best is None or v < best[1]:
            best = (z, v)
    fr = to_fr(best[0])
    s = spline_for(fr)

    # size: the leg length that brings this shape closest to the radius-1 fillet (golden section)
    def gap(L):
        sl = Spline([mul(q, L) for q in s.ctrl], deg)
        return max(fillet_profile_distance(sl.point(t), phi) for t in ts)

    arc_tangent = 1.0 / math.tan(phi / 2)
    a, b = 0.5 * arc_tangent, 6.0 * arc_tangent
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(40):
        c, d = b - gr * (b - a), a + gr * (b - a)
        if gap(c) < gap(d):
            b = d
        else:
            a = c
    out = (fr, (a + b) / 2, s, max_curvature_fast(s), length(s.point(0.5)))
    _fillet_smooth_cache[key] = out
    return out


def fillet_layout(phi, R, count=7, match=FILLET_BEST, emulate=False):
    """Leg distances for a G3 (count 7) / G4 (count 9) curve that stands in for a radius-R fillet.

    Shape (spacing of the leg points): the smoothest comb by default (fillet_smoothest), or with
    emulate=True the closest match to the arc (fillet_shape). match picks its size:
      FILLET_BEST      -- this shape's closest fit to the true fillet profile (smallest largest gap)
      FILLET_DEPTH     -- passes through the arc's midpoint (same depth into the corner)
      FILLET_TANGENT   -- starts where the arc's tangent points are
      FILLET_MINRADIUS -- never bends tighter than R (tightest radius = R)
    Returns dict with dist (outer -> inner), L, min_radius, depth, max_gap (largest distance
    from the true fillet), and the arc's own depth / tangent distance.
    """
    if not (1e-6 < phi < math.pi - 1e-6):
        raise ValueError("The lines must form a corner (not parallel).")
    shape = fillet_shape if emulate else fillet_smoothest
    fr, L_fit, s1, k1, depth1 = shape(phi, count)
    arc_depth = R / math.sin(phi / 2) - R
    arc_tangent = R / math.tan(phi / 2)
    if match == FILLET_DEPTH:
        L = arc_depth / depth1
    elif match == FILLET_TANGENT:
        L = arc_tangent
    elif match == FILLET_MINRADIUS:
        L = k1 * R
    else:
        L = L_fit * R
    # largest gap to the true fillet: evaluate at R = 1 and scale
    s = Spline([mul(q, L / R) for q in s1.ctrl], s1.p)
    gap = max(fillet_profile_distance(s.point((i + 0.5) / 300), phi) for i in range(300)) * R
    return dict(L=L, dist=[L * f for f in fr], fractions=fr, min_radius=L / k1, depth=L * depth1,
                max_gap=gap, arc_depth=arc_depth, arc_tangent=arc_tangent,
                roughness=fairness(s),           # integral (dk/ds)^2 ds at R = 1
                comb_accel=comb_acceleration(s))  # integral (d2k/ds2)^2 ds at R = 1


# --------------------------------------------------------------------------- blends into curves (conics)
# For G3 contact with a curve, the blend must match the curve's position, unit tangent T,
# signed curvature k and dk/ds at the contact point. Writing the blend's start in terms of
# a local reparametrisation s(t) with s' = alpha > 0, s'' = beta, s''' = gamma:
#     C'(0)   = alpha T
#     C''(0)  = beta T + alpha^2 k N
#     C'''(0) = (gamma - alpha^3 k^2) T + (3 alpha beta k + alpha^3 k') N
# A clamped B-spline's k-th end derivative depends only on P0..Pk (a triangular linear map),
# so P1..P3 follow directly; alpha, beta, gamma are free shape parameters. With k = k' = 0
# this is exactly "the first four control points on the line".

class Curve2D:
    """Parametric curve: fn(u) -> (r, r', r'', r''') as 2D tuples, on [u_min, u_max]."""

    def __init__(self, fn, u_min=-1e9, u_max=1e9, periodic=None):
        self.fn, self.u_min, self.u_max, self.periodic = fn, u_min, u_max, periodic

    def wrap(self, u):
        if self.periodic:
            return self.u_min + (u - self.u_min) % self.periodic
        return u

    def inside(self, u, tol=1e-9):
        return bool(self.periodic) or (self.u_min - tol <= u <= self.u_max + tol)

    def point(self, u):
        return self.fn(self.wrap(u))[0]

    def frame(self, u, sign):
        """Local data walking in direction sign (+1 = increasing u)."""
        r, r1, r2, r3 = self.fn(self.wrap(u))
        sp = length(r1)
        T = mul(r1, sign / sp)
        N = (-T[1], T[0])
        k = sign * cross(r1, r2) / sp ** 3
        s2 = dot(r1, r1)
        dk = (cross(r1, r3) * s2 - 3.0 * cross(r1, r2) * dot(r1, r2)) / (s2 ** 2.5) / sp
        return dict(P=r, T=T, N=N, k=k, dk=dk, u=u, sign=sign)

    def arc_param(self, u, sign, s, iters=30):
        """Parameter reached after walking arc length s (may be negative) in direction sign."""
        def arclen(a, b, n=24):  # Simpson
            if a == b:
                return 0.0
            h = (b - a) / n
            tot = 0.0
            for i in range(n + 1):
                w = 1 if i in (0, n) else (4 if i % 2 else 2)
                tot += w * length(self.fn(self.wrap(a + i * h))[1])
            return tot * h / 3
        target = s
        v = u + sign * s / max(length(self.fn(self.wrap(u))[1]), EPS)
        for _ in range(iters):
            cur = sign * arclen(u, v)
            err = target - cur
            if abs(err) < 1e-12 * max(1.0, abs(s)):
                break
            v += sign * err / max(length(self.fn(self.wrap(v))[1]), EPS)
        return v

    def closest(self, p, u0, iters=50):
        u = u0
        for _ in range(iters):
            r, r1, r2, _ = self.fn(self.wrap(u))
            d = sub(r, p)
            f, df = dot(d, r1), dot(r1, r1) + dot(d, r2)
            if abs(df) < EPS:
                break
            step = f / df
            u -= step
            if abs(step) < 1e-14:
                break
        return u


def line_curve(p, d):
    return Curve2D(lambda u: (add(p, mul(d, u)), tuple(d), (0.0, 0.0), (0.0, 0.0)))


def _rot(v, c, s):
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def ellipse_curve(centre, a, b, angle=0.0):
    c, s = math.cos(angle), math.sin(angle)

    def fn(u):
        cu, su = math.cos(u), math.sin(u)
        return (add(centre, _rot((a * cu, b * su), c, s)), _rot((-a * su, b * cu), c, s),
                _rot((-a * cu, -b * su), c, s), _rot((a * su, -b * cu), c, s))
    return Curve2D(fn, 0.0, 2 * math.pi, periodic=2 * math.pi)


def parabola_curve(vertex, f, angle=0.0):
    """x = u, y = u^2 / (4f) in the local frame (opens toward +y)."""
    c, s = math.cos(angle), math.sin(angle)

    def fn(u):
        return (add(vertex, _rot((u, u * u / (4 * f)), c, s)), _rot((1.0, u / (2 * f)), c, s),
                _rot((0.0, 1.0 / (2 * f)), c, s), (0.0, 0.0))
    return Curve2D(fn)


def hyperbola_curve(centre, a, b, angle=0.0):
    """Right branch: (a cosh u, b sinh u)."""
    c, s = math.cos(angle), math.sin(angle)

    def fn(u):
        ch, sh = math.cosh(u), math.sinh(u)
        return (add(centre, _rot((a * ch, b * sh), c, s)), _rot((a * sh, b * ch), c, s),
                _rot((a * ch, b * sh), c, s), _rot((a * sh, b * ch), c, s))
    return Curve2D(fn, -20.0, 20.0)


_end_matrix_cache = {}


def end_matrix(n, p):
    """M[k][j] = N_j^(k)(0) for k, j = 0..3 (uniform clamped knots; by symmetry also the t=1 end
    in the reversed parameter)."""
    key = (n, p)
    if key not in _end_matrix_cache:
        basis = [Spline([(1.0 if j == i else 0.0, 0.0) for j in range(n)], p) for i in range(min(n, 4))]
        _end_matrix_cache[key] = [[basis[j].d(0.0, k)[0] if j < len(basis) else 0.0 for j in range(4)]
                                  for k in range(4)]
    return _end_matrix_cache[key]


def end_derivs(F, alpha, beta, gamma):
    T, N, k, dk = F["T"], F["N"], F["k"], F["dk"]
    D1 = mul(T, alpha)
    D2 = add(mul(T, beta), mul(N, alpha * alpha * k))
    D3 = add(mul(T, gamma - alpha ** 3 * k * k), mul(N, 3 * alpha * beta * k + alpha ** 3 * dk))
    return [F["P"], D1, D2, D3]


def end_points(F, alpha, beta, gamma, M, order):
    """Control points P0..P_order giving G(order) contact with the curve frame F."""
    D = end_derivs(F, alpha, beta, gamma)
    P = []
    for k in range(order + 1):
        acc = D[k]
        for j in range(k):
            acc = sub(acc, mul(P[j], M[k][j]))
        P.append(mul(acc, 1.0 / M[k][k]))
    return P


def end_shape_from_points(P, M):
    """alpha, beta, gamma of an end whose first control points are P (inverse of end_points, for
    straight-leg starting guesses)."""
    out = []
    for k in (1, 2, 3):
        Dk = (0.0, 0.0)
        for j in range(min(k + 1, len(P))):
            Dk = add(Dk, mul(P[j], M[k][j]))
        out.append(Dk)
    return out  # D1, D2, D3


BLEND_STYLES = {
    # count -> (n control points, degree, per-side order, shared middle point, free apex)
    (5, "cubic"): (5, 3, 2, True, False),
    (5, "quintic"): (6, 5, 2, False, False),
    (7, "cubic"): (7, 5, 3, True, False),
    (9, "cubic"): (9, 5, 3, False, True),
}


def blend_ctrl(FA, FB, count, five_type, shape):
    """Control points for a blend from frame FA to frame FB (each T pointing into the blend).
    shape: dict alphaA, betaA, gammaA, alphaB, betaB, gammaB, apex. Shared middle points are
    solved for (using the last free parameter of each side). Returns ctrl or None."""
    n, p, order, shared, free_apex = BLEND_STYLES[(count, five_type if count == 5 else "cubic")]
    M = end_matrix(n, p)
    a = end_points(FA, shape["alphaA"], shape["betaA"], shape.get("gammaA", 0.0), M, order)
    b = end_points(FB, shape["alphaB"], shape["betaB"], shape.get("gammaB", 0.0), M, order)
    if shared:
        # last point of each side moves along its T as the last free parameter changes:
        # a[-1] + x * TA / Mkk  ==  b[-1] + y * TB / Mkk
        Mkk = M[order][order]
        ta, tb = mul(FA["T"], 1.0 / Mkk), mul(FB["T"], 1.0 / Mkk)
        det = cross(ta, mul(tb, -1.0))
        if abs(det) < 1e-12:
            return None
        w = sub(b[-1], a[-1])
        x = cross(w, mul(tb, -1.0)) / det
        mid = add(a[-1], mul(ta, x))
        return a[:-1] + [mid] + list(reversed(b[:-1]))
    mid = [shape["apex"]] if free_apex else []
    return a + mid + list(reversed(b))


def _profile_polyline(curve, u0, u1, n=60):
    return [curve.point(u0 + (u1 - u0) * i / n) for i in range(n + 1)]


def _dist_to_polyline(p, pl):
    best = float("inf")
    for a, b in zip(pl, pl[1:]):
        ab = sub(b, a)
        L2 = dot(ab, ab)
        t = 0.0 if L2 < EPS else max(0.0, min(1.0, dot(sub(p, a), ab) / L2))
        best = min(best, length(sub(p, add(a, mul(ab, t)))))
    return best


def _fillet_profile(A, ua, B, ub, arc, n=24):
    """Distance function to the true fillet profile: curve A from the blend's contact to the arc,
    the arc itself (exact), then curve B."""
    c, R = arc["centre"], arc["R"]
    ta, tb = sub(A.point(arc["ua"]), c), sub(B.point(arc["ub"]), c)
    sweep_sign = cross(ta, tb)
    pa = _profile_polyline(A, ua, arc["ua"], n)
    pb = _profile_polyline(B, arc["ub"], ub, n)

    def dist(p):
        v = sub(p, c)
        best = min(_dist_to_polyline(p, pa), _dist_to_polyline(p, pb))
        if cross(ta, v) * sweep_sign >= 0 and cross(v, tb) * sweep_sign >= 0:  # within the arc's sector
            best = min(best, abs(length(v) - R))
        return best
    return dist


def find_intersection(A, ua, B, ub, iters=60):
    """Newton for A(u) = B(v) from (ua, ub). Returns (u, v) or None."""
    u, v = ua, ub
    for _ in range(iters):
        ra, da, _, _ = A.fn(A.wrap(u))
        rb, db, _, _ = B.fn(B.wrap(v))
        f = sub(ra, rb)
        det = cross(da, mul(db, -1.0))
        if abs(det) < EPS:
            return None
        du = cross(mul(f, -1.0), mul(db, -1.0)) / det
        dv = cross(da, mul(f, -1.0)) / det
        u, v = u + du, v + dv
        if abs(du) + abs(dv) < 1e-13:
            break
    if length(sub(A.point(u), B.point(v))) > 1e-7:
        return None
    return u, v


def circle_fillet(A, uX, sA, B, vX, sB, R):
    """Radius-R arc tangent to the kept parts of A and B near their crossing (u = uX, v = vX).
    sA / sB: direction along each curve's parameter that goes *away* from the crossing into the
    kept part. Returns dict ua, ub (tangent points), centre, phi (equivalent corner angle) or None."""
    a = A.frame(uX, sA)["T"]
    b = B.frame(vX, sB)["T"]
    phi = math.acos(max(-1.0, min(1.0, dot(a, b))))
    if not (1e-4 < phi < math.pi - 1e-4):
        return None
    # normal side: the side of A where B's kept part lies (and vice versa), fixed in parameter terms
    def nrm(curve, u):
        r1 = curve.fn(curve.wrap(u))[1]
        sp = length(r1)
        return (-r1[1] / sp, r1[0] / sp)
    oa = 1.0 if dot(nrm(A, uX), b) > 0 else -1.0
    ob = 1.0 if dot(nrm(B, vX), a) > 0 else -1.0
    tdist = R / math.tan(phi / 2)
    u = A.arc_param(uX, sA, tdist)
    v = B.arc_param(vX, sB, tdist)

    def F(u, v):
        return sub(add(A.point(u), mul(nrm(A, u), oa * R)), add(B.point(v), mul(nrm(B, v), ob * R)))

    for _ in range(60):
        f = F(u, v)
        if length(f) < 1e-12 * max(1.0, R):
            break
        h = 1e-7
        fu = mul(sub(F(u + h, v), f), 1 / h)
        fv = mul(sub(F(u, v + h), f), 1 / h)
        det = cross(fu, fv)
        if abs(det) < EPS:
            return None
        du = cross(mul(f, -1.0), fv) / det
        dv = cross(fu, mul(f, -1.0)) / det
        u, v = u + du, v + dv
    if length(F(u, v)) > 1e-7 * max(1.0, R):
        return None
    if sA * (u - uX) <= 0 or sB * (v - vX) <= 0 or not (A.inside(u) and B.inside(v)):
        return None
    centre = add(A.point(u), mul(nrm(A, u), oa * R))
    na, nb = sub(A.point(u), centre), sub(B.point(v), centre)
    sweep = math.acos(max(-1.0, min(1.0, dot(na, nb) / (R * R))))
    return dict(ua=u, ub=v, centre=centre, phi=math.pi - sweep, R=R)


def curve_fillet(A, uX, sA, B, vX, sB, R, count=7, emulate=False, match=FILLET_BEST, five_type="cubic",
                 quick=False, layout=None):
    """G3 (G2 for 5 points) blend standing in for a radius-R fillet between curves A and B
    (lines, conics, any smooth curve). Contact points sit beyond the arc's tangent points by the
    same proportion the straight-line fillet uses for this angle; the shape is then optimised
    (smoothest comb, or closest to the arc with emulate). Returns dict ctrl, degree, ua, ub (contact
    parameters), arc (circle_fillet result), max_gap, min_radius; raises ValueError.
    quick: skip the optimisation and use the straight-line layout carried over to the curves (still
    G3; max_gap and min_radius are then None). layout(phi): optional stand-in for fillet_layout at
    R = 1, giving dict L, arc_tangent, fractions."""
    arc = circle_fillet(A, uX, sA, B, vX, sB, R)
    if arc is None:
        raise ValueError("No radius-%.4g fillet fits between these curves here." % R)
    phi = arc["phi"]
    line_count = 7 if count == 5 else count
    lay = layout(phi) if layout else fillet_layout(phi, 1.0, line_count, match, emulate)
    ext = (lay["L"] / lay["arc_tangent"] - 1.0) * R / math.tan(phi / 2)
    ua = A.arc_param(arc["ua"], sA, ext)
    ub = B.arc_param(arc["ub"], sB, ext)
    if not (A.inside(ua) and B.inside(ub)):
        raise ValueError("The blend would run past the end of a curve; use a smaller radius.")
    FA, FB = A.frame(ua, -sA), B.frame(ub, -sB)  # T points from the kept part into the blend

    n, p, order, shared, free_apex = BLEND_STYLES[(count, five_type if count == 5 else "cubic")]
    M = end_matrix(n, p)
    # starting guess: the straight-line layout on the tangent lines at the contacts
    hit = line_intersection(FA["P"], FA["T"], FB["P"], FB["T"])
    if hit is None or hit[1] <= 0 or hit[2] <= 0:
        raise ValueError("The curves turn too far between the contact points for this blend.")
    Xv = hit[0]
    LA, LB = length(sub(FA["P"], Xv)), length(sub(FB["P"], Xv))
    fr = lay["fractions"]
    m = len(fr)
    lineA = [add(Xv, mul(FA["T"], -LA * f)) for f in fr] + [Xv]
    lineB = [add(Xv, mul(FB["T"], -LB * f)) for f in fr] + [Xv]
    DA, DB = end_shape_from_points(lineA, M), end_shape_from_points(lineB, M)
    x0 = [math.log(max(dot(DA[0], FA["T"]), 1e-9)), dot(DA[1], FA["T"]),
          math.log(max(dot(DB[0], FB["T"]), 1e-9)), dot(DB[1], FB["T"])]
    gA, gB = dot(DA[2], FA["T"]), dot(DB[2], FB["T"])
    if free_apex:
        x0 += [Xv[0], Xv[1]]

    def shape_of(x):
        sh = dict(alphaA=math.exp(x[0]), betaA=x[1], gammaA=gA, alphaB=math.exp(x[2]), betaB=x[3], gammaB=gB)
        if free_apex:
            sh["apex"] = (x[4], x[5])
        return sh

    if quick:
        ctrl = blend_ctrl(FA, FB, count, five_type, shape_of(x0))
        if ctrl is None:
            raise ValueError("No blend fits between these curves here.")
        return dict(ctrl=ctrl, degree=p, ua=ua, ub=ub, arc=arc, FA=FA, FB=FB, max_gap=None, min_radius=None, G=order)

    chord = length(sub(FA["P"], FB["P"]))
    gap_to_profile = _fillet_profile(A, ua, B, ub, arc)
    ts = [(i + 0.5) / 60 for i in range(60)]
    min_gap = 0.005 * chord  # neighbouring control points must stay this far apart (no cusps)

    def roughness(s):
        if p == 3:  # a cubic's curvature slope jumps at its inner knot by construction; use the slope
            return fairness(s, 80) * chord ** 3
        v = comb_acceleration(s, 80)
        return v * chord ** 5 if v < float("inf") else 1e30

    def make(x):
        ctrl = blend_ctrl(FA, FB, count, five_type, shape_of(x))
        if ctrl is None or any(length(sub(q2, q1)) < min_gap for q1, q2 in zip(ctrl, ctrl[1:])):
            return None
        return Spline(ctrl, p)

    s0 = make(x0)
    rough0 = max(roughness(s0), 1e-9) if s0 else 1.0

    def cost(x):
        s = make(x)
        if s is None:
            return 1e30
        gap = max(gap_to_profile(s.point(t)) for t in ts) / R
        if emulate:
            # closest to the arc, with a small roughness penalty so it can't buy a few % of fit
            # with a spike in the comb
            return gap + 1e-2 * roughness(s) / rough0
        # smoothest, with a gentle pull toward the arc (a 10%-of-R gap costs as much as the
        # starting roughness) so a free layout can't bulge away from the fillet's size
        return roughness(s) / rough0 + (gap / 0.1) ** 2

    if free_apex:  # shape first with the apex held, then everything
        k = len(x0) - 2
        xs, _ = nelder_mead(lambda y: cost(list(y) + x0[k:]), x0[:k], step=0.2, iters=40 * k, tol=1e-12)
        x0 = list(xs) + x0[k:]
    x, best = nelder_mead(cost, x0, step=0.2, iters=40 * len(x0), tol=1e-12)
    for step in (0.1, 0.05):  # restarts: Nelder-Mead stalls easily in 4-6 dimensions
        x2, v2 = nelder_mead(cost, x, step=step, iters=25 * len(x0), tol=1e-12)
        if v2 < best:
            x, best = x2, v2
    ctrl = blend_ctrl(FA, FB, count, five_type, shape_of(x))
    s = Spline(ctrl, p)
    gap = max(gap_to_profile(s.point((i + 0.5) / 300)) for i in range(300))
    return dict(ctrl=ctrl, degree=p, ua=ua, ub=ub, arc=arc, FA=FA, FB=FB, max_gap=gap,
                min_radius=1.0 / max(max_curvature(s, 600), EPS), G=order)


def conic_segment(curve, u0, u1):
    """Exact Fusion-style conic for the piece of `curve` between u0 and u1 (exact if the curve is
    a conic): ('line', p0, p1) or ('conic', p0, p1, apex, rho). A conic segment is a rational
    quadratic with apex at the end tangents' crossing; rho = |S - M| / |apex - M| where M is the
    chord midpoint and S the curve's point on the line M -> apex (the "shoulder")."""
    p0, p1 = curve.point(u0), curve.point(u1)
    t0, t1 = curve.frame(u0, +1)["T"], curve.frame(u1, +1)["T"]
    chord = sub(p1, p0)
    if length(chord) < EPS:
        raise ValueError("zero-length segment")
    if abs(cross(t0, normalize(chord))) < 1e-9 and abs(cross(t1, normalize(chord))) < 1e-9:
        return ("line", p0, p1)
    hit = line_intersection(p0, t0, p1, t1)
    if hit is None:
        raise ValueError("segment turns 180 degrees or more")
    apex = hit[0]
    M = mul(add(p0, p1), 0.5)
    w = sub(apex, M)

    def side(u):
        return cross(sub(curve.point(u), M), w)
    a, b = u0, u1
    fa = side(a)
    for _ in range(80):  # bisection: the shoulder is where the curve crosses the line M -> apex
        mid = 0.5 * (a + b)
        fm = side(mid)
        if (fm > 0) == (fa > 0):
            a, fa = mid, fm
        else:
            b = mid
    S = curve.point(0.5 * (a + b))
    rho = length(sub(S, M)) / length(w)
    if not (0.0 < rho < 1.0):
        raise ValueError("segment isn't a simple conic arc")
    return ("conic", p0, p1, apex, rho)
