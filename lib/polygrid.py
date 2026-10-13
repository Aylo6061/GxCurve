"""Polygrid geometry (pure Python, no Fusion).

A polygrid is a lattice of polygonal pockets (triangles, squares, hexagons or rectangles)
separated by ribs of thickness `rib`, filling an area. The pipeline:

  1. erode(): shrink the area by `border`, the rib that runs along the area's edge and
     around its holes. Every boundary piece is offset inwards, the offsets are split where
     they cross, the parts at least `border` from the boundary are kept and joined up again.
  2. lattice(): the pocket polygons, placed so the pattern is mirror-symmetric about the
     grid's two axes (through cell centres or along ribs).
  3. Each pocket is clipped to the shrunk area. Pockets that cross its edge are cut to it,
     and cut pockets smaller than a fraction of a full one are dropped.
  4. Each resulting loop is turned back into lines, arcs and curves (one per run of the same
     source curve), with corners recomputed exactly where lines and arcs meet.

Coordinates are 2D tuples in sketch space (cm). An area is a list of loops (outer first is
not required); a loop is a list of primitives in any order and direction:
  ("line", p0, p1)
  ("arc", centre, r, a0, sweep)   starts at angle a0; sweep is signed (CCW > 0), ±2π = circle
  ("curve", [p0, p1, ...])         anything else (splines, ellipses), as a dense polyline
After prepare_area() the outer loop runs CCW and holes CW, so the area is always on the left.

Results are drawing primitives:
  ("seg", a, b, is_cell_edge)
  ("arc3", start, mid, end, centre, sweep)
  ("circle", centre, r)
  ("spline", points, closed)        fitted through the points
  ("cspline", control_points, degree)   a G3 / G4 corner
"""

import math

try:
    from . import gx_math as g
except ImportError:  # imported as a plain module (tests)
    import gx_math as g

TAU = 2.0 * math.pi
SQ3 = math.sqrt(3.0)

TRIANGLE, SQUARE, HEXAGON, RECTANGLE = "triangle", "square", "hexagon", "rectangle"
SHAPES = (TRIANGLE, SQUARE, HEXAGON, RECTANGLE)
CELL, RIB = "cell", "rib"          # what a symmetry axis runs through
SIDE, INSCRIBED = "side", "inscribed"

NODE_TOL = 1e-9    # cm: points closer than this are the same point
MIN_EDGE = 1e-5    # cm: shorter output edges are merged away
SAG = 1e-4         # cm: chord tolerance when arcs become polylines
MAX_CELLS = 20000


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


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(a):
    n = math.hypot(a[0], a[1])
    if n == 0:
        raise ValueError("zero-length vector")
    return (a[0] / n, a[1] / n)


def perp(a):
    """Left normal."""
    return (-a[1], a[0])


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def polar(c, r, ang):
    return (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))


def angle_of(v):
    return math.atan2(v[1], v[0])


def turn(a, b):
    """Signed angle from direction a to direction b."""
    return math.atan2(cross(a, b), dot(a, b))


def poly_area(pts):
    """Signed area (CCW > 0)."""
    s = 0.0
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        s += a[0] * b[1] - b[0] * a[1]
    return 0.5 * s


def bbox(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def point_in_poly(p, pts):
    inside = False
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        if (a[1] > p[1]) != (b[1] > p[1]):
            x = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x > p[0]:
                inside = not inside
    return inside


def seg_dist(q, a, b):
    ab = sub(b, a)
    l2 = dot(ab, ab)
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, dot(sub(q, a), ab) / l2))
    return dist(q, lerp(a, b, t))


# --------------------------------------------------------------------------- primitives
def prim_start(p):
    if p[0] == "line":
        return p[1]
    if p[0] == "arc":
        return polar(p[1], p[2], p[3])
    return p[1][0]


def prim_end(p):
    if p[0] == "line":
        return p[2]
    if p[0] == "arc":
        return polar(p[1], p[2], p[3] + p[4])
    return p[1][-1]


def prim_reverse(p):
    if p[0] == "line":
        return ("line", p[2], p[1])
    if p[0] == "arc":
        return ("arc", p[1], p[2], p[3] + p[4], -p[4])
    return ("curve", list(reversed(p[1])))


def _arc_tangent(a, sweep):
    s = 1.0 if sweep > 0 else -1.0
    return (-math.sin(a) * s, math.cos(a) * s)


def prim_tangents(p):
    """Unit tangents at the start and at the end."""
    if p[0] == "line":
        d = unit(sub(p[2], p[1]))
        return d, d
    if p[0] == "arc":
        return _arc_tangent(p[3], p[4]), _arc_tangent(p[3] + p[4], p[4])
    pts = p[1]
    return unit(sub(pts[1], pts[0])), unit(sub(pts[-1], pts[-2]))


def arc_steps(r, sweep, sag=SAG):
    """Chords needed for an arc to stay within sag of the true curve."""
    da = math.pi / 8
    if r > sag:
        da = min(da, 2.0 * math.acos(1.0 - sag / r))
    return max(1, min(4000, int(math.ceil(abs(sweep) / da - 1e-9))))


def prim_points(p, sag=SAG):
    """Polyline from start to end (both included)."""
    if p[0] == "line":
        return [p[1], p[2]]
    if p[0] == "arc":
        c, r, a0, sw = p[1], p[2], p[3], p[4]
        n = arc_steps(r, sw, sag)
        return [polar(c, r, a0 + sw * i / n) for i in range(n + 1)]
    return list(p[1])


def _in_sweep(ang, a0, sw):
    if abs(sw) >= TAU - 1e-12:
        return True
    f = (ang - a0) % TAU if sw > 0 else (a0 - ang) % TAU
    return f <= abs(sw)


def _arc_dist(q, c, r, a0, sw):
    v = sub(q, c)
    d = length(v)
    if d == 0:
        return r
    if _in_sweep(angle_of(v), a0, sw):
        return abs(d - r)
    return min(dist(q, polar(c, r, a0)), dist(q, polar(c, r, a0 + sw)))


def _clean_prim(p):
    """None for a degenerate primitive; curves lose repeated points."""
    if p[0] == "line":
        return p if dist(p[1], p[2]) > NODE_TOL else None
    if p[0] == "arc":
        return p if p[2] > NODE_TOL and abs(p[4]) * p[2] > NODE_TOL else None
    pts = []
    for q in p[1]:
        if not pts or dist(q, pts[-1]) > NODE_TOL:
            pts.append(tuple(q))
    return ("curve", pts) if len(pts) >= 2 else None


def chain_loop(prims, tol=1e-5):
    """Order and orient a loop's primitives head to tail."""
    rest = list(prims)
    out = [rest.pop(0)]
    while rest:
        end = prim_end(out[-1])
        best = None
        for i, p in enumerate(rest):
            for q in (p, prim_reverse(p)):
                d = dist(prim_start(q), end)
                if best is None or d < best[0]:
                    best = (d, i, q)
        if best[0] > tol:
            raise ValueError("A boundary loop isn't closed (gap of %.3g cm)." % best[0])
        out.append(best[2])
        rest.pop(best[1])
    gap = dist(prim_end(out[-1]), prim_start(out[0]))
    if gap > tol:
        raise ValueError("A boundary loop isn't closed (gap of %.3g cm)." % gap)
    return out


def _split_kinks(loop, max_turn=0.35):
    """Split polyline curves at sharp corners, so each piece can be offset smoothly."""
    out = []
    for p in loop:
        if p[0] != "curve":
            out.append(p)
            continue
        pts = p[1]
        start = 0
        for i in range(1, len(pts) - 1):
            if abs(turn(sub(pts[i], pts[i - 1]), sub(pts[i + 1], pts[i]))) > max_turn:
                out.append(("curve", pts[start:i + 1]))
                start = i
        out.append(("curve", pts[start:]))
    return out


def loop_polyline(loop, sag=SAG):
    pts = []
    for p in loop:
        pts.extend(prim_points(p, sag)[:-1])
    return pts


def reverse_loop(loop):
    return [prim_reverse(p) for p in reversed(loop)]


def prepare_area(area, sag=SAG):
    """Clean, chain and orient an area's loops: [outer (CCW), holes (CW)...]."""
    loops = []
    for lp in area:
        lp = [q for q in (_clean_prim(p) for p in lp) if q is not None]
        if lp:
            loops.append(_split_kinks(chain_loop(lp)))
    if not loops:
        raise ValueError("The area has no boundary.")
    areas = [poly_area(loop_polyline(lp, sag)) for lp in loops]
    order = sorted(range(len(loops)), key=lambda k: -abs(areas[k]))
    out = []
    for n, k in enumerate(order):
        lp = loops[k]
        if (areas[k] > 0) != (n == 0):
            lp = reverse_loop(lp)
        out.append(lp)
    return out


def chain_loops(prims, tol=1e-5):
    """Sort a pile of curves (an outline drawn in a sketch) into closed loops, head to tail.
    Raises ValueError where they don't close up."""
    rest = [q for q in (_clean_prim(p) for p in prims) if q is not None]
    loops = []
    while rest:
        lp = [rest.pop(0)]
        while dist(prim_end(lp[-1]), prim_start(lp[0])) > tol:
            end = prim_end(lp[-1])
            best = None
            for i, p in enumerate(rest):
                for q in (p, prim_reverse(p)):
                    d = dist(prim_start(q), end)
                    if best is None or d < best[0]:
                        best = (d, i, q)
            if best is None or best[0] > tol:
                raise ValueError("The outline isn't closed: it has a gap at (%.4g, %.4g)." % end)
            lp.append(best[2])
            rest.pop(best[1])
        loops.append(lp)
    return loops


def areas_from_loops(loops, sag=SAG):
    """Group closed loops into areas [outer, holes...] by how they nest: a loop inside one other
    loop is a hole in it, a loop inside two is a new area within that hole, and so on."""
    polys = [loop_polyline(lp, sag) for lp in loops]
    sizes = [abs(poly_area(p)) for p in polys]
    inside = [[j for j in range(len(loops)) if j != i and sizes[j] > sizes[i] and point_in_poly(polys[i][0], polys[j])]
              for i in range(len(loops))]
    areas = {}
    for i in sorted(range(len(loops)), key=lambda k: len(inside[k])):
        if len(inside[i]) % 2 == 0:
            areas[i] = [loops[i]]
        else:  # a hole: in the smallest loop around it
            areas[min(inside[i], key=lambda j: sizes[j])].append(loops[i])
    return list(areas.values())


def pocket_finder(res):
    """f(p) -> is the 2D point p inside one of the result's pockets (and not in a hole of it)?"""
    polys = []
    for pk in res["pockets"]:
        loops = [[q for prim in lp for q in prim_polyline(prim, 32)[:-1]] for lp in pk["loops"]]
        polys.append((bbox(loops[0]), loops))

    def find(p):
        for (x0, y0, x1, y1), loops in polys:
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and point_in_poly(p, loops[0]) and \
                    not any(point_in_poly(p, h) for h in loops[1:]):
                return True
        return False
    return find


def area_finder(res, sag=SAG):
    """f(p) -> is the 2D point p inside the result's area (inside an outer loop, outside its holes)?"""
    areas = [[loop_polyline(lp, sag) for lp in a] for a in res["areas"]]

    def find(p):
        return any(point_in_poly(p, a[0]) and not any(point_in_poly(p, h) for h in a[1:]) for a in areas)
    return find


def area_contains(area_loops, p, sag=SAG):
    """Is p inside a prepared area (inside the outer loop, outside the holes)?"""
    inside = False
    for lp in area_loops:
        if point_in_poly(p, loop_polyline(lp, sag)):
            inside = not inside
    return inside


def interior_point(area_loops, sag=SAG):
    """A point well inside a prepared area: the middle of the widest run of a few scanlines."""
    polys = [loop_polyline(lp, sag) for lp in area_loops]
    x0, y0, x1, y1 = bbox(polys[0])
    best = None
    for f in (0.5, 0.35, 0.65, 0.2, 0.8, 0.1, 0.9):
        y = y0 + (f + 1.234567e-7) * (y1 - y0)
        xs = []
        for pts in polys:
            n = len(pts)
            for i in range(n):
                a, b = pts[i], pts[(i + 1) % n]
                if (a[1] > y) != (b[1] > y):
                    xs.append(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
        xs.sort()
        for k in range(0, len(xs) - 1, 2):
            w = xs[k + 1] - xs[k]
            if best is None or w > best[0]:
                best = (w, (0.5 * (xs[k] + xs[k + 1]), y))
    return best[1] if best else ((x0 + x1) / 2, (y0 + y1) / 2)


# --------------------------------------------------------------------------- spatial grid
class _Grid:
    """Uniform bucket grid of items with bounding boxes."""

    def __init__(self, h):
        self.h = h
        self.cells = {}
        self.imax = None

    def _span(self, x0, y0, x1, y1):
        h = self.h
        return (range(int(math.floor(x0 / h)), int(math.floor(x1 / h)) + 1),
                range(int(math.floor(y0 / h)), int(math.floor(y1 / h)) + 1))

    def insert(self, item, box):
        xs, ys = self._span(*box)
        for i in xs:
            for j in ys:
                self.cells.setdefault((i, j), []).append(item)
        self.imax = xs[-1] if self.imax is None else max(self.imax, xs[-1])

    def query(self, box):
        xs, ys = self._span(*box)
        out = set()
        for i in xs:
            for j in ys:
                out.update(self.cells.get((i, j), ()))
        return out

    def row_right(self, p):
        """Items in p's row of buckets, from p's bucket rightwards."""
        h = self.h
        j, i0 = int(math.floor(p[1] / h)), int(math.floor(p[0] / h))
        out = set()
        if self.imax is not None:
            for i in range(i0, self.imax + 1):
                out.update(self.cells.get((i, j), ()))
        return out


def _bucket_size(boxes, diag):
    sizes = sorted(max(b[2] - b[0], b[3] - b[1]) for b in boxes) or [diag]
    return max(2.0 * sizes[len(sizes) // 2], diag / 400.0, 1e-6)


# --------------------------------------------------------------------------- erosion
class _Edge:
    """A straight piece of a raw offset. ev(t), for t in [t0, t1], is the exact curve it
    stands in for (None = the edge is exact)."""
    __slots__ = ("a", "b", "tag", "ev", "t0", "t1")

    def __init__(self, a, b, tag, ev=None, t0=0.0, t1=1.0):
        self.a, self.b, self.tag, self.ev, self.t0, self.t1 = a, b, tag, ev, t0, t1

    def at(self, f):
        if self.ev is None:
            return lerp(self.a, self.b, f)
        return self.ev(self.t0 + (self.t1 - self.t0) * f)

    def box(self):
        return (min(self.a[0], self.b[0]), min(self.a[1], self.b[1]),
                max(self.a[0], self.b[0]), max(self.a[1], self.b[1]))


def _prim_tag(p, pid):
    if p[0] == "line":
        return ("L", pid, p[1], unit(sub(p[2], p[1])))
    if p[0] == "arc":
        return ("A", pid, p[1], p[2])
    return ("S", pid)


def tagged_loop(loop, key, sag=SAG, ids=None):
    """Polyline of a prepared loop, each edge tagged with the primitive it came from
    (its id: ids[k], or (key, k))."""
    pts, tags = [], []
    for k, p in enumerate(loop):
        tag = _prim_tag(p, ids[k] if ids else (key, k))
        for q in prim_points(p, sag)[:-1]:
            pts.append(q)
            tags.append(tag)
    return pts, tags


def _offset_prim(p, b, pid, sag):
    """Raw offset of one primitive by b to its left, as edges."""
    if p[0] == "line":
        d = unit(sub(p[2], p[1]))
        n = perp(d)
        a, e = add(p[1], mul(n, b)), add(p[2], mul(n, b))
        return [_Edge(a, e, ("L", pid, a, d))]
    if p[0] == "arc":
        c, r, a0, sw = p[1], p[2], p[3], p[4]
        rho = r - b if sw > 0 else r + b  # signed: <= 0 means the offset turned inside out
        n = arc_steps(max(abs(rho), r), sw, sag)
        angs = [a0 + sw * i / n for i in range(n + 1)]
        pts = [polar(c, rho, x) for x in angs]

        def ev(x, c=c, rho=rho):
            return polar(c, rho, x)
        tag = ("A", pid, c, abs(rho))
        return [_Edge(pts[i], pts[i + 1], tag, ev, angs[i], angs[i + 1]) for i in range(n)]
    pts = p[1]
    m = len(pts)
    nrm = [perp(unit(sub(pts[i + 1], pts[i]))) for i in range(m - 1)]
    off = [add(pts[0], mul(nrm[0], b))]
    for i in range(1, m - 1):
        n1, n2 = nrm[i - 1], nrm[i]
        k = 1.0 + dot(n1, n2)  # miter: the point b from both neighbouring edge lines
        off.append(add(pts[i], mul(add(n1, n2), b / k)) if k > 1e-6 else add(pts[i], mul(n1, b)))
    off.append(add(pts[-1], mul(nrm[-1], b)))
    tag = ("S", pid)
    return [_Edge(off[i], off[i + 1], tag) for i in range(m - 1)]


def _junction(e_end, s_start, v, t1, t2, b, jid, sag):
    """Edges joining one primitive's offset to the next one's, around the shared vertex v.
    Convex corner: through v itself (the overlapping offsets are trimmed off later, along with
    this detour). Reflex corner: an arc of radius b around v."""
    a = turn(t1, t2)
    if abs(a) < 1e-6:
        if dist(e_end, s_start) <= NODE_TOL:
            return []
        return [_Edge(e_end, s_start, ("L", jid, e_end, unit(sub(s_start, e_end))))]
    if a > 0:
        return [_Edge(e_end, v, ("X", jid)), _Edge(v, s_start, ("X", jid))]
    a1 = angle_of(sub(e_end, v))
    n = arc_steps(b, a, sag)
    angs = [a1 + a * i / n for i in range(n + 1)]
    pts = [e_end] + [polar(v, b, x) for x in angs[1:-1]] + [s_start]

    def ev(x, v=v):
        return polar(v, b, x)
    tag = ("A", jid, v, b)
    return [_Edge(pts[i], pts[i + 1], tag, ev, angs[i], angs[i + 1]) for i in range(n)]


def _seg_hits(e1, e2):
    """[(t on e1, u on e2, point)] where two segments meet (both ends of a collinear overlap)."""
    a, b, c, d = e1.a, e1.b, e2.a, e2.b
    r, s = sub(b, a), sub(d, c)
    rr, ss = dot(r, r), dot(s, s)
    if rr == 0 or ss == 0:
        return []
    qa = sub(c, a)
    den = cross(r, s)
    if abs(den) <= 1e-12 * math.sqrt(rr * ss):
        if abs(cross(qa, r)) / math.sqrt(rr) > NODE_TOL:
            return []
        t0, t1 = dot(qa, r) / rr, dot(sub(d, a), r) / rr
        lo, hi = max(0.0, min(t0, t1)), min(1.0, max(t0, t1))
        if lo > hi:
            return []
        out = []
        for t in ((lo,) if hi - lo < 1e-15 else (lo, hi)):
            p = lerp(a, b, t)
            out.append((t, max(0.0, min(1.0, dot(sub(p, c), s) / ss)), p))
        return out
    t = cross(qa, s) / den
    u = cross(qa, r) / den
    tol = 1e-12
    if t < -tol or t > 1 + tol or u < -tol or u > 1 + tol:
        return []
    t, u = max(0.0, min(1.0, t)), max(0.0, min(1.0, u))
    return [(t, u, lerp(a, b, t))]


def _split_all(edges, nxt, diag):
    """Split edges wherever they cross or touch each other."""
    grid = _Grid(_bucket_size([e.box() for e in edges], diag))
    for i, e in enumerate(edges):
        grid.insert(i, e.box())
    cuts = [[] for _ in edges]
    seen = set()
    for items in grid.cells.values():
        k = len(items)
        for x in range(k):
            i = items[x]
            for y in range(x + 1, k):
                j = items[y]
                key = (i, j) if i < j else (j, i)
                if key in seen:
                    continue
                seen.add(key)
                if nxt[i] == j or nxt[j] == i:
                    continue  # neighbours share an end
                for t, u, p in _seg_hits(edges[i], edges[j]):
                    cuts[i].append((t, p))
                    cuts[j].append((u, p))
    out = []
    for e, cs in zip(edges, cuts):
        pts = [(0.0, e.a)] + sorted(cs) + [(1.0, e.b)]
        for (ta, pa), (tb, pb) in zip(pts, pts[1:]):
            if dist(pa, pb) <= NODE_TOL:
                continue
            out.append(_Edge(pa, pb, e.tag, e.ev, e.t0 + (e.t1 - e.t0) * ta, e.t0 + (e.t1 - e.t0) * tb))
    return out


class _Boundary:
    """The exact original boundary, for 'is anything closer than d' queries."""

    def __init__(self, loops, reach, sag):
        self.items = []
        boxes = []
        for lp in loops:
            for p in lp:
                if p[0] == "curve":
                    pts = p[1]
                    for i in range(len(pts) - 1):
                        self.items.append(("line", pts[i], pts[i + 1]))
                        boxes.append(bbox([pts[i], pts[i + 1]]))
                else:
                    self.items.append(p)
                    x0, y0, x1, y1 = bbox(prim_points(p, sag))
                    boxes.append((x0 - sag, y0 - sag, x1 + sag, y1 + sag))
        allb = (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))
        diag = math.hypot(allb[2] - allb[0], allb[3] - allb[1])
        self.grid = _Grid(max(reach, diag / 200.0, 1e-6))
        for i, bx in enumerate(boxes):
            self.grid.insert(i, bx)

    def closer_than(self, q, d):
        for i in self.grid.query((q[0] - d, q[1] - d, q[0] + d, q[1] + d)):
            it = self.items[i]
            if it[0] == "line":
                if seg_dist(q, it[1], it[2]) < d:
                    return True
            elif _arc_dist(q, it[1], it[2], it[3], it[4]) < d:
                return True
        return False


class _Nodes:
    """Points merged within NODE_TOL."""

    def __init__(self):
        self.cells = {}
        self.pts = []

    def id(self, p):
        h = NODE_TOL
        ix, iy = int(math.floor(p[0] / h)), int(math.floor(p[1] / h))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for nid in self.cells.get((ix + dx, iy + dy), ()):
                    if dist(self.pts[nid], p) <= h:
                        return nid
        nid = len(self.pts)
        self.pts.append(p)
        self.cells.setdefault((ix, iy), []).append(nid)
        return nid


def _assemble(edges):
    """Join directed edges into closed loops [(points, tags)]."""
    nodes = _Nodes()
    recs = []
    for e in edges:
        na, nb = nodes.id(e.a), nodes.id(e.b)
        if na != nb:
            recs.append((na, nb, e))
    outgoing = {}
    for k, (na, _, _) in enumerate(recs):
        outgoing.setdefault(na, []).append(k)
    used = [False] * len(recs)

    def direction(k):
        na, nb, _ = recs[k]
        return sub(nodes.pts[nb], nodes.pts[na])

    loops = []
    for k0 in range(len(recs)):
        if used[k0]:
            continue
        used[k0] = True
        chain = [k0]
        start, cur = recs[k0][0], recs[k0][1]
        closed = False
        while True:
            if cur == start:
                closed = True
                break
            cands = [k for k in outgoing.get(cur, ()) if not used[k]]
            if not cands:
                break
            din = direction(chain[-1])
            k = max(cands, key=lambda c: turn(din, direction(c))) if len(cands) > 1 else cands[0]
            used[k] = True
            chain.append(k)
            cur = recs[k][1]
        if closed:
            pts = [nodes.pts[recs[k][0]] for k in chain]
            if len(pts) >= 2 and abs(poly_area(pts)) > 1e-12:
                loops.append((pts, [recs[k][2].tag for k in chain]))
    return loops


def erode(loops, b, sag=SAG, ids=None, jtag="j"):
    """The region at least b inside the prepared loops (all areas' loops together), as tagged
    loops [(points, tags)]: CCW around kept regions, CW around holes.
    ids: optional ids for the primitives (ids[loop][k]); each offset piece keeps its primitive's
    id, so a curve can be followed through several offsets. Corner arcs get (jtag, id)."""
    if b <= 0:
        return [tagged_loop(lp, li, sag, ids[li] if ids else None) for li, lp in enumerate(loops)]
    edges, nxt = [], []
    for li, lp in enumerate(loops):
        first = len(edges)
        m = len(lp)
        pids = ids[li] if ids else [(li, k) for k in range(m)]
        offs = [_offset_prim(p, b, pids[k], sag) for k, p in enumerate(lp)]
        for k in range(m):
            P, Q = lp[k], lp[(k + 1) % m]
            edges.extend(offs[k])
            t1 = prim_tangents(P)[1]
            t2 = prim_tangents(Q)[0]
            v = lerp(prim_end(P), prim_start(Q), 0.5)
            edges.extend(_junction(offs[k][-1].b, offs[(k + 1) % m][0].a, v, t1, t2, b, (jtag, pids[k]), sag))
        nxt.extend(range(first + 1, len(edges)))
        nxt.append(first)
    allpts = [p for e in edges for p in (e.a, e.b)]
    x0, y0, x1, y1 = bbox(allpts)
    diag = math.hypot(x1 - x0, y1 - y0)
    subs = _split_all(edges, nxt, diag)
    bound = _Boundary(loops, b, sag)
    lim = b - (1e-10 + 1e-9 * b)
    keep = [e for e in subs if e.tag[0] != "X" and not bound.closer_than(e.at(0.5), lim)]
    return _assemble(keep)


# --------------------------------------------------------------------------- lattice
def pocket_inradius(shape, size_by, value):
    """Inradius of a regular pocket given its side length or inscribed-circle diameter."""
    if size_by == INSCRIBED:
        return value / 2.0
    return {TRIANGLE: value / (2.0 * SQ3), SQUARE: value / 2.0, HEXAGON: value * SQ3 / 2.0}[shape]


def pocket_side(shape, inradius):
    return {TRIANGLE: inradius * 2.0 * SQ3, SQUARE: inradius * 2.0, HEXAGON: inradius * 2.0 / SQ3}[shape]


def allowed_phases(shape, rot90=False):
    """What each symmetry axis can run through, as (vertical axis options, horizontal axis options).
    A hexagon grid's mirror lines across its columns pass through cell centres only; a triangle
    grid's vertical mirror lines pass through nodes and cell centres, its horizontal ones along ribs."""
    if shape in (SQUARE, RECTANGLE):
        v = h = (CELL, RIB)
    elif shape == HEXAGON:
        v, h = (CELL,), (CELL, RIB)
    else:
        v, h = (CELL,), (RIB,)
    return (h, v) if rot90 else (v, h)


def lattice_cell_area(shape, dims, rib):
    """Area of the plane per cell (pocket plus its share of rib)."""
    if shape == SQUARE:
        return (2 * dims + rib) ** 2
    if shape == RECTANGLE:
        return (dims[0] + rib) * (dims[1] + rib)
    if shape == HEXAGON:
        return SQ3 / 2.0 * (2 * dims + rib) ** 2
    s = 2.0 * SQ3 * (dims + rib / 2.0)
    return SQ3 / 4.0 * s * s


def lattice(shape, dims, rib, bounds, rot90=False, vphase=CELL, hphase=CELL):
    """Pocket polygons (CCW, grid frame) for every cell near bounds = (umin, vmin, umax, vmax).
    dims: the pocket's inradius, or (length, width) for rectangles. The pattern is mirror-
    symmetric about u = 0 (through cells or along a rib, per vphase) and v = 0 (per hphase)."""
    if rot90:
        umin, vmin, umax, vmax = bounds
        cells = lattice(shape, dims, rib, (vmin, -umax, vmax, -umin), False, hphase, vphase)
        return [[(-p[1], p[0]) for p in c] for c in cells]
    umin, vmin, umax, vmax = bounds
    fl, ce = math.floor, math.ceil
    cells = []
    if shape in (SQUARE, RECTANGLE):
        hl, hw = (dims, dims) if shape == SQUARE else (dims[0] / 2.0, dims[1] / 2.0)
        px, py = 2 * hl + rib, 2 * hw + rib
        u0 = 0.0 if vphase == CELL else px / 2.0
        v0 = 0.0 if hphase == CELL else py / 2.0
        for i in range(int(fl((umin - u0) / px)) - 1, int(ce((umax - u0) / px)) + 2):
            for j in range(int(fl((vmin - v0) / py)) - 1, int(ce((vmax - v0) / py)) + 2):
                cu, cv = u0 + i * px, v0 + j * py
                cells.append([(cu - hl, cv - hw), (cu + hl, cv - hw), (cu + hl, cv + hw), (cu - hl, cv + hw)])
    elif shape == HEXAGON:
        # flat top: columns 1.5 R apart, every other column shifted half a cell
        F = 2 * dims + rib
        cx = 1.5 * F / SQ3
        Rc = 2 * dims / SQ3
        v0 = 0.0 if hphase == CELL else F / 2.0
        for i in range(int(fl(umin / cx)) - 1, int(ce(umax / cx)) + 2):
            off = v0 + (F / 2.0 if i % 2 else 0.0)
            for j in range(int(fl((vmin - off) / F)) - 1, int(ce((vmax - off) / F)) + 2):
                c = (i * cx, off + j * F)
                cells.append([polar(c, Rc, k * math.pi / 3.0) for k in range(6)])
    else:
        # rows of alternating up/down triangles; nodes at (i s + j s/2, j h)
        rL = dims + rib / 2.0
        s, h = 2.0 * SQ3 * rL, 3.0 * rL
        Rc = 2.0 * dims
        up_a = [math.radians(a) for a in (210, 330, 90)]
        dn_a = [math.radians(a) for a in (270, 30, 150)]
        for j in range(int(fl(vmin / h)) - 1, int(ce(vmax / h)) + 2):
            x0 = j * s / 2.0
            for i in range(int(fl((umin - x0) / s)) - 2, int(ce((umax - x0) / s)) + 2):
                bx = x0 + i * s
                up, dn = (bx + s / 2.0, j * h + rL), (bx + s, j * h + 2 * rL)
                cells.append([polar(up, Rc, a) for a in up_a])
                cells.append([polar(dn, Rc, a) for a in dn_a])
    return cells


def grid_frame(centre, vline=None, hline=None):
    """(origin, angle of the grid's u axis). vline/hline: (p0, p1) of the symmetry lines, or None.
    The grid's axes follow the lines (so tilted lines tilt the grid); without lines the grid is
    centred on `centre` and square to the sketch."""
    if vline is None and hline is None:
        return centre, 0.0
    dv = dh = None
    if vline is not None:
        dv = unit(sub(vline[1], vline[0]))
        if dv[1] < 0 or (dv[1] == 0 and dv[0] > 0):
            dv = mul(dv, -1.0)
    if hline is not None:
        dh = unit(sub(hline[1], hline[0]))
        if dh[0] < 0 or (dh[0] == 0 and dh[1] < 0):
            dh = mul(dh, -1.0)
    if dv is not None and dh is not None:
        if abs(dot(dv, dh)) > 1e-6:
            raise ValueError("The vertical and horizontal symmetry lines must be perpendicular.")
        # where the lines cross
        den = cross(dh, dv)
        t = cross(sub(vline[0], hline[0]), dv) / den
        return add(hline[0], mul(dh, t)), angle_of(dh)
    if dh is not None:
        foot = add(hline[0], mul(dh, dot(sub(centre, hline[0]), dh)))
        return foot, angle_of(dh)
    foot = add(vline[0], mul(dv, dot(sub(centre, vline[0]), dv)))
    return foot, angle_of((dv[1], -dv[0]))


# --------------------------------------------------------------------------- clipping
class _Region:
    """Tagged loops with a bucket index: edges near a box, and point-in-region."""

    def __init__(self, loops, h):
        self.loops = loops
        self.grid = _Grid(h)
        for li, (pts, _) in enumerate(loops):
            m = len(pts)
            for k in range(m):
                a, b = pts[k], pts[(k + 1) % m]
                self.grid.insert((li, k), (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))

    def near(self, box):
        return self.grid.query(box)

    def inside(self, p):
        c = 0
        for li, k in self.grid.row_right(p):
            pts = self.loops[li][0]
            a, b = pts[k], pts[(k + 1) % len(pts)]
            if (a[1] > p[1]) != (b[1] > p[1]):
                if a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) > p[0]:
                    c += 1
        return c % 2 == 1


def _clip_cell(cell, cid, region, eps=1e-9):
    """Loops [(points, tags)] of a convex CCW pocket polygon intersected with the region.
    Region points on the pocket's edge count as outside a pocket shrunk by eps, which keeps
    exact touches (a pocket edge lying on the border) from creating crossings."""
    n = len(cell)
    dirs = [unit(sub(cell[(k + 1) % n], cell[k])) for k in range(n)]
    lens = [dist(cell[(k + 1) % n], cell[k]) for k in range(n)]
    hp = [(perp(d), dot(perp(d), c) + eps) for d, c in zip(dirs, cell)]
    ctags = [("L", ("c", cid, k), cell[k], dirs[k]) for k in range(n)]
    box = bbox(cell)
    near = region.near(box)

    def full():
        return [(list(cell), list(ctags))]

    def probe():
        # a point just inside the pocket's first edge, away from its corners
        return add(lerp(cell[0], cell[1], 0.5), mul(hp[0][0], 2 * eps))

    if not near:
        return (full(), True) if region.inside(probe()) else ([], False)

    vals = {}

    def values(li, vi):
        key = (li, vi)
        v = vals.get(key)
        if v is None:
            p = region.loops[li][0][vi]
            v = [dot(nm, p) - off for nm, off in hp]
            vals[key] = v
        return v

    def inside(li, vi):
        return all(x > 0 for x in values(li, vi))

    def perim(k, p):
        return k + max(0.0, min(1.0, dot(sub(p, cell[k]), dirs[k]) / lens[k]))

    by_loop = {}
    for li, k in near:
        by_loop.setdefault(li, set()).add(k)
    pieces, whole = [], []
    for li in sorted(by_loop):
        pts, tags = region.loops[li]
        m = len(pts)
        ks = by_loop[li]
        if len(ks) == m and all(inside(li, v) for v in range(m)):
            whole.append((list(pts), list(tags)))
            continue
        order = sorted(ks)
        s0 = next((i for i, k in enumerate(order) if not inside(li, k)), None)
        if s0 is None:
            continue  # can't happen: the edges not listed are far away, so their ends are outside
        cur = None
        # edges not in `order` are far from the pocket, so wholly outside it
        for k in order[s0:] + order[:s0]:
            p, q = pts[k], pts[(k + 1) % m]
            va, vb = values(li, k), values(li, (k + 1) % m)
            qin = inside(li, (k + 1) % m)
            tin, tout, kin, kout = 0.0, 1.0, -1, -1
            empty = False
            for j in range(n):
                a_, b_ = va[j], vb[j]
                if a_ > 0 and b_ > 0:
                    continue
                if a_ <= 0 and b_ <= 0:
                    empty = True
                    break
                t = a_ / (a_ - b_)
                if a_ <= 0:
                    if t > tin or kin < 0:
                        tin, kin = max(t, tin), j
                elif t < tout or kout < 0:
                    tout, kout = min(t, tout), j
            if cur is None:
                if empty:
                    continue
                if qin:
                    x = lerp(p, q, tin)
                    cur = {"pts": [x, q], "tags": [tags[k]], "s_in": perim(kin, x)}
                elif tin < tout and kin >= 0 and kout >= 0:
                    x, y = lerp(p, q, tin), lerp(p, q, tout)
                    pieces.append({"pts": [x, y], "tags": [tags[k]], "s_in": perim(kin, x), "s_out": perim(kout, y)})
            elif qin:
                cur["pts"].append(q)
                cur["tags"].append(tags[k])
            else:
                y = lerp(p, q, tout)
                cur["pts"].append(y)
                cur["tags"].append(tags[k])
                cur["s_out"] = perim(kout if kout >= 0 else 0, y)
                pieces.append(cur)
                cur = None

    if not pieces:
        out = full() if region.inside(probe()) else []
        return out + whole, (not whole and bool(out))

    # walk: piece -> along the pocket's edge (CCW) to the next entry -> next piece ...
    loops = []
    used = [False] * len(pieces)
    for i0 in range(len(pieces)):
        if used[i0]:
            continue
        lp, lt = [], []
        i = i0
        for _ in range(len(pieces) + 1):
            used[i] = True
            P = pieces[i]
            lp.extend(P["pts"][:-1])
            lt.extend(P["tags"])
            s_out = P["s_out"]

            def ahead(j):
                d = (pieces[j]["s_in"] - s_out) % n
                return 0.0 if d > n - 1e-12 else d
            j = min(range(len(pieces)), key=ahead)
            lp.append(P["pts"][-1])
            ko = int(s_out) % n
            steps = (int(pieces[j]["s_in"]) % n - ko) % n
            if steps == 0 and ahead(j) > 1.0 - 1e-12:
                steps = n
            lt.append(ctags[ko])
            for st in range(1, steps + 1):
                kk = (ko + st) % n
                lp.append(cell[kk])
                lt.append(ctags[kk])
            i = j
            if used[i]:
                break
        loops.append((lp, lt))
    return loops + whole, False


def _clean_loop(pts, tags):
    """Merge vertices closer than MIN_EDGE; the merged edge keeps the longer edge's tag."""
    op, ot = [], []
    for p, t in zip(pts, tags):
        if op and dist(p, op[-1]) < MIN_EDGE:
            ot[-1] = t
            continue
        op.append(p)
        ot.append(t)
    while len(op) > 1 and dist(op[0], op[-1]) < MIN_EDGE:
        op.pop()
        ot.pop()
    return (op, ot) if len(op) >= 3 else None


# --------------------------------------------------------------------------- back to curves
def _line_line(p1, d1, p2, d2):
    den = cross(d1, d2)
    if abs(den) < 1e-6:
        return None
    t = cross(sub(p2, p1), d2) / den
    return add(p1, mul(d1, t))


def _line_circle(p, d, c, r, near):
    f = add(p, mul(d, dot(sub(c, p), d)))
    h2 = r * r - dot(sub(f, c), sub(f, c))
    if h2 <= (1e-4 * r) ** 2:
        return None  # misses or nearly tangent: keep the computed point
    s = math.sqrt(h2)
    return min((add(f, mul(d, s)), sub(f, mul(d, s))), key=lambda q: dist(q, near))


def _circle_circle(c1, r1, c2, r2, near):
    d = dist(c1, c2)
    if d < 1e-12:
        return None
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h2 = r1 * r1 - a * a
    if h2 <= (1e-4 * min(r1, r2)) ** 2:
        return None
    h = math.sqrt(h2)
    u = unit(sub(c2, c1))
    m = add(c1, mul(u, a))
    return min((add(m, mul(perp(u), h)), sub(m, mul(perp(u), h))), key=lambda q: dist(q, near))


def _meet(p, ta, tb):
    """Exact meeting point of two runs' curves near p (lines and circles), else p."""
    ka, kb = ta[0], tb[0]
    if ka not in "LA" or kb not in "LA":
        return p
    if ka == "L" and kb == "L":
        q = _line_line(ta[2], ta[3], tb[2], tb[3])
    elif ka == "L":
        q = _line_circle(ta[2], ta[3], tb[2], tb[3], p)
    elif kb == "L":
        q = _line_circle(tb[2], tb[3], ta[2], ta[3], p)
    else:
        q = _circle_circle(ta[2], ta[3], tb[2], tb[3], p)
    if q is None or dist(q, p) > 1e-3:
        return p
    return q


def _thin(pts, most):
    if len(pts) <= most:
        return list(pts)
    return [pts[int(round(i * (len(pts) - 1) / (most - 1)))] for i in range(most)]


def _run_prim(tag, rp):
    a, b = rp[0], rp[-1]
    if tag[0] == "A":
        c, R = tag[2], tag[3]
        sw = sum(turn(sub(rp[i], c), sub(rp[i + 1], c)) for i in range(len(rp) - 1))
        if abs(sw) * R > 10 * MIN_EDGE and abs(sw) > 1e-6:
            return ("arc3", a, polar(c, R, angle_of(sub(a, c)) + sw / 2.0), b, c, sw)
        return ("seg", a, b, False)
    if tag[0] == "S" and len(rp) > 2:
        return ("spline", _thin(rp, 40), False)
    return ("seg", a, b, tag[0] == "L" and isinstance(tag[1], tuple) and tag[1][0] == "c")


def loop_runs(pts, tags):
    """A tagged loop as runs [(tag, points)] of the same curve, with the corners between runs
    recomputed exactly. A loop that is one closed curve gives one run ending where it starts."""
    n = len(pts)
    ids = [t[1] for t in tags]
    starts = [i for i in range(n) if ids[i] != ids[i - 1]]
    if not starts:
        return [(tags[0], list(pts) + [pts[0]])]
    runs = []
    for a, b in zip(starts, starts[1:] + [starts[0] + n]):
        runs.append([tags[a], [pts[i % n] for i in range(a, b + 1)]])
    for r in range(len(runs)):
        (t1, p1), (t2, p2) = runs[r], runs[(r + 1) % len(runs)]
        q = _meet(p1[-1], t1, t2)
        p1[-1] = q
        p2[0] = q
    return runs


def loop_items(pts, tags):
    """[(drawing primitive, id of the curve it lies on)] for a tagged loop."""
    runs = loop_runs(pts, tags)
    if len(runs) == 1:  # a single closed curve
        t = runs[0][0]
        if t[0] == "A":
            return [(("circle", t[2], t[3]), t[1])]
        return [(("spline", _thin(pts, 60), True), t[1])]
    return [(_run_prim(t, rp), t[1]) for t, rp in runs]


def loop_prims(pts, tags):
    """Drawing primitives for a tagged loop: one per run of the same curve."""
    return [p for p, _ in loop_items(pts, tags)]


def _kernel_loop(pts, tags):
    """A tagged loop as kernel primitives (to offset it again), with each primitive's id."""
    prims, ids = [], []
    runs = loop_runs(pts, tags)
    for t, rp in runs:
        a, b = rp[0], rp[-1]
        if t[0] == "A":
            c, R = t[2], t[3]
            sw = sum(turn(sub(rp[i], c), sub(rp[i + 1], c)) for i in range(len(rp) - 1))
            if len(runs) == 1:
                sw = math.copysign(TAU, sw)
            if abs(sw) * R > NODE_TOL and abs(sw) > 1e-9:
                prims.append(("arc", c, R, angle_of(sub(a, c)), sw))
            elif dist(a, b) > NODE_TOL:
                prims.append(("line", a, b))
            else:
                continue
        elif t[0] == "S" and len(rp) > 2:
            prims.append(("curve", rp))
        elif dist(a, b) > NODE_TOL:
            prims.append(("line", a, b))
        else:
            continue
        ids.append(t[1])
    return prims, ids


def _reverse_tagged(pts, tags):
    n = len(pts)
    return list(reversed(pts)), [tags[(n - 2 - j) % n] for j in range(n)]


def round_corners(loops, r, sag=SAG):
    """Round the convex corners of a region (tagged loops) to radius r, leaving its inward
    corners sharp: shrink it by r, then grow it back by r. Parts narrower than 2r vanish, as
    they would for a cutter of radius r. Curves keep their ids; the new corner arcs get
    ("fillet", id)."""
    conv = [c for c in (_kernel_loop(p, t) for p, t in loops) if c[0]]
    if not conv:
        return []
    inner = erode([c[0] for c in conv], r, sag, ids=[c[1] for c in conv], jtag="shrunk")
    back = [c for c in (_kernel_loop(*_reverse_tagged(p, t)) for p, t in inner) if c[0]]
    if not back:
        return []
    grown = erode([c[0] for c in back], r, sag, ids=[c[1] for c in back], jtag="fillet")  # shrinks the outside
    return [_reverse_tagged(p, t) for p, t in grown]


# G3 (7-point) and G4 (9-point) corner curves: per corner angle in degrees, the leg length as a
# multiple of the radius-R arc's tangent length, then the inner leg points' spacing (fractions of
# the leg, outer to inner). Tabulated from gx_math.fillet_smoothest (the smoothest curve, sized to
# fit the R fillet best; what GX G3 Fillet makes by default), since each new angle takes seconds.
CORNER_TABLE = {
    7: [
        [5, 1.554457, 0.980000, 0.960400],
        [10, 1.476485, 0.980000, 0.960400],
        [15, 1.411883, 0.980000, 0.960400],
        [20, 1.373366, 0.968042, 0.948681],
        [25, 1.345900, 0.952548, 0.933497],
        [30, 1.327319, 0.934090, 0.915408],
        [35, 1.317157, 0.912500, 0.894250],
        [40, 1.315493, 0.887548, 0.869797],
        [45, 1.322785, 0.858941, 0.841762],
        [50, 1.339974, 0.826430, 0.809901],
        [55, 1.361577, 0.803260, 0.774575],
        [60, 1.376051, 0.801794, 0.740063],
        [65, 1.390527, 0.800307, 0.707938],
        [70, 1.404585, 0.799195, 0.678210],
        [75, 1.417869, 0.798614, 0.650825],
        [80, 1.430296, 0.798545, 0.625672],
        [85, 1.441791, 0.798874, 0.602604],
        [90, 1.452488, 0.799462, 0.581464],
        [95, 1.462459, 0.800184, 0.562103],
        [100, 1.471739, 0.800946, 0.544390],
        [105, 1.480389, 0.801688, 0.528209],
        [110, 1.488461, 0.802374, 0.513460],
        [115, 1.495968, 0.802987, 0.500057],
        [120, 1.502917, 0.803523, 0.487922],
        [125, 1.509310, 0.803981, 0.476989],
        [130, 1.515146, 0.804369, 0.467196],
        [135, 1.520424, 0.804694, 0.458489],
        [140, 1.525128, 0.804962, 0.450822],
        [145, 1.529271, 0.805182, 0.444152],
        [150, 1.532858, 0.805359, 0.438443],
        [155, 1.535890, 0.805500, 0.433665],
        [160, 1.538369, 0.805609, 0.429789],
        [165, 1.540295, 0.805690, 0.426797],
        [170, 1.541669, 0.805746, 0.424671],
        [175, 1.542493, 0.805779, 0.423400],
    ],
    9: [
        [5, 1.787187, 0.980000, 0.960400, 0.941192],
        [10, 1.694999, 0.980000, 0.960400, 0.941192],
        [15, 1.618599, 0.980000, 0.960400, 0.941192],
        [20, 1.552513, 0.980000, 0.960400, 0.941192],
        [25, 1.494939, 0.980000, 0.960400, 0.941192],
        [30, 1.462117, 0.980000, 0.948187, 0.929224],
        [35, 1.437774, 0.980000, 0.933650, 0.914977],
        [40, 1.418807, 0.980000, 0.918031, 0.899670],
        [45, 1.405288, 0.980000, 0.901107, 0.883085],
        [50, 1.397426, 0.980000, 0.882557, 0.864905],
        [55, 1.395730, 0.980000, 0.861971, 0.844731],
        [60, 1.400943, 0.980000, 0.838932, 0.822153],
        [65, 1.408939, 0.980000, 0.835147, 0.796838],
        [70, 1.417926, 0.980000, 0.831886, 0.772635],
        [75, 1.427626, 0.980000, 0.828190, 0.749705],
        [80, 1.437773, 0.980000, 0.824437, 0.728096],
        [85, 1.448040, 0.979919, 0.820953, 0.707838],
        [90, 1.458344, 0.978258, 0.818692, 0.688773],
        [95, 1.467898, 0.976600, 0.817034, 0.671280],
        [100, 1.476775, 0.974962, 0.815902, 0.655270],
        [105, 1.484932, 0.973367, 0.815189, 0.640646],
        [110, 1.492420, 0.971837, 0.814786, 0.627311],
        [115, 1.499337, 0.970393, 0.814599, 0.615177],
        [120, 1.505718, 0.969049, 0.814557, 0.604170],
        [125, 1.511544, 0.967815, 0.814605, 0.594225],
        [130, 1.516843, 0.966697, 0.814705, 0.585291],
        [135, 1.521653, 0.965697, 0.814831, 0.577323],
        [140, 1.525975, 0.964815, 0.814966, 0.570283],
        [145, 1.529804, 0.964047, 0.815098, 0.564141],
        [150, 1.533136, 0.963391, 0.815219, 0.558868],
        [155, 1.535966, 0.962842, 0.815325, 0.554443],
        [160, 1.538289, 0.962399, 0.815415, 0.550848],
        [165, 1.540101, 0.962057, 0.815485, 0.548067],
        [170, 1.541397, 0.961814, 0.815535, 0.546088],
        [175, 1.542176, 0.961670, 0.815565, 0.544904],
    ],
}

CORNER_COUNTS = {"g3": 7, "g4": 9}


def corner_shape(phi, count):
    """(leg length for R = 1, fractions outer -> inner) of a G3/G4 corner of angle phi."""
    rows = CORNER_TABLE[count]
    deg = min(max(math.degrees(phi), rows[0][0]), rows[-1][0])
    i = min(int((deg - rows[0][0]) // (rows[1][0] - rows[0][0])), len(rows) - 2)
    a, b = rows[i], rows[i + 1]
    w = (deg - a[0]) / float(b[0] - a[0])
    vals = [x + (y - x) * w for x, y in zip(a[1:], b[1:])]
    return vals[0] / math.tan(phi / 2.0), [1.0] + vals[1:]


def _corner_layout(count):
    def layout(phi):
        L, fr = corner_shape(phi, count)
        return dict(L=L, arc_tangent=1.0 / math.tan(phi / 2.0), fractions=fr)
    return layout


def _line_corner(X, uA, uB, R, count):
    """Control points of a G3/G4 curve for the corner X between legs uA, uB (unit, pointing away
    from X), standing in for a radius-R fillet. Returns (ctrl, leg length)."""
    phi = math.acos(max(-1.0, min(1.0, dot(uA, uB))))
    L1, fr = corner_shape(phi, count)
    dists = [R * L1 * f for f in fr]
    ctrl = [add(X, mul(uA, d)) for d in dists] + [X] + [add(X, mul(uB, d)) for d in reversed(dists)]
    return ctrl, R * L1


# --------------------------------------------------------------------------- bottom blends
# Polygrid can round the bottom of its cuts (where the pocket walls meet the floor) or of what it
# extrudes (where the walls meet the body below), with a radius, or a G3/G4 curve swept around
# each pocket outline.
BLEND_SIDE = {"cut_holes": 1, "cut_ribs": -1, "extrude_holes": -1, "extrude_ribs": 1}


def bottom_reach(R, style):
    """How far up the wall and across the floor a bottom blend of size R reaches."""
    if style in CORNER_COUNTS:
        return R * corner_shape(math.pi / 2, CORNER_COUNTS[style])[0]
    return R


def bottom_profile(R, style):
    """Control points of a G3/G4 bottom blend's cross-section, from the floor contact to the wall
    contact: the corner at the origin, the floor along +x, the wall along +y."""
    return _line_corner((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), R, CORNER_COUNTS[style])[0]


def blend_loops(res, op):
    """[(pocket index, loop index, prims, side)]: the pocket outlines whose bottom edge gets a blend
    for this operation, and the side of the outline (as it runs: +1 left, -1 right) the blend goes
    on: into the pocket when the pockets are cut or the ribs extruded, out of it otherwise.
    Extrusions skip the outlines of holes inside pockets (the area's holes may go right through)."""
    side = BLEND_SIDE[op]
    out = []
    for i, pk in enumerate(res["pockets"]):
        for j, lp in enumerate(pk["loops"]):
            if j == 0 or op.startswith("cut"):
                out.append((i, j, lp, side))
    return out


def _key_points(p):
    if p[0] == "seg":
        return [p[1], p[2]]
    if p[0] == "arc3":
        return [p[1], p[2], p[3]]
    if p[0] == "circle":
        return [p[1], (p[1][0] + p[2], p[1][1])]
    return list(p[1])


def shape_groups(loops, tol=1e-6):
    """Group closed loops of drawing primitives that are translated copies of one another.
    Returns [(seed index, [(index, (dx, dy) from the seed)...])], each group listing its seed too."""
    groups = {}
    for k, prims in enumerate(loops):
        pts = [q for p in prims for q in _key_points(p)]
        x0, y0 = min(q[0] for q in pts), min(q[1] for q in pts)
        key = tuple(sorted((p[0],) + tuple((int(round((q[0] - x0) / tol)), int(round((q[1] - y0) / tol)))
                                           for q in _key_points(p)) for p in prims))
        groups.setdefault(key, []).append((k, (x0, y0)))
    out = []
    for members in groups.values():
        seed, ref = members[0]
        out.append((seed, [(k, (r[0] - ref[0], r[1] - ref[1])) for k, r in members]))
    return out


def loop_distance(prims, p, steps=32):
    """Distance from point p to a loop of drawing primitives."""
    best = float("inf")
    for prim in prims:
        pts = prim_polyline(prim, steps)
        for a, b in zip(pts, pts[1:]):
            best = min(best, seg_dist(p, a, b))
    return best


def max_corner(shape, dims, style="arc"):
    """The largest corner radius a whole pocket can take."""
    if style == "arc":
        return min(dims) / 2.0 if shape == RECTANGLE else dims
    count = CORNER_COUNTS[style]
    if shape == RECTANGLE:  # legs from both ends of the shorter side must fit
        return min(dims) / 2.0 / corner_shape(math.pi / 2, count)[0]
    n = {TRIANGLE: 3, SQUARE: 4, HEXAGON: 6}[shape]
    phi = math.pi - TAU / n
    return pocket_side(shape, dims) / 2.0 / corner_shape(phi, count)[0]


def _rounded_cell(cell, r, style="arc"):
    """Drawing primitives and area of a convex pocket polygon with its corners rounded to
    radius r: arcs, or G3 / G4 curves standing in for them."""
    n = len(cell)
    dirs = [unit(sub(cell[(k + 1) % n], cell[k])) for k in range(n)]
    corners = []
    area = poly_area(cell)
    for k in range(n):
        din, dout, v = dirs[k - 1], dirs[k], cell[k]
        if style == "arc":
            tw = turn(din, dout)  # how far the outline turns at this corner
            t = r * math.tan(tw / 2.0)  # corner to tangent point
            c = add(v, mul(unit(sub(dout, din)), r / math.cos(tw / 2.0)))
            a, b = sub(v, mul(din, t)), add(v, mul(dout, t))
            corners.append((a, b, ("arc3", a, polar(c, r, angle_of(sub(a, c)) + tw / 2.0), b, c, tw)))
            area -= r * t - 0.5 * r * r * tw  # the corner's kite, less the arc's sector
        else:
            ctrl, _ = _line_corner(v, mul(din, -1.0), dout, r, CORNER_COUNTS[style])
            corners.append((ctrl[0], ctrl[-1], ("cspline", ctrl, 5)))
    prims = []
    for k, (a, b, p) in enumerate(corners):
        prims.append(p)
        nxt = corners[(k + 1) % n][0]
        if dist(b, nxt) > MIN_EDGE:
            prims.append(("seg", b, nxt, True))
    if style != "arc":
        area = poly_area([q for p in prims for q in prim_polyline(p, 200)[:-1]])
    return prims, area


def _run_curve(p):
    """(gx_math Curve2D, parameter at a point, direction of travel in parameter) for a seg or arc."""
    if p[0] == "seg":
        a, d = p[1], unit(sub(p[2], p[1]))
        return g.line_curve(a, d), (lambda q: dot(sub(q, a), d)), 1.0
    c, sw = p[4], p[5]
    R = dist(p[1], c)
    return g.ellipse_curve(c, R, R), (lambda q: angle_of(sub(q, c))), (1.0 if sw > 0 else -1.0)


def _corner_blend(P, F, Q, R, count):
    """G3/G4 curve replacing fillet arc F between runs P (before it) and Q (after it), radius R.
    Returns (ctrl, contact on P, contact on Q) or None."""
    if P[0] not in ("seg", "arc3") or Q[0] not in ("seg", "arc3"):
        return None
    try:
        if P[0] == "seg" and Q[0] == "seg":
            dP, dQ = unit(sub(P[2], P[1])), unit(sub(Q[2], Q[1]))
            X = _line_line(P[1], dP, Q[1], dQ)
            if X is None:
                return None
            ctrl, _ = _line_corner(X, mul(dP, -1.0), dQ, R, count)
            return ctrl, ctrl[0], ctrl[-1]
        A, pa, sgnA = _run_curve(P)
        B, pb, sgnB = _run_curve(Q)
        hit = g.find_intersection(A, pa(F[1]), B, pb(F[3]))  # the corner the fillet rounds off
        if hit is None:
            return None
        res = g.curve_fillet(A, hit[0], -sgnA, B, hit[1], sgnB, R, count, quick=True, layout=_corner_layout(count))
        return res["ctrl"], A.point(res["ua"]), B.point(res["ub"])
    except (ValueError, ZeroDivisionError):
        return None


def _along(p, q):
    """Where q lies along a seg or arc3 primitive, 0 at its start to 1 at its end."""
    if p[0] == "seg":
        d = sub(p[2], p[1])
        return dot(sub(q, p[1]), d) / dot(d, d)
    c, sw = p[4], p[5]
    t = turn(sub(p[1], c), sub(q, c))
    if sw > 0 and t < -1e-9:
        t += TAU
    elif sw < 0 and t > 1e-9:
        t -= TAU
    return t / sw


def _trim(p, a, b):
    """A seg or arc3 primitive cut down to run from a to b (points on it)."""
    if p[0] == "seg":
        return ("seg", a, b, p[3])
    c, sw = p[4], p[5]
    s = (_along(p, b) - _along(p, a)) * sw
    return ("arc3", a, polar(c, dist(a, c), angle_of(sub(a, c)) + s / 2.0), b, c, s)


def blend_corners(items, R, count, tries=(1.0, 0.75, 0.5, 0.3)):
    """Replace the fillet arcs of a rounded loop [(prim, id)] by G3/G4 curves of radius R. A curve
    whose legs don't fit on its neighbouring runs is retried smaller; one that can't fit at all,
    or whose neighbour is a spline, stays an arc. Returns (items, arcs kept, corners made smaller)."""
    n = len(items)
    fil = [i for i, (p, pid) in enumerate(items)
           if p[0] == "arc3" and isinstance(pid, tuple) and pid[:1] == ("fillet",)]
    if not fil or n < 3:
        return items, 0, 0
    level = {i: 0 for i in fil}
    blend = {}

    def make(i):
        k = level[i]
        P, F, Q = items[i - 1][0], items[i][0], items[(i + 1) % n][0]
        b = None if k >= len(tries) or (i - 1) % n in level or (i + 1) % n in level else \
            _corner_blend(P, F, Q, R * tries[k], count)
        if b is None:
            blend.pop(i, None)
            level[i] = len(tries)
        else:
            blend[i] = b

    for i in fil:
        make(i)
    for _ in range(4 * len(tries)):
        bad = set()
        for j in range(n):  # each run: its trimmed start must stay before its trimmed end
            if j in level:
                continue
            p = items[j][0]
            if p[0] not in ("seg", "arc3"):
                continue
            prev, nxt = (j - 1) % n, (j + 1) % n
            s = _along(p, blend[prev][2]) if prev in blend else 0.0
            e = _along(p, blend[nxt][1]) if nxt in blend else 1.0
            if not (-1e-9 <= s < e - 1e-9 and e <= 1 + 1e-9):
                bad.update(k for k in (prev, nxt) if k in blend)
        if not bad:
            break
        for i in bad:
            level[i] += 1
            make(i)
    out = []
    for j, (p, pid) in enumerate(items):
        if j in blend:
            out.append((("cspline", blend[j][0], 5), pid))
        elif j in level or p[0] not in ("seg", "arc3"):
            out.append((p, pid))
        else:
            prev, nxt = (j - 1) % n, (j + 1) % n
            a = blend[prev][2] if prev in blend else p[1]
            b = blend[nxt][1] if nxt in blend else (p[2] if p[0] == "seg" else p[3])
            out.append((_trim(p, a, b), pid))
    kept = sum(1 for i in fil if i not in blend)
    smaller = sum(1 for i in fil if i in blend and level[i] > 0)
    return out, kept, smaller


def outline_prims(loop):
    """Drawing primitives for a prepared boundary loop."""
    if len(loop) == 1 and loop[0][0] == "curve":  # one closed curve (a spline or ellipse hole)
        pts = loop[0][1]
        return [("spline", _thin(pts[:-1] if dist(pts[0], pts[-1]) < 1e-7 else pts, 200), True)]
    out = []
    for p in loop:
        if p[0] == "line":
            out.append(("seg", p[1], p[2], False))
        elif p[0] == "arc":
            c, r, a0, sw = p[1:]
            if abs(sw) >= TAU - 1e-9:
                out.append(("circle", c, r))
            else:
                out.append(("arc3", polar(c, r, a0), polar(c, r, a0 + sw / 2.0), polar(c, r, a0 + sw), c, sw))
        else:
            out.append(("spline", _thin(p[1], 200), False))
    return out


def prim_polyline(p, steps=24):
    """Points for drawing a primitive as a polyline (preview)."""
    if p[0] == "seg":
        return [p[1], p[2]]
    if p[0] == "cspline":
        s = g.Spline(p[1], p[2])
        return [s.point(i / (2.0 * steps)) for i in range(2 * steps + 1)]
    if p[0] == "arc3":
        a, c, sw = p[1], p[4], p[5]
        r = dist(a, c)
        a0 = angle_of(sub(a, c))
        n = max(2, int(steps * abs(sw) / math.pi) + 1)
        return [polar(c, r, a0 + sw * i / n) for i in range(n + 1)]
    if p[0] == "circle":
        n = 2 * steps
        return [polar(p[1], p[2], TAU * i / n) for i in range(n + 1)]
    pts = list(p[1])
    return pts + [pts[0]] if p[2] else pts


# --------------------------------------------------------------------------- the grid
def build(areas, shape, dims, rib, border, vline=None, hline=None, origin=None, angle=0.0, rot90=False,
          vphase=CELL, hphase=CELL, min_partial=0.1, corner=0.0, corner_style="arc", max_cells=MAX_CELLS,
          sag=SAG):
    """Polygrid pockets filling the areas.

    areas: list of areas (each a list of loops of primitives, see the module doc).
    dims: pocket inradius (triangle, square, hexagon) or (length, width) for rectangles.
    vline, hline: symmetry lines (p0, p1) or None; see grid_frame. Or give origin and angle
    (the grid's symmetry centre and u-axis direction) directly. With neither, the grid is
    centred on the areas' bounding box.
    min_partial: cut pockets smaller than this fraction of a full pocket are dropped.
    corner, corner_style: radius of the pockets' corners (0 = sharp), as "arc", "g3" or "g4"
    curves. Cut pockets are rounded where they meet the edge too; their inward corners stay
    sharp. A G3/G4 corner that doesn't fit is made smaller or, failing that, left an arc.

    Returns dict(pockets=[{"loops": [[prims]], "edges": [[prims]], "area", "full"}],
    outlines=[[prims]] per boundary loop, areas (prepared), seeds=[a point inside each area], full, partial,
    dropped, pocket_area, region_area, cell_area, arc_corners, smaller_corners). "edges" leaves
    out the stretches on the area's own outline (only with no border), for drawing beside it."""
    if rib <= 0:
        raise ValueError("The rib thickness must be above zero.")
    if border < 0:
        raise ValueError("The border can't be negative.")
    if shape == RECTANGLE:
        if dims[0] <= 0 or dims[1] <= 0:
            raise ValueError("The cell length and width must be above zero.")
    elif dims <= 0:
        raise ValueError("The cell size must be above zero.")
    if corner < 0:
        raise ValueError("The corner radius can't be negative.")
    if corner > 0 and corner > max_corner(shape, dims, corner_style) * (1 + 1e-9):
        raise ValueError("With %s corners these pockets take a corner radius of at most %.4g cm."
                         % ("arc" if corner_style == "arc" else corner_style.upper(),
                            max_corner(shape, dims, corner_style)))
    prepared = [prepare_area(a, sag) for a in areas]
    loops = [lp for a in prepared for lp in a]
    polys = [loop_polyline(lp, sag) for lp in loops]
    region_area = sum(poly_area(p) for p in polys)
    x0, y0, x1, y1 = bbox([q for p in polys for q in p])
    if origin is None:
        origin, angle = grid_frame(((x0 + x1) / 2.0, (y0 + y1) / 2.0), vline, hline)
    ca, sa = math.cos(angle), math.sin(angle)

    def to_grid(p):
        d = sub(p, origin)
        return (ca * d[0] + sa * d[1], -sa * d[0] + ca * d[1])

    def to_sketch(q):
        return (origin[0] + ca * q[0] - sa * q[1], origin[1] + sa * q[0] + ca * q[1])

    gc = [to_grid(p) for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    bounds = bbox(gc)
    est = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1]) / lattice_cell_area(shape, dims, rib)
    if est > max_cells:
        raise ValueError("That would be about %d cells (the limit is %d). Use bigger cells." % (est, max_cells))

    eroded = erode(loops, border, sag)
    out = dict(pockets=[], outlines=[outline_prims(lp) for lp in loops], areas=prepared,
               seeds=[interior_point(a, sag) for a in prepared],
               full=0, partial=0, dropped=0, pocket_area=0.0, region_area=region_area, cell_area=0.0,
               arc_corners=0, smaller_corners=0)
    cells = [[to_sketch(q) for q in c] for c in lattice(shape, dims, rib, bounds, rot90, vphase, hphase)]
    if not cells:
        return out
    full_area = poly_area(cells[0])
    if corner > 0:
        full_area = _rounded_cell(cells[0], corner, corner_style)[1]
    out["cell_area"] = full_area
    if not eroded:
        return out

    def on_outline(prim_id):
        return border <= 0 and isinstance(prim_id, tuple) and len(prim_id) == 2 and isinstance(prim_id[0], int)

    def add_pocket(item_loops, area, full):
        out["pockets"].append({"loops": [[p for p, _ in it] for it in item_loops],
                               "edges": [[p for p, i in it if not on_outline(i)] for it in item_loops],
                               "area": area, "full": full})
        out["pocket_area"] += area
    ex0, ey0, ex1, ey1 = bbox([p for pts, _ in eroded for p in pts])
    region = _Region(eroded, max(math.sqrt(full_area), (ex1 - ex0 + ey1 - ey0) / 400.0, 1e-6))
    for cid, cell in enumerate(cells):
        cx0, cy0, cx1, cy1 = bbox(cell)
        if cx1 < ex0 or cx0 > ex1 or cy1 < ey0 or cy0 > ey1:
            continue
        loops_out, is_full = _clip_cell(cell, cid, region)
        if is_full:
            if corner > 0:
                add_pocket([[(p, None) for p in _rounded_cell(cell, corner, corner_style)[0]]], full_area, True)
            else:
                add_pocket([loop_items(*loops_out[0])], full_area, True)
            out["full"] += 1
            continue
        cleaned = [c for c in (_clean_loop(p, t) for p, t in loops_out) if c]
        if corner > 0 and cleaned:
            cleaned = [c for c in (_clean_loop(p, t) for p, t in round_corners(cleaned, corner, sag)) if c]
            if not cleaned:
                out["dropped"] += 1
                continue
        outers = [c for c in cleaned if poly_area(c[0]) > 0]
        holes = [c for c in cleaned if poly_area(c[0]) <= 0]
        groups = [[o] for o in outers]
        for h in holes:
            owners = [g for g in groups if point_in_poly(h[0][0], g[0][0])]
            if owners:
                min(owners, key=lambda g: poly_area(g[0][0])).append(h)
        for g in groups:
            a = sum(poly_area(c[0]) for c in g)
            if a < min_partial * full_area:
                out["dropped"] += 1
                continue
            items = [loop_items(*c) for c in g]
            if corner > 0 and corner_style != "arc":
                done = [blend_corners(it, corner, CORNER_COUNTS[corner_style]) for it in items]
                items = [d[0] for d in done]
                out["arc_corners"] += sum(d[1] for d in done)
                out["smaller_corners"] += sum(d[2] for d in done)
            add_pocket(items, a, False)
            out["partial"] += 1
    return out
