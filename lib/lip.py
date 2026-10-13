"""Lip geometry, pure Python: the cross-sections of a lip and of the groove it fits into, the reveal
(gap) on each side, and putting picked edges in order as sweep paths.

Cross-sections are drawn in a frame across the parting line, at a point on a picked edge:
  u: across the parting face, from the picked edge into the face (into the wall)
  v: up from the parting face, out of the lip body and into the groove body
The lip stands on the parting face; its "near" side faces the picked edge, its "far" side faces away.
"""

import math

NONE, INNER, OUTER, BOTH = "none", "inner", "outer", "both"


# --------------------------------------------------------------------------- 3D vectors
def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    return mul(a, 1.0 / norm(a))


def frame_map(P, w, n):
    """Maps a cross-section point (u, v) to model space: P + u w + v n."""
    return lambda q: add(P, add(mul(w, q[0]), mul(n, q[1])))


# --------------------------------------------------------------------------- cross-sections
def gaps(reveal, gap, inner_far):
    """(near, far): the reveal on the lip's side toward the picked edge, and on its other side.
    inner_far: the far side is the inner one (the picked edge is on the parting face's outer
    boundary, so the face runs inward from it)."""
    inner = gap if reveal in (INNER, BOTH) else 0.0
    outer = gap if reveal in (OUTER, BOTH) else 0.0
    return (outer, inner) if inner_far else (inner, outer)


def lip_section(offset, width, height, draft):
    """The lip's corners, counterclockwise: Lip width wide on the parting face, offset from the
    edge, Lip height tall, each side leaning in by the draft angle (radians)."""
    k = height * math.tan(draft)
    return [(offset, 0.0), (offset + width, 0.0), (offset + width - k, height), (offset + k, height)]


def groove_section(offset, width, height, draft, near, far, top, below):
    """The groove's corners, counterclockwise: the lip grown by the reveals near and far
    (measured square to its drafted sides) and by top above it. It reaches below down past the
    parting face, so the cut doesn't leave a skin where the faces coincide; only the groove body
    is cut, which isn't there."""
    t, s = math.tan(draft), 1.0 / math.cos(draft)
    u0, u1 = offset - near * s, offset + width + far * s  # where the sides cross the parting face
    h = height + top
    return [(u0 - below * t, -below), (u1 + below * t, -below), (u1 - h * t, h), (u0 + h * t, h)]


def top_width(section):
    return section[2][0] - section[3][0]


def errors(offset, width, height, draft, gap, top):
    """What's wrong with these sizes (lengths in cm, the draft in radians), as messages."""
    errs = []
    if height <= 0:
        errs.append("The lip height must be above zero.")
    if width <= 0:
        errs.append("The lip width must be above zero.")
    if offset < 0:
        errs.append("The offset from the edge can't be negative.")
    if gap < 0 or top < 0:
        errs.append("The gaps can't be negative.")
    if not 0 <= draft < math.radians(45):
        errs.append("The draft angle must be from 0 to 45°.")
    if errs:
        return errs
    w = top_width(lip_section(offset, width, height, draft))
    if w <= 1e-6:
        errs.append("The draft is too steep for this lip: its sides would meet below its top. Make it "
                    "wider or lower, or use less draft (at most %.2f°)."
                    % math.degrees(math.atan(0.5 * width / height)))
    return errs


def describe(offset, width, height, draft, near, far, top):
    """(lip top width, groove width at the parting face, groove depth)."""
    lip = lip_section(offset, width, height, draft)
    groove = groove_section(offset, width, height, draft, near, far, top, 0.0)
    return top_width(lip), groove[1][0] - groove[0][0], height + top


# --------------------------------------------------------------------------- sweep paths
def chains(ends, tol=1e-6):
    """Put edges in order, end to end. ends: (start point, end point) of each edge. Returns chains
    as dicts: edges = [(edge index, reversed)] in order, closed = True for a loop. An open chain
    starts at one of its free ends. Raises ValueError where three or more edges meet."""
    verts = []

    def vid(p):
        for k, q in enumerate(verts):
            if math.dist(p, q) <= tol:
                return k
        verts.append(p)
        return len(verts) - 1
    ev = [(vid(a), vid(b)) for a, b in ends]
    at, deg = {}, {}
    for i, (a, b) in enumerate(ev):
        for v in {a, b}:
            at.setdefault(v, []).append(i)
        deg[a] = deg.get(a, 0) + 1
        deg[b] = deg.get(b, 0) + 1
    if any(d > 2 for d in deg.values()):
        raise ValueError("Three or more of the edges meet at one point. Pick edges that run end to end, "
                         "one loop or chain at a time.")
    used = set()

    def walk(i, v):
        order = []
        while i is not None and i not in used:
            used.add(i)
            a, b = ev[i]
            rev = b == v and a != v
            order.append((i, rev))
            v = a if rev else b
            i = next((j for j in at[v] if j not in used), None)
        return order

    out = []
    for v in sorted(v for v, d in deg.items() if d == 1):
        if at[v][0] not in used:
            out.append(dict(edges=walk(at[v][0], v), closed=False))
    for i in range(len(ev)):
        if i not in used:
            out.append(dict(edges=walk(i, ev[i][0]), closed=True))
    return out
