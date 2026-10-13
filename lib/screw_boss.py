"""Screw boss sizes and checks, pure Python: the hole for a tapped thread, a thread-forming screw or
a heat-set insert (M2 to M10), the boss's outside diameter, and where the bosses go.

Lengths here are in mm unless a function says otherwise; the command converts (Fusion works in cm).
"""

import math

TAP, SELF_TAP, INSERT, CUSTOM = "tap", "self_tap", "insert", "custom"
WALL, OD = "wall", "od"  # size the boss by its wall thickness or its outside diameter

SIZES = ("M2", "M2.5", "M3", "M4", "M5", "M6", "M8", "M10")  # ISO 261 first-choice sizes
NOMINAL = {"M2": 2.0, "M2.5": 2.5, "M3": 3.0, "M4": 4.0, "M5": 5.0, "M6": 6.0, "M8": 8.0, "M10": 10.0}
PITCH = {"M2": 0.4, "M2.5": 0.45, "M3": 0.5, "M4": 0.7, "M5": 0.8, "M6": 1.0, "M8": 1.25, "M10": 1.5}  # ISO coarse

# Tap drill for an ISO coarse thread (ISO 2306): the nominal size less the pitch, to a standard drill
# (M8: 6.8 rather than 6.75).
TAP_DRILL = {"M2": 1.6, "M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2, "M6": 5.0, "M8": 6.8, "M10": 8.5}

# Thread-forming screws for plastic (EJOT PT / DELTA PT design guides): a hole of 0.7 to 0.8 d
# depending on the plastic, 0.8 d being the usual choice, in a boss of about 2 d.
SELF_TAP_HOLE, SELF_TAP_BOSS = 0.8, 2.0

# McMaster-Carr 94459A brass heat-set inserts for plastic (straight, standard profile), per size:
# (part number, installed length, drill McMaster lists, its diameter, McMaster's largest hole).
# The hole is drilled with the listed drill.
_INSERTS = {
    "M2": [("94459A110", 2.5, "#30", 3.26, 3.3), ("94459A767", 3.2, "3.2 mm", 3.2, 3.2),
           ("94459A120", 4.0, "#30", 3.26, 3.3)],
    "M2.5": [("94459A768", 4.0, "#22", 3.99, 4.0), ("94459A439", 5.9, "#20", 4.09, 4.1)],
    "M3": [("94459A769", 3.4, "#22", 3.99, 4.0), ("94459A130", 4.3, "#22", 3.99, 4.0),
           ("94459A140", 5.7, "#22", 3.99, 4.0), ("94459A771", 6.4, '3/16"', 4.76, 4.8)],
    "M4": [("94459A150", 4.7, '7/32"', 5.56, 5.6), ("94459A160", 6.4, '7/32"', 5.56, 5.6),
           ("94459A170", 8.2, '7/32"', 5.56, 5.6)],
    "M5": [("94459A774", 5.7, "6.4 mm", 6.4, 6.4), ("94459A775", 7.9, "6.4 mm", 6.4, 6.4),
           ("94459A180", 9.5, "6.5 mm", 6.5, 6.5)],
    "M6": [("94459A776", 6.4, "8 mm", 8.0, 8.0), ("94459A190", 7.9, "8 mm", 8.0, 8.0),
           ("94459A210", 9.5, "8 mm", 8.0, 8.0), ("94459A220", 12.7, "8 mm", 8.0, 8.0)],
    "M8": [("94459A777", 6.4, "V", 9.58, 9.6), ("94459A778", 12.7, "V", 9.58, 9.6)],
    "M10": [("94459A779", 12.7, '15/32"', 11.9, 11.9)],  # the drill is 11.906 mm: kept to McMaster's largest hole
}
INSERTS = {s: [dict(part=p, length=L, drill=dr, hole=h, max_hole=mx) for p, L, dr, h, mx in rows]
           for s, rows in _INSERTS.items()}
# the one picked for each size until you choose another: about 1.6 to 2 d long
TYPICAL = {"M2": "94459A120", "M2.5": "94459A768", "M3": "94459A140", "M4": "94459A160", "M5": "94459A775",
           "M6": "94459A210", "M8": "94459A778", "M10": "94459A779"}


def insert(size, part=None):
    """The insert of this size with this part number (or the typical one)."""
    rows = INSERTS[size]
    return next((r for r in rows if r["part"] == part), None) or next(r for r in rows if r["part"] == TYPICAL[size])


def insert_depth(size, part=None):
    """Hole depth for an insert: its length, plus room below it for the plastic it pushes down
    (two thread pitches, as Spirol recommends, and at least 1 mm)."""
    return round(insert(size, part)["length"] + max(1.0, 2 * PITCH[size]), 2)


def preset_hole(kind, size, part=None):
    """The hole diameter for a preset, or None for CUSTOM."""
    if kind == TAP:
        return TAP_DRILL[size]
    if kind == SELF_TAP:
        return round(SELF_TAP_HOLE * NOMINAL[size], 2)
    if kind == INSERT:
        return insert(size, part)["hole"]
    return None


def suggested_od(kind, size, hole):
    """The smallest boss outside diameter the guidelines suggest, or None: 2 d for a thread-forming
    screw, twice the hole for an insert."""
    if kind == SELF_TAP:
        return SELF_TAP_BOSS * NOMINAL[size]
    if kind == INSERT:
        return 2.0 * hole
    return None


def describe(kind, size, part=None):
    """What the hole is for, in words."""
    if kind == TAP:
        return "tap drill for %s × %g" % (size, PITCH[size])
    if kind == SELF_TAP:
        return "for an %s thread-forming screw (%g × d)" % (size, SELF_TAP_HOLE)
    if kind == INSERT:
        r = insert(size, part)
        return "for McMaster %s (%s insert, %g mm long; drill %s)" % (r["part"], size, r["length"], r["drill"])
    return "custom diameter"


def boss_od(hole, by, value):
    """The boss's outside diameter, from its wall thickness or given outright (any one unit)."""
    return hole + 2 * value if by == WALL else value


# --------------------------------------------------------------------------- where the bosses go
def merge_points(pts, tol=1e-6):
    """Indices of pts to keep, dropping any at the same place as an earlier one (2D points)."""
    keep = []
    for i, p in enumerate(pts):
        if all(math.dist(p, pts[j]) > tol for j in keep):
            keep.append(i)
    return keep


def overlapping(pts, od, tol=1e-6):
    """Pairs (i, j) of bosses of diameter od at 2D points pts that overlap."""
    return [(i, j) for i in range(len(pts)) for j in range(i + 1, len(pts)) if math.dist(pts[i], pts[j]) < od - tol]


def group_levels(levels, tol=1e-5):
    """Indices of levels grouped by value (within tol), lowest first."""
    order = sorted(range(len(levels)), key=lambda k: levels[k])
    groups = []
    for k in order:
        if groups and levels[k] - levels[groups[-1][0]] <= tol:
            groups[-1].append(k)
        else:
            groups.append([k])
    return groups
