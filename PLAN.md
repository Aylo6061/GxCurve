# GX_tool — Plan

A Fusion add-in (Python) that creates G2 / G3 continuous curves in sketches, with a curvature comb you can turn on and off.

---

## 1. The core idea: continuity comes from control-point placement

A clamped B-spline's end behaviour depends only on its first few control points:

| Continuity to a straight leg | Condition at the curve end |
|---|---|
| G1 (tangent) | P0, P1 lie on the leg |
| G2 (curvature = 0, matches the line) | P0, P1, P2 lie on the leg |
| G3 (curvature rate = 0 too) | P0, P1, P2, P3 lie on the leg |

That makes 5 / 7 / 9 control points a natural set for a corner, where the middle point sits **on the corner** (shared by both legs):

| CPs | Layout (A = leg A, C = corner, B = leg B) | Degree | Result |
|---|---|---|---|
| **5** | A A **C** B B | 3 | G2 at both ends, C2 inside |
| **7** | A A A **C** B B B | 5 | G3 at both ends, C4 inside |
| **9** | A A A A **C** B B B B | 5 | G3 at both ends (G4 with suitable spacing), plus an extra point per side for shape control |

Fusion control-point splines only come in degree 3 or 5, which is why 5 CPs uses degree 3: degree 5 needs at least 6 points. The alternative is an exact degree-4 Bezier elevated to degree 5, which shows up as 6 CPs. That's open question 4.

**Shape parameters.** The corner point is fixed, so the free choices are:
- **Leg lengths:** how far along each leg the curve starts (like a fillet's size).
- **Spacing of the intermediate points** along each leg (as fractions of the leg length). This controls how "tight" or "flowing" the curve is. The UI shows one **Tension** slider that sets a good default spacing, and an optional **Advanced** section with the individual fractions.

---

## 2. Modes

1. **Corner (fillet-like).** Pick two sketch lines, or a corner point where two lines meet. The tool finds the intersection, even if the lines don't actually touch, and builds the curve into that corner.
   - **Symmetric:** one leg length and one set of spacing fractions, mirrored.
   - **Asymmetric:** separate leg lengths and spacing for leg A and leg B.
   - Option: **trim/extend the lines** back to the curve's start points and add coincident constraints, as Fusion's fillet does. Default: on.
2. **Freeform.** Pick a start point and an end point. If a picked point is the end of an existing line, the curve starts G2/G3 to that line. The tool builds a 5/7/9 CP spline with a sensible initial shape and hands you a **plain, unconstrained control-point spline**, so you can drag any control point afterwards however you like. No constraints or dimensions are added beyond coincident constraints at the picked endpoints.

Symmetric and asymmetric are sub-options of Corner mode. Freeform is a separate mode because it has no corner point and stays free after creation.

---

## 3. Curvature comb

- Drawn with **Fusion custom graphics**: temporary display lines that are not sketch geometry, so they don't clutter the timeline or constraints.
- Shows on the live preview while the command dialog is open (checkbox, scale slider, density slider).
- A separate **toggle button** in the toolbar shows or hides combs on GX curves after creation. It can also show them on any selected sketch curves (lines, arcs, splines), so you can check continuity across joints.
- GX curves are tagged with attributes (`GX_tool` group), so the toggle can find them again and restore settings.
- The comb draws both the teeth and the outer envelope line. Teeth point to the concave side and are scaled by a user factor.
- Fusion has its own curvature-comb display for splines in the UI, but I don't think it's in the API. The custom one also works on lines and arcs and can be styled.

---

## 4. Dragging and constraints

### While the command is open (Corner mode)
Each leg control point gets an **on-canvas drag handle**: a Fusion distance manipulator that slides along its leg. Dragging updates the preview and comb live.
- **Symmetric:** dragging a handle on one leg moves its mirror on the other leg too.
- **Asymmetric:** each handle moves on its own, but it stays on its leg, so the curve is always G2/G3.
- The corner control point stays locked to the corner.
- The dialog's length and spacing fields update as you drag, and dragging moves the handles when you type values. It's one set of values shown two ways.

### When you click OK (Corner mode): lock it down with real sketch constraints
The spline stays draggable in the sketch afterwards but keeps its continuity and symmetry:
1. **Virtual sharp:** construction lines run from each curve end to the corner, collinear with the original lines, the same way the fillet tool keeps the corner. If trimming is on, the removed part of each line becomes this construction line.
2. **Coincident:** the spline ends sit on the line ends, every leg-side control point sits on its construction line, and the middle control point sits on the corner. This is what locks in G2/G3.
3. **Dimensions:** distance from the corner to each leg control point. In symmetric mode that's one side only.
4. **Symmetric mode:** a construction bisector line through the corner, plus **symmetry constraints** pairing each control point with its mirror. Dragging one point in the sketch then moves its partner, and editing a dimension reshapes both sides.
5. **Asymmetric mode:** dimensions on both sides and no symmetry constraint.
6. Everything goes into a single undo step. The curve is tagged with attributes so the comb toggle (and a future edit command) can find it.

The aim is a **fully constrained** sketch that you can edit through the dimensions. If you'd rather drag points freely along the legs, there's a **"Dimension points"** checkbox. With it off, only the geometric constraints are added, so points slide along the legs but can't leave them.

### Freeform
No handles and no constraints: you get a plain control-point spline to drag freely in the sketch.

*(Needs checking in Fusion: whether constraints and dimensions can attach directly to a control-point spline's control points, so dragging in the sketch is solved by Fusion. I'm fairly confident, and it's the first thing milestone 3 checks.)*

---

## 5. Architecture

```
GX_tool/
  GX_tool.manifest            # add-in manifest
  GX_tool.py                  # run()/stop(): register commands, toolbar
  config.py
  commands/
    gx_curve/entry.py         # main command: inputs, preview, execute
    gx_comb_toggle/entry.py   # show/hide comb button
    resources/                # icons (16/32/64 px)
  lib/
    gx_math.py                # PURE PYTHON: CP layouts, B-spline eval,
                              # derivatives, curvature, comb samples
    fusion_geom.py            # read lines/intersections, make splines,
                              # trim, constraints, attributes
    comb_graphics.py          # custom-graphics comb drawing
    fusionAddInUtils/         # event-handler helpers (Autodesk template style)
  tests/
    test_gx_math.py           # runs outside Fusion with plain Python
  README.md
```

`gx_math.py` doesn't use the Fusion API, so the geometry can be **tested outside Fusion**. The tests check that curvature is about 0 at the ends (G2) and that its derivative is about 0 for G3. They also check the continuity inside the curve, symmetry in symmetric mode, and the degenerate cases.

**UI placement:** Sketch workspace → Create panel → "GX Curve", with the comb toggle next to it.

**Command dialog inputs:** Mode (Corner / Freeform) · Selection(s) · Symmetric / Asymmetric · CP count (5 / 7 / 9) · Leg length(s) · Tension · Advanced spacing (also driven by the canvas handles) · Trim lines ☑ · Dimension points ☑ · Show comb ☑ · Comb scale · Comb density.

**Checks:** reject parallel lines and leg lengths longer than the available line, and show a warning in the dialog.

---

## 6. Milestones

1. **Scaffold + math:** add-in skeleton that loads in Fusion, `gx_math.py`, and unit tests that pass locally.
2. **Corner mode:** symmetric and asymmetric, 5/7/9 CPs, live preview, spline creation.
3. **Handles, trim and constraints:** canvas drag handles (symmetric pairing), trim with virtual sharp, coincident, symmetry and dimensions on OK, attribute tagging.
4. **Curvature comb:** preview in the dialog plus the toolbar toggle.
5. **Freeform mode:** start/end picks, optional G2/G3 to picked lines, then a free spline.
6. **Polish:** icons, tooltips, input checks, undo as one step, README with install steps.

I can build and unit-test the math here, but I can't run Fusion. You'll need to load the add-in (Utilities → Add-Ins → **+** → pick this folder) and tell me what you see at each milestone.

---

## 7. Open questions

**Decisions (round 2):**
- Trim is a checkbox, on by default.
- 5 CPs: a per-curve choice between cubic (5 points) and quintic (6 points). See the README for the difference.
- The 9-CP behaviour is chosen per curve with an **Apex** option. Correction to section 1: a 9-point layout with the apex on the corner is G4 automatically, whatever the spacing. So the real choice is apex **locked** (G4) or **free** (G3, and you can pull the apex toward or away from the corner). 7 CPs get the same option: locked gives G3, free gives G2.
- Freeform: G2/G3 when you pick a line's endpoint, otherwise the end is free.

Answered (round 1):
- Freeform: a free spline whose control points you drag after creation.
- Corner modes: draggable, but kept symmetric/constrained, with constraints and dimensions added when the tool exits.

Still open (these defaults apply unless you say otherwise):
1. **Trim the corner lines like the fillet tool does:** yes, and the removed part is kept as a construction virtual sharp.
2. **5 CPs:** degree-3 spline with exactly 5 points.
3. **9 CPs:** the extra points give more shape control at G3; they aren't locked for G4.
4. **Freeform ends:** G2/G3 to a line only when you pick a line's endpoint. Otherwise the ends are free.
