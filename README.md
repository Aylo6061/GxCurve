# GX_tool

Fusion add-in for G2 / G3 / G4 continuous curves in sketches, with live curvature combs.

## Install

Fusion → **Utilities → Add-Ins** (Shift+S) → **Add-Ins** tab → **+** → pick this `GX_tool` folder → **Run**.
Tick *Run on Startup* if you want it loaded every time.

The two commands appear in the **Sketch → Create** menu: **GX Curve** and **GX Curvature Comb**.

## GX Curve

### Corner mode (works like fillet)
1. Pick **Line A** and **Line B**. Click each line on the side you want to keep. The lines don't need to touch.
2. Choose **Control points** and **Symmetry**, then shape the curve:
   - Drag the **arrows on the canvas**. Each one slides a control point along its line, so the continuity can't break.
   - **Tension** evens out or bunches up the inner points. **Leg length** (the outer arrow) scales the whole curve when *Inner points follow leg length* is on.
3. Click **OK**. The curve gets locked with sketch constraints:
   - Every leg control point sits on its line, and the middle point sits on the corner.
   - **Symmetric:** a construction bisector with symmetry constraints between point pairs, so dragging a point on one leg moves its mirror on the other.
   - **Dimension points** (on by default): distance dimensions from the corner, so you edit the curve by changing numbers. Turn it off to drag points along the legs instead.
   - **Trim lines** (on by default): each original line becomes a construction line to the corner (the "virtual sharp"), and a new solid segment runs to the curve end.

| Control points | Layout (A = on leg A, C = on corner, X = free apex) | Degree | Continuity to the lines |
|---|---|---|---|
| 5, cubic | A A C B B | 3 | G2 |
| 5, quintic | A A A B B B (6 points) | 5 | G2 |
| 7, apex locked | A A A C B B B | 5 | **G3** |
| 7, apex free | A A A X B B B | 5 | G2, with an apex you can pull |
| 9, apex locked | A A A A C B B B B | 5 | **G4** |
| 9, apex free | A A A A X B B B B | 5 | **G3**, with an apex you can pull |

**5-point cubic vs quintic:**
- *Cubic* is exactly 5 points, but degree 3 means it's two cubic pieces joined in the middle. Curvature is continuous there, but the rate of change of curvature jumps, which shows up as a small corner in the comb's outline at the midpoint.
- *Quintic* is a single degree-5 piece with a perfectly smooth comb. It needs 6 points, and none of them sits on the corner.

Both are G2 to the lines.

### Freeform mode
Pick a **start point** and an **end point**, which can be the ends of existing lines. With *Continue picked lines* on, the curve starts G2/G3 from any line ending at a picked point. The result is a plain control-point spline, joined to the two points only, so you can drag its control points freely. Dragging can break the G2/G3 at the ends.

## GX Curvature Comb
Pick the sketch curves that should show a comb (lines, arcs and splines all work), then set the scale and density. Clear the selection to remove all combs, or untick **Show combs** to hide them without forgetting the selection. Combs are redrawn while you drag control points or edit dimensions. All combs share one scale, so a curvature jump where two curves meet shows up clearly.

Combs are display-only custom graphics. They're not saved in the file, but the list of combed curves is, and the combs come back when the add-in loads.

## Development
```
python -m unittest discover tests      # math + constraint bookkeeping, no Fusion needed
python tools/make_icons.py             # regenerate toolbar icons
```
- `lib/gx_math.py`: pure-Python layouts, B-spline evaluation, curvature and its rate
- `lib/fusion_geom.py`: selection analysis, spline creation, constraints
- `lib/comb_graphics.py`: comb drawing
- `commands/`: the two commands

Messages, errors (with full tracebacks) and constraint failures are logged to:
- `logs/gx_tool.log` in this folder, with timestamps. At 1 MB it's moved to `gx_tool.log.1` and a new log starts.
- Fusion's **Text Commands** window, prefixed `[GX_tool]`.
