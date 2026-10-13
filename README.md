# GX_tool

Fusion add-in for G2 / G3 / G4 continuous curves in sketches, with live curvature combs.

## Install

Fusion → **Utilities → Add-Ins** (Shift+S) → **Add-Ins** tab → **+** → pick this `GX_tool` folder → **Run**.
Tick *Run on Startup* if you want it loaded every time.

These commands appear in the **Sketch → Create** menu: **GX Curve**, **GX G3 Fillet**, **GX Smooth**, **GX Curvature Comb** and **Sketch Polygrid**. **GX Edge Fillet** and **Polygrid** are in **Solid → Modify**, and **Screw Boss** and **Lip** are in **Solid → Create**.

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

## GX G3 Fillet
Works like Sketch → Fillet: pick two lines and set a **Radius**, either by typing it or by dragging the arrow at the corner. Instead of an arc, you get a **G3** curve (7 points) or a **G4** curve (9 points) that stands in for that fillet. A green **true fillet arc** is drawn alongside for comparison.

A G3 curve can't follow an arc exactly. It has to ease in from zero curvature where it meets the line, so it starts a bit further out along each line and bends a bit tighter than R in the middle. The dialog shows the largest gap from the true fillet, the leg length, and the tightest radius.

- **Default: smoothest shape.** The points are spaced to minimise the comb's *acceleration*, ∫(d²κ/ds²)² ds, so curvature eases in gently from each line instead of ramping straight up. Minimising the comb's slope instead gives a clothoid-like ramp with a sharp knee right next to each join. At 90° this shape stays within about 1.3% of R of the true arc, with legs about 1.45× the fillet's tangent distance.
- **Emulate fillet** (checkbox): the points are spaced to hug the true arc as closely as possible. At 90°, a 7-point curve stays within about 0.3% of R. Its legs are 1.10× the fillet's tangent distance, and its tightest radius is about 0.9 R. Note that the 7-point emulated comb ramps up fast and has a small bump; the 9-point one doesn't.

Sharper corners leave a bigger gap either way; at 30° it's about 12% of R.

**Match** picks the size:

| Option | Meaning |
|---|---|
| Best fit to the fillet | Smallest largest gap (default) |
| Same depth into the corner | Passes through the arc's midpoint |
| Same tangent points | Starts where the arc would |
| Never tighter than R | Tightest radius = R, for example so a cutter of radius R fits |

With two lines, the result is a symmetric GX corner curve, with the same trim, constraints and dimensions as GX Curve.

**Curves and conics.** Either pick can also be an arc, circle, ellipse, elliptical arc, conic curve (parabola/hyperbola) or spline. The curves have to cross, and you click each one on the side to keep. The blend is then **G3 to that curve's own curvature**: at each contact point it matches the curve's position, tangent, curvature and rate of change of curvature. The tool:
1. Finds the radius-R arc tangent to both curves.
2. Moves the contact points out along each curve by the same proportion the straight-line fillet uses for that angle.
3. Optimises the shape: smoothest comb by default, or closest to the arc with Emulate fillet.

**Trim** splits the curves at the contact points and turns the corner pieces into construction geometry. Sketch constraints can't hold G3 to a curve, so the blend is **fixed**. It gets coincident constraints, plus a G2 "smooth" constraint where Fusion accepts one. With curves, 9 points gives G3 plus extra freedom, not G4.

## GX Edge Fillet (Solid → Modify)
This is the solid-body version of GX G3 Fillet. Pick straight edges between flat faces, then set Radius, Curve (G3/G4), Match and Emulate fillet as above. For each edge, the add-in:
1. Makes a plane across the edge and sketches the GX profile there.
2. Extrudes the profile along the edge: a **cut** for an outside edge, a **join** for an inside edge.

These steps are grouped in the timeline as "GX Edge Fillet", and the helper planes and sketches are hidden.

**Curved faces.** A face can also be curved, as long as its cross-section is the same all along the edge: cylinders, elliptical cylinders, and extruded ellipses/parabolas/hyperbolas. The add-in reads each face's cross-section, makes the blend G3 to that face's own curvature, and draws the face side of the profile as an exact conic arc, so the cut follows the face precisely.

**Limits for now:**
- Edges must be straight.
- Cones, spheres and tori aren't supported, because their cross-section changes along the edge.
- Where several filleted edges meet at a corner, the cuts overlap instead of blending into a smooth corner patch.
- The legs have to fit on the faces.

For curved edges or blended corners, Fusion's own Fillet with Continuity set to **Curvature** gives G2.

## GX Smooth
Pick any control-point spline, including GX curves. **Amount** controls how far it moves toward the smoothest shape.

- **Bending energy (fast):** minimises ∫|C″|², roughly the curve's acceleration squared. **Tension** blends in ∫|C′|² (velocity), which makes the curve shorter and tauter.
- **Curvature flow (exact, slower):** starts from the bending result, then directly smooths the comb by minimising its acceleration, ∫(d²κ/ds²)² ds.
- **Keep end continuity:** the ends stay fixed, points on each end's line only slide along it (so G2/G3/G4 to the lines is kept), and a corner apex stays fixed.

The new shape goes in through the sketch. Dimensioned points get new dimension values, and other points are dragged, so existing constraints are respected. The dialog shows the comb roughness before → after.

## Polygrid (Solid → Modify)
Cuts or extrudes a regular grid of pockets separated by ribs over a sketch area, for lightening pockets, honeycomb or isogrid panels, or rib webs. Like GX Edge Fillet, it's made of ordinary features, grouped in the timeline as **Polygrid**:
- **Polygrid sketch**: the grid, on the area's plane.
- **Polygrid**: one Extrude.
- With a bottom blend: a Fillet, or the G3/G4 rings (a plane, a profile sketch and a sweep for each distinct pocket outline, a base feature with the copies, and a Combine).

1. Pick the **Area**: one or more closed regions of a sketch, *or* the closed outline curves around the area (in any order; loops inside loops become holes). Holes in the area are kept clear, with a border around them.
2. Choose what to make:
   - **Operation**:
     - *Cut holes* cuts the pockets out.
     - *Cut ribs* cuts the rib web away.
     - *Extrude holes* or *Extrude ribs* adds them as solid, either joined to a body or as a new body (**Extrude as**).
   - **Bodies**: the bodies to cut or join.
   - **Extent**:
     - *Distance*: a depth, with **Flip direction**.
     - *Through all*.
     - *To face*: up to a face you pick, with an **Offset** from it. The offset's sign decides whether it stops short of the face (leaving a floor) or goes past it; check the preview.
   - **Bottom**: rounds where the walls meet the floor of the cut, or where what's extruded meets the body below. There's no bottom for a cut that goes through all, or for an extrude made as a new body.
     - *Radius* or *Curvature (G2)*: Fusion's Fillet on those edges, tangent or curvature continuous.
     - *G3 curve* / *G4 curve*: Fusion's Fillet stops at G2, so these are built. The GX corner curve between wall and floor is swept around a pocket outline as a ring, and the rings are joined to the bodies with Combine.
       - Outlines that are copies of each other share one sweep, and the ring is copied to the rest: every whole pocket, and the cut pockets along a straight edge. So a big grid needs a sweep only for each *distinct* outline; the dialog says how many.
       - If an outline's sweep fails, it gets the G2 fillet instead, and you're told how many did. Sweeps are most reliable when the pocket corners are rounder than the blend is tall, and the dialog warns when they aren't.
       - The blend goes into the pockets when you cut holes or extrude ribs, and around them when you cut ribs or extrude holes. Extruded shapes are blended only around the pockets' outer outlines, not along the area's own edge.
     - **Bottom radius** is the fillet's radius, or the radius the G3/G4 curve stands in for. A G3/G4 curve reaches about 1.45× that up the wall, and it has to fit within the depth.
3. Set up the grid (the same settings as Sketch Polygrid, below). Tick **Preview solid** to see the real cut while you work. It's slower on big grids; otherwise you see the pocket outlines. Swept G3/G4 bottoms aren't previewed.
4. Click **OK**.

### Editing a Polygrid
Do one of these:
- Select the Polygrid (its timeline group, its extrude or its sketch) and run **Polygrid**.
- Right-click it and choose **Edit Polygrid**.

While the dialog is open, the timeline rolls back to just before the Polygrid. **OK** redraws its sketch, points the same Extrude at the new profiles and rebuilds the bottom blend. Features after it keep their references to the Extrude wherever Fusion can resolve them; references to the bottom blend's faces are lost, because it's rebuilt.
- The dialog reads the Extrude's own depth, offset, bodies and face, so changes made in Fusion's Extrude dialog are kept.
- The depth and offset can be user parameters. You can also change the depth in the Extrude's own parameters, without the dialog.
- Like GX Edge Fillet, a Polygrid doesn't refit by itself when its area changes. Edit it and click **OK** to refit.

## Sketch Polygrid (Sketch → Create)
Draws the same grid into a sketch of its own, **Sketch Polygrid**, on the area's plane, for when you want to do your own modelling with it. The area's own sketch isn't changed.

1. Pick the **Area**: sketch regions or closed outline curves, as for Polygrid.
2. Set up the grid:
   - **Make**: *Cutouts* draws just the pocket outlines, so you can window-select them all and use Extrude → Cut. *Ribs* also draws the area's outline, so the web of ribs between the pockets is a single profile to extrude.
   - **Cell shape**: triangle, square, hexagon or rectangle. **Rotate 90°** turns triangles and hexagons a quarter turn (for hexagons, that's flat top ↔ pointed top).
   - **Size by**: a pocket's **side length** or **inscribed circle diameter**. Rectangles take a **cell length** and **width** instead. The info line shows the other measure and the pitch (pocket + rib).
   - **Rib thickness**, and **Border**: the rib along the area's edge and around its holes (0 for none). Border follows the rib thickness until you change it.
   - **Corner radius** and **Corner style** round the pockets' corners (0 for sharp):
     - *Arc*: a plain radius, up to the pocket's inscribed radius (which makes round pockets).
     - *G3 curve (7 points)* / *G4 curve (9 points)*: a curvature-continuous curve standing in for that radius, the same shape GX G3 Fillet makes by default (smoothest comb, best fit to the arc). Its legs run further along the edges than an arc's (about 1.45x at 90 degrees), so the largest radius is smaller. Each corner is a control-point spline.

     Pockets cut by the edge are rounded where they meet it too. Where the edge is an arc, the G3 curve matches its curvature; it uses the straight-corner layout carried onto the arc rather than GX G3 Fillet's per-corner optimisation, which takes seconds per corner. Corners that point into a pocket (an L-shaped area's inside corner, with no border) stay sharp. Where a cut pocket is too tight for a G3/G4 corner, the corner is made smaller or left as an arc, and the dialog says how many. Corners next to spline or ellipse edges also stay arcs.
   - **Smallest cut pocket %**: pockets cut by the edge that are smaller than this share of a whole pocket are left out, so you don't get slivers.
   - **Symmetry** (optional): a **vertical line** and/or a **horizontal line** (sketch lines or construction axes). The pattern is mirrored about them. Without lines, the axes cross at the centre of the area, so a symmetric area gets a symmetric grid. Tilted lines tilt the grid to match; if you pick both, they must be perpendicular.
   - **On the vertical/horizontal axis**: whether that mirror axis runs through the middle of a row of pockets or along a rib, where the shape allows both:

     | Shape | Vertical axis | Horizontal axis |
     |---|---|---|
     | Square, rectangle | pocket centres or rib | pocket centres or rib |
     | Hexagon (flat top) | pocket centres | pocket centres or rib |
     | Triangle | pocket centres (and rib crossings) | rib |

     **Rotate 90°** swaps the two columns.
3. Click **OK**. Every curve in the grid sketch is fixed.

Where pockets meet the border, their edges follow the area's shape: lines stay lines, arcs become concentric arcs, inside corners get a border-radius arc, and splines or ellipses become fitted splines.

### Editing a Sketch Polygrid
The settings are stored on its sketch. To change them, do one of these:
- Select the sketch, or any of its curves, and run **Sketch Polygrid**.
- Right-click it and choose **Edit Sketch Polygrid**.
- Pick it in the **Area** box while the command is open.

The dialog opens with the stored settings, and **OK** redraws the sketch in place.
- The value boxes take expressions and user parameters (for example `wall * 2`). They're stored as you typed them, so after changing a user parameter, Edit → OK picks up the new value.
- The area is found again from a point stored inside each region, or from the outline curves. If you've reshaped the area's sketch since, Edit → OK fits the grid to the new shape. If the area can't be found, the dialog asks you to select it again.
- Redrawing replaces the sketch's curves. Features built on its profiles lose them, so reselect the profiles afterwards, or use **Polygrid**, which does that for you. Don't draw your own geometry in it, because editing clears it.

## Screw Boss (Solid → Create)
Puts a boss for a screw at each picked sketch point on a flat face, with a hole down its middle. Like Polygrid, it's made of ordinary features, grouped in the timeline as **Screw Boss**:
- **Screw boss sketch**: a circle round each point, on the points' sketch plane. The circles are centred on linked projections of the points, so moving a point moves its boss. One circle has a diameter dimension and the others are equal to it, so editing that one dimension resizes every boss.
- **Screw boss**: one Extrude of the circles.
- **Screw boss top** and **Screw boss hole**: a sketch on the bosses' top face, and a Hole feature going down from it. Bosses that end at different heights get one of each per height.

Afterwards you can change it all in the Screw Boss dialog (see *Editing a Screw Boss* below), or each part in Fusion's own dialogs: the height in the Extrude, and the hole's diameter, depth and tip in the Hole.

1. Pick the **Points**: sketch points, all in one sketch, drawn on the flat face the bosses stand on (or on a plane).
2. Choose the hole in **Hole for**:

   | Hole for | Hole Ø, M2 / M2.5 / M3 / M4 / M5 / M6 / M8 / M10 | Where it comes from |
   |---|---|---|
   | Tap drill | 1.6 / 2.05 / 2.5 / 3.3 / 4.2 / 5.0 / 6.8 / 8.5 mm | ISO coarse thread (ISO 2306): the size less the pitch, to a standard drill |
   | Self-tapping screw | 1.6 / 2.0 / 2.4 / 3.2 / 4.0 / 4.8 / 6.4 / 8.0 mm | 0.8 × d, from EJOT's guides for thread-forming screws in plastic (0.7 to 0.8 d, depending on the plastic) |
   | Heat-set insert | see below | McMaster-Carr 94459A brass inserts for plastic |
   | Custom diameter | your own | |

   - **Heat-set insert**: pick the **Insert**. Each size lists McMaster's straight brass inserts by length, with a typical one picked first. The hole Ø is the drill McMaster lists for it. Picking an insert sets **Hole depth** to its length plus room for the plastic it pushes down: two thread pitches, at least 1 mm.

     | Size | Picked first | Length | Hole Ø (drill) | Hole depth |
     |---|---|---|---|---|
     | M2 | 94459A120 | 4.0 mm | 3.26 mm (#30) | 5.0 mm |
     | M2.5 | 94459A768 | 4.0 mm | 3.99 mm (#22) | 5.0 mm |
     | M3 | 94459A140 | 5.7 mm | 3.99 mm (#22) | 6.7 mm |
     | M4 | 94459A160 | 6.4 mm | 5.56 mm (7/32") | 7.8 mm |
     | M5 | 94459A775 | 7.9 mm | 6.4 mm | 9.5 mm |
     | M6 | 94459A210 | 9.5 mm | 8.0 mm | 11.5 mm |
     | M8 | 94459A778 | 12.7 mm | 9.58 mm (V) | 15.2 mm |
     | M10 | 94459A779 | 12.7 mm | 11.9 mm (15/32") | 15.7 mm |
   - To adjust a preset's diameter (for example, to allow for how your printer makes holes), switch to **Custom diameter**. It starts from the preset's value.
   - **Hole depth** is measured down from the top of the boss.
   - **Hole bottom**: *Drill point* (118°) or *Flat*. A flat bottom is a 180° tip angle on the Hole; if Fusion won't take that, the hole is made as an extruded cut instead.
3. Size the boss by **Wall thickness** (from the hole to the outside) or **Outer diameter**. Switching between them keeps the boss the same size.
4. Choose how tall:
   - **Extent**:
     - *Distance*: a **Height**, with **Flip direction**.
     - *To object*: up to a face, construction plane or body, with an **Offset**. The offset's sign decides whether it stops short or goes past; check the preview. The bosses' tops must come out flat and parallel to the sketch, so the hole starts square to them.
   - **Boss as**: *Join* to the body the bosses stand on, or a *New body*. Joined bosses that run into the body they're joined to have no top for the hole; leave a gap with the offset.
5. Click **OK**.

The dialog warns you, without stopping you, when:
- the hole is shorter than the insert;
- the hole reaches below the boss;
- the boss is thinner than the guideline (2 d for thread-forming screws, twice the hole for inserts);
- bosses overlap.

The settings you used last come back next time.

### Editing a Screw Boss
Do one of these:
- Select the Screw Boss (its timeline group, or any of its sketches or features) and run **Screw Boss**.
- Right-click it and choose **Edit Screw Boss**.

While the dialog is open, the timeline rolls back to just before the Screw Boss.
- The dialog opens with the settings the Screw Boss was made with, and reads back anything you've changed since in Fusion's own dialogs:
  - the Extrude's height, or its object and offset;
  - the boss sketch's diameter dimension (shown as *Outer diameter*);
  - the Hole's diameter (shown as *Custom diameter*), depth and drill point.
- You can add or remove points. They have to be in the plane of the Screw Boss's sketch.
- **OK** redraws the boss sketch and points the same Extrude at it, so features after it keep their references to the bosses wherever Fusion can resolve them. The top sketches and holes are made again, so references to the old holes' faces are lost.
- **Cancel** leaves it as it was.

## Lip (Solid → Create)
Makes a lip-and-groove joint where two bodies meet, such as the two halves of an enclosure: a lip standing on one body's parting face, and the groove it fits into, cut from the other body. Like Polygrid, it's made of ordinary features, grouped in the timeline as **Lip**:
- **Lip plane**, with the **Lip profile** and **Lip groove profile** sketches: the two cross-sections, on a plane across the edges.
- **Lip** and **Lip groove** sweeps: each cross-section swept along the edges, as a new body.
- **Lip groove** and **Lip** Combines: the groove is cut from the groove body, and the lip is joined to the lip body. Combine only changes the body it's given, so the lip can't merge the two bodies where it touches the groove.

1. Pick the **Edges** along the parting line. These are where the parting face (the flat face where the two bodies meet) runs into a wall, and they can be on either body. With **Whole loop** on (the default), one edge picks the whole loop of the parting face it's on.

   Picking an edge also fills in empty body boxes: its body becomes the **Lip body**, and the body sitting on its parting face becomes the **Groove body**. To swap them, pick them yourself.
2. Shape the lip:
   - **Lip height**, and **Lip width** where it stands on the parting face.
   - **Offset from edge**: how far across the parting face from the picked edges the lip starts. 0 puts it flush with that wall. For the usual enclosure lip on the inside of the wall, pick the inner edges and leave it at 0. For a lip in the middle of the wall, pick either side's edges and offset it.
   - **Draft angle**: both sides of the lip lean in by this much, and the groove's sides lean with them.
3. Set the fit:
   - **Reveal**: a gap between the lip and the groove's wall, on the lip's **Inner side** (toward the inside of the loop), its **Outer side**, or **Both sides**. **Reveal gap** is its size, measured square to the drafted sides. Where the groove cuts right through the wall (beside a lip that's flush with the inner wall, for example), there's no wall to leave a gap to, so a reveal on that side changes nothing.
   - **Top gap**: room between the top of the lip and the bottom of the groove.
4. The preview is the real result. Click **OK**.

The dialog warns when the lip runs off the parting face, because it's wider than the face or offset too far.

**Limits for now:**
- The parting face along each loop of edges must be flat. The edges themselves can be straight or curved.
- Both bodies must be in the same component.
- Each loop or chain is one sweep. A corner of the edges that's tighter than the lip reaches across the face (offset + width + gap) makes the sweep fail. Picking the edges on the other side of the parting face usually fixes that, because the lip then runs round the outside of those corners.
- The sizes are fixed in the profile sketches, and there's no edit command yet. To change a Lip, delete it and make it again.

## GX Curvature Comb
Pick the sketch curves that should show a comb (lines, arcs and splines all work), then set the scale and density. Clear the selection to remove all combs, or untick **Show combs** to hide them without forgetting the selection. By default, combs only show **while you're editing the curve's sketch**, and they disappear when you finish the sketch. Untick **Only while editing the sketch** to keep them on all the time. Combs are redrawn while you drag control points or edit dimensions. All combs share one scale, so a curvature jump where two curves meet shows up clearly.

Combs are display-only custom graphics. They're not saved in the file, but the list of combed curves is, and the combs come back when the add-in loads.

## Development
```
python -m unittest discover tests      # math + constraint bookkeeping, no Fusion needed
python tools/make_icons.py             # regenerate toolbar icons
```
- `lib/gx_math.py`: pure-Python layouts, B-spline evaluation, curvature and its rate
- `lib/fusion_geom.py`: selection analysis, spline creation, constraints
- `lib/comb_graphics.py`: comb drawing
- `lib/polygrid.py`: polygrid geometry (border offset, lattices, clipping pockets to the area, corners), pure Python
- `lib/screw_boss.py`: screw boss hole sizes (tap drills, thread-forming screws, McMaster heat-set inserts), pure Python
- `lib/lip.py`: lip and groove cross-sections, reveals, and ordering edges into sweep paths, pure Python
- `commands/`: the commands

Messages, errors (with full tracebacks) and constraint failures are logged to:
- `logs/gx_tool.log` in this folder, with timestamps. At 1 MB it's moved to `gx_tool.log.1` and a new log starts.
- Fusion's **Text Commands** window, prefixed `[GX_tool]`.
