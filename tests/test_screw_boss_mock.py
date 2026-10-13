"""Drives the Screw Boss command (Solid > Create) against fake Fusion objects: the presets in the
dialog, the boss sketch, the extrude, a top sketch and Hole feature for each plane the bosses end on,
the flat-bottom fallback, the timeline group and the errors. No Fusion needed.
Run: python -m unittest discover tests
"""

import contextlib
import json
import math
import os
import sys
import types
import unittest
import unittest.mock as mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_fakes as F  # noqa: E402

import adsk.core  # noqa: E402
import adsk.fusion  # noqa: E402

from GX_tool.commands import gx_comb  # noqa: E402
from GX_tool.commands import screw_boss as cmd  # noqa: E402
from GX_tool.lib import futil  # noqa: E402


class FakePlane:
    def __init__(self, origin, normal):
        self.origin, self.normal = F.Pt(*origin), F.Pt(*normal)


class TopFace(F.Face):
    """A flat face at height z, covering disks of radius r round the given (x, y) centres."""

    def __init__(self, z, centres, r, normal=(0.0, 0.0, 1.0), curved=False):
        super().__init__()
        self.geometry = "a cylinder" if curved else FakePlane((0.0, 0.0, z), normal)
        self.frame = ((0.0, 0.0, z), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        ev = mock.Mock()
        ev.getParameterAtPoint.side_effect = lambda p: (True, p)
        ev.isParameterOnFace.side_effect = lambda p: abs(p.z - z) < 1e-9 and any(
            math.hypot(p.x - c[0], p.y - c[1]) <= r for c in centres)
        ev.getPointAtParameter.side_effect = lambda p: (True, p)
        self.evaluator = ev


class HoleInput:
    def __init__(self, dia):
        self.dia, self.points, self.depth = dia, None, None
        self.tipAngle, self.isDefaultDirection, self.participantBodies = "118 deg", True, None

    def setPositionBySketchPoints(self, coll):
        self.points = list(coll)

    def setDistanceExtent(self, d):
        self.depth = d


class Harness:
    """A fake design with a sketch "Base" (identity frame, z = 0) holding the points for the bosses."""

    def __init__(self, stack, pts=((1.0, 1.0), (4.0, 1.0), (4.0, 3.0))):
        self.comp = F.Component(True)
        self.src = F.Sketch("Base", self.comp)
        self.points = [F.SketchPoint(self.src, F.Pt(x, y)) for x, y in pts]
        self.body = F.Body()
        self.tops = None  # end faces of the boss extrude; None: one flat top at the extrude's height
        self.settings, self.handlers = {}, []
        tl = self.comp.timeline
        tl.items.append("Extrude2")
        tl.moveToEnd()
        self.design = mock.Mock()
        self.design.timeline = tl
        self.design.designType = F.DesignTypes.ParametricDesignType
        self.design.unitsManager.defaultLengthUnits = "mm"
        self.design.unitsManager.formatInternalValue.side_effect = lambda v, u, s: "%.4g mm" % (v * 10)
        self.design.findEntityByToken.side_effect = lambda t: [F.TOKENS[t]] if t in F.TOKENS else []
        self.holes = []
        comp, h = self.comp, self

        def add_hole(hi):
            if h.hole_fails and hi.tipAngle == "180 deg":
                raise RuntimeError("tip angle out of range")
            f = F.TimelineFeature(comp, input=hi, kind="hole", extentDefinition=F.DistExt(hi.depth),
                                  holeDiameter=types.SimpleNamespace(expression=hi.dia, value=F._parse(hi.dia)),
                                  tipAngle=types.SimpleNamespace(value=math.radians(float(hi.tipAngle.split()[0]))))
            comp.features.made.append(f)
            h.holes.append(f)
            return f
        self.hole_fails = False
        holes = mock.Mock()
        holes.createSimpleInput.side_effect = HoleInput
        holes.add.side_effect = add_hole
        comp.features.holeFeatures = holes

        def project2(sk, ents, linked):
            e = ents[0]
            return [F.SketchPoint(sk, sk.modelToSketchSpace(e.parentSketch.sketchToModelSpace(e.geometry)))]

        def end_faces(ext):
            if h.tops is not None:
                return F.Items(h.tops)
            sk = ext.profile[0].parentSketch
            circles = [c for c in sk.curves if c.kind == "circle" and c.isValid]
            z = F._parse(ext.extentOne.distance.expression) * (1 if ext.direction == "pos" else -1)
            return F.Items([TopFace(z, [F.xy(c.center) for c in circles], circles[0].radius)])

        def delete_sketch(sk):
            sk.isValid = False
            sk.parentComponent.timeline.remove(sk)

        for obj, name, val in (
                (adsk.fusion.Design, "cast", lambda x: self.design),
                (adsk.fusion.HoleFeature, "cast", lambda x: x if getattr(x, "kind", None) == "hole" else None),
                (adsk.fusion.SketchDiameterDimension, "cast",
                 lambda x: x if getattr(x, "kind", None) == "diameter" else None),
                (F.Sketch, "deleteMe", delete_sketch),
                (adsk.core.Plane, "cast", lambda x: x if isinstance(x, FakePlane) else None),
                (adsk.fusion.BRepFace, "cast", lambda x: x if isinstance(x, F.Face) else None),
                (adsk.fusion.ConstructionPlane, "cast", lambda x: None),
                (F.Sketch, "project2", project2),
                (F.Extrude, "endFaces", property(end_faces)),
                (F.Extrude, "bodies", property(lambda ext: F.Items([h.body]))),
                (gx_comb, "get_setting", lambda name, default: self.settings.get(name, default)),
                (gx_comb, "set_setting", lambda name, value: self.settings.__setitem__(name, value)),
                (futil, "add_handler", lambda event, cb, c: self.handlers.append(cb))):
            stack.enter_context(mock.patch.object(obj, name, val, create=True))

    @property
    def dlg(self):
        return cmd._dlg

    def open(self, preselected=()):
        futil.ui.activeSelections.count = len(preselected)
        futil.ui.activeSelections.item.side_effect = lambda k: mock.Mock(entity=preselected[k])
        self.inputs = F.Inputs()
        args = mock.Mock()
        args.command.commandInputs = self.inputs
        cmd._on_created(args)
        self.dlg.on_activate(mock.Mock())
        return self.inputs

    def set(self, **values):
        """Set inputs by id: dropdowns by label, selections by entity list, values by expression."""
        for id_, v in values.items():
            inp = self.inputs.itemById(id_)
            if isinstance(inp, F.DD):
                inp.pick(v)
            elif isinstance(inp, F.Sel):
                inp.clearSelection()
                for e in v:
                    inp.addSelection(e)
            elif isinstance(inp, F.ValueInp):
                inp.expression = v
            else:
                inp.value = v
            self.changed(id_)

    def changed(self, id_):
        args = mock.Mock()
        args.input = self.inputs.itemById(id_)
        self.dlg.on_input_changed(args)

    def validate(self):
        args = mock.Mock()
        self.dlg.on_validate(args)
        return args.areInputsValid

    def info(self):
        return self.inputs.itemById("info").formattedText

    def preview(self):
        tl = self.comp.timeline
        snap = (list(tl.items), tl.markerPosition, list(tl.groups))
        counts = [(lst, len(lst)) for lst in (self.comp.made, self.comp.features.made, self.holes)]
        args = mock.Mock()
        self.dlg.on_preview(args)
        # like Fusion, undo what the preview made
        tl.items[:], tl.markerPosition, tl.groups[:] = snap
        for lst, n in counts:
            for x in lst[n:]:
                x.isValid = False
            del lst[n:]
        return args

    def execute(self):
        args = mock.Mock()
        args.executeFailed = False
        self.dlg.on_execute(args)
        args.terminationReason = "done"
        self.dlg.on_destroy(args)
        return args

    def cancel(self):
        args = mock.Mock()
        args.terminationReason = "cancelled"
        self.dlg.on_destroy(args)


class ScrewBoss(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        F.fake_fusion(self.stack)

    def tearDown(self):
        self.stack.close()

    def harness(self, **kw):
        return Harness(self.stack, **kw)

    def make(self, h, **values):
        """Open the dialog, pick all the points, set values, and click OK."""
        h.open()
        h.set(points=h.points, **values)
        self.assertTrue(h.validate(), h.info())
        h.preview()
        self.assertNotIn("color:#c33", h.info())
        args = h.execute()
        self.assertFalse(args.executeFailed, getattr(args, "executeFailedMessage", ""))
        return h.comp.made[0], h.comp.features.of(F.Extrude)[0]

    def test_presets(self):
        h = self.harness()
        inputs = h.open()
        dia, depth, ins = inputs.itemById("holeDia"), inputs.itemById("holeDepth"), inputs.itemById("insert")
        self.assertEqual(dia.expression, "2.5 mm")  # M3 tap drill
        self.assertFalse(dia.isEnabled)
        self.assertFalse(ins.isVisible)
        h.set(kind="Heat-set insert")
        self.assertTrue(ins.isVisible)
        self.assertEqual([it.name for it in ins.items][2], "5.7 mm long, McMaster 94459A140")
        self.assertEqual(ins.selectedItem.name, "5.7 mm long, McMaster 94459A140")
        self.assertEqual((dia.expression, depth.expression), ("3.99 mm", "6.7 mm"))
        h.set(size="M5")
        self.assertEqual(len(ins.items), 3)
        self.assertEqual(h.dlg.part(), "94459A775")
        self.assertEqual((dia.expression, depth.expression), ("6.4 mm", "9.5 mm"))
        h.set(insert="9.5 mm long, McMaster 94459A180")
        self.assertEqual((dia.expression, depth.expression), ("6.5 mm", "11.1 mm"))
        h.set(kind="Self-tapping screw")
        self.assertEqual((dia.expression, depth.expression), ("4 mm", "11.1 mm"))  # depth is left alone
        h.set(kind="Custom diameter")
        self.assertTrue(dia.isEnabled)
        self.assertFalse(inputs.itemById("size").isVisible)
        self.assertEqual(dia.expression, "4 mm")  # starts from the last preset
        h.set(sizeBy="Outer diameter")  # the boss stays the same size
        self.assertAlmostEqual(inputs.itemById("od").value, 0.8)
        h.set(od="1 cm", sizeBy="Wall thickness")
        self.assertAlmostEqual(inputs.itemById("wall").value, 0.3)

    def test_m8_and_m10(self):
        h = self.harness()
        inputs = h.open()
        self.assertEqual([it.name for it in inputs.itemById("size").items][-2:], ["M8", "M10"])
        h.set(size="M8")
        self.assertEqual(inputs.itemById("holeDia").expression, "6.8 mm")
        h.set(kind="Heat-set insert", size="M10")
        self.assertEqual([it.name for it in inputs.itemById("insert").items], ["12.7 mm long, McMaster 94459A779"])
        self.assertEqual((inputs.itemById("holeDia").expression, inputs.itemById("holeDepth").expression),
                         ("11.9 mm", "15.7 mm"))

    def test_bosses_a_distance(self):
        h = self.harness()
        sk, ext = self.make(h, holeDepth="6 mm")
        self.assertEqual(sk.name, "Screw boss sketch")
        self.assertFalse(sk.isVisible)
        circles = [c for c in sk.curves if c.kind == "circle"]
        self.assertEqual([F.xy(c.center) for c in circles], [(1.0, 1.0), (4.0, 1.0), (4.0, 3.0)])
        self.assertTrue(all(abs(c.radius - 0.325) < 1e-9 for c in circles))  # (2.5 mm + 2 x 2 mm) / 2
        dims = sk.sketchDimensions.made
        self.assertEqual(len(dims), 1)
        self.assertEqual(dims[0].parameter.expression, "(2.5 mm) + 2 * (2 mm)")
        self.assertEqual(sk.geometricConstraints.addEqual.call_count, 2)
        self.assertEqual((ext.name, ext.operation, ext.direction), ("Screw boss", "join", "pos"))
        self.assertEqual(ext.extentOne.distance.expression, "10 mm")
        self.assertEqual(len(ext.profile), 3)
        top = h.comp.made[1]
        self.assertEqual((top.name, top.frame[0], top.isVisible), ("Screw boss top", (0.0, 0.0, 1.0), False))
        (hole,) = h.holes
        hi = hole.input
        self.assertEqual((hole.name, hi.dia, hi.depth, hi.tipAngle), ("Screw boss hole", "2.5 mm", "6 mm", "118 deg"))
        self.assertTrue(hi.isDefaultDirection)
        self.assertEqual(hi.participantBodies, [h.body])
        self.assertEqual([(p.parentSketch, F.xy(p.geometry)) for p in hi.points],
                         [(top, (1.0, 1.0)), (top, (4.0, 1.0)), (top, (4.0, 3.0))])
        tl = h.comp.timeline
        self.assertEqual([(g.name, g.members) for g in tl.groups], [("Screw Boss", [sk, ext, top, hole])])
        self.assertEqual(tl.markerPosition, tl.count)
        data = json.loads(ext.attributes.itemByName("GX_tool", cmd.ATTR).value)
        self.assertEqual((data["kind"], data["size"], data["holeDepth"]), ("tap", "M3", "6 mm"))
        self.assertEqual(data["holeItems"], [top.entityToken, hole.entityToken])
        for x in (sk, top, hole):
            self.assertEqual(x.attributes.itemByName("GX_tool", cmd.OF_ATTR).value, ext.entityToken)
        self.assertFalse(h.inputs.itemById("editing").isVisible)
        self.assertEqual(json.loads(h.settings[cmd.LAST_KEY])["holeDepth"], "6 mm")
        # the next run starts from these settings
        inputs = h.open()
        self.assertEqual(inputs.itemById("holeDepth").expression, "6 mm")

    def test_insert_flat_bottom_new_body_flipped(self):
        h = self.harness()
        sk, ext = self.make(h, kind="Heat-set insert", size="M4", end="Flat", bossAs="New body", flip=True,
                            sizeBy="Outer diameter", od="12 mm")
        self.assertEqual((ext.operation, ext.direction), ("new", "neg"))
        self.assertEqual(sk.sketchDimensions.made[0].parameter.expression, "12 mm")
        top = h.comp.made[1]
        self.assertEqual(top.frame[0], (0.0, 0.0, -1.0))  # the bosses go down
        hi = h.holes[0].input
        self.assertEqual((hi.dia, hi.depth, hi.tipAngle), ("5.56 mm", "7.8 mm", "180 deg"))
        self.assertFalse(hi.isDefaultDirection)  # the top sketch faces down the boss: drill the other way
        self.assertIn("McMaster 94459A160", h.info())
        self.assertIn("flat bottom", h.info())

    def test_flat_bottom_fallback_cut(self):
        h = self.harness()
        h.hole_fails = True
        self.make(h, end="Flat", holeDepth="5 mm")
        self.assertEqual(h.holes, [])
        boss, cut = h.comp.features.of(F.Extrude)
        top = h.comp.made[1]
        self.assertEqual((cut.name, cut.operation, cut.direction), ("Screw boss hole", "cut", "neg"))
        self.assertEqual(cut.extentOne.distance.expression, "5 mm")
        self.assertEqual(cut.participantBodies, [h.body])
        self.assertEqual([round(c.radius, 9) for c in top.curves if c.kind == "circle"], [0.125] * 3)
        self.assertEqual(h.comp.timeline.groups[0].members[-1], cut)

    def test_up_to_a_face_with_two_levels(self):
        h = self.harness()
        target = F.Face(pointOnFace=F.Pt(0.0, 0.0, 2.0))
        h.tops = [TopFace(1.5, [(1.0, 1.0), (4.0, 1.0)], 0.4), TopFace(2.5, [(4.0, 3.0)], 0.4)]
        sk, ext = self.make(h, extent="To object", to=[target], offset="-0.5 mm")
        self.assertFalse(h.inputs.itemById("height").isVisible)
        self.assertEqual((ext.extentOne.entity, ext.extentOne.offset.expression, ext.direction), (target, "-0.5 mm", "pos"))
        self.assertEqual([len(x.input.points) for x in h.holes], [2, 1])
        self.assertEqual([F.xy(p.geometry) for p in h.holes[1].input.points], [(4.0, 3.0)])
        tops = [s for s in h.comp.made if s.name == "Screw boss top"]
        self.assertEqual([s.frame[0][2] for s in tops], [1.5, 2.5])
        self.assertIn("up to the face", h.info())
        self.assertEqual(h.comp.timeline.groups[0].members[-1], h.holes[-1])

    def test_up_to_an_object_below(self):
        h = self.harness()
        target = F.Face(pointOnFace=F.Pt(0.0, 0.0, -2.0))
        h.tops = [TopFace(-2.0, [(1.0, 1.0), (4.0, 1.0), (4.0, 3.0)], 0.4)]
        sk, ext = self.make(h, extent="To object", to=[target])
        self.assertEqual(ext.direction, "neg")
        # and if Fusion refuses the first direction, the other one is tried
        h2 = self.harness()
        h2.tops = h.tops
        real_add = h2.comp.features.extrudeFeatures.add.side_effect
        tries = []

        def add(inp):
            tries.append(inp.direction)
            if len(tries) == 1:
                raise RuntimeError("no target body found")
            return real_add(inp)
        h2.comp.features.extrudeFeatures.add.side_effect = add
        h2.open()
        h2.set(extent="To object", to=[target])
        h2.set(points=h2.points)
        h2.execute()
        self.assertEqual(tries, ["neg", "pos"])

    def test_notes(self):
        h = self.harness(pts=((1.0, 1.0), (1.3, 1.0), (1.0, 1.0)))
        h.open()
        h.set(points=h.points, kind="Heat-set insert", holeDepth="5 mm", height="4 mm")
        self.assertTrue(h.validate())
        h.preview()
        info = h.info()
        self.assertIn("<b>2 bosses</b>", info)
        self.assertIn("shorter than the insert (5.7 mm)", info)
        self.assertIn("reaches below the boss", info)
        self.assertIn("on top of another", info)
        self.assertIn("1 pair(s) of bosses overlap", info)
        h.set(kind="Self-tapping screw", wall="1 mm")  # 2.4 + 2 mm < 2 d = 6 mm
        h.preview()
        self.assertIn("thread-forming screws: a boss at least Ø6 mm", h.info())

    def test_errors(self):
        h = self.harness()
        h.open()
        self.assertFalse(h.validate())
        self.assertIn("Select the sketch points", h.info())
        other = F.SketchPoint(F.Sketch("Other", h.comp), F.Pt(0.0, 0.0))
        h.set(points=[h.points[0], other])
        self.assertFalse(h.validate())
        self.assertIn("one sketch", h.info())
        h.set(points=h.points, kind="Custom diameter", holeDia="10 mm", sizeBy="Outer diameter", od="8 mm")
        self.assertFalse(h.validate())
        self.assertIn("must be wider than its hole", h.info())
        h.set(holeDia="2 mm", sizeBy="Wall thickness", wall="0 mm")
        self.assertFalse(h.validate())
        self.assertIn("Wall thickness must be above zero", h.info())
        h.set(wall="2 mm", extent="To object")
        self.assertFalse(h.validate())
        self.assertIn("Pick the face, plane or body", h.info())

    def test_tops_that_wont_do(self):
        for tops, msg in (([TopFace(1.0, [], 1.0, curved=True)], "curved face"),
                          ([TopFace(1.0, [], 1.0, normal=(0.0, 0.6, 0.8))], "isn't parallel"),
                          ([], "no top face")):
            h = self.harness()
            h.tops = tops
            h.open()
            h.set(points=h.points)
            self.assertTrue(h.validate())
            h.preview()
            self.assertIn(msg, h.info())
            args = h.execute()
            self.assertTrue(args.executeFailed)
            self.assertIn(msg, args.executeFailedMessage)

    def test_points_sketch_on_a_face(self):
        for at_end, face_found in ((True, True), (False, True), (True, False)):
            h = self.harness()
            h.src.on_face = True
            tl = h.comp.timeline
            if not at_end:
                tl.items.append("Fillet1")
                tl.markerPosition = tl.count - 1  # rolled back before the fillet
            if not face_found:
                h.design.findEntityByToken.side_effect = lambda t: []
            sk, ext = self.make(h)
            members = tl.groups[0].members
            self.assertEqual(len(members), 4)
            i = tl.items.index(sk)
            self.assertEqual(tl.items[i:i + 4], members)  # all together, from the boss sketch on
            self.assertEqual(tl.markerPosition, tl.count if at_end else tl.count - 1)


    # ------------------------------------------------------------- editing
    def test_finding_a_screw_boss_and_the_menu(self):
        h = self.harness()
        sk, ext = self.make(h)
        top, hole = h.comp.made[1], h.holes[0]
        circle = next(c for c in sk.curves if c.kind == "circle")
        for x in (ext, sk, top, hole, circle, sk.points[1], h.comp.timeline.groups[0]):
            self.assertIs(cmd.screw_boss_of(x), ext, x)
        for x in (h.src, h.points[0], h.body, None):
            self.assertIsNone(cmd.screw_boss_of(x))
        # right-click: Edit Screw Boss is added for any part of it, and hidden for anything else
        args = mock.Mock()
        args.selectedEntities = [hole]
        args.linearMarkingMenu.controls.itemById.return_value = None
        cmd._marking_menu(args)
        args.linearMarkingMenu.controls.addCommand.assert_called_once()
        shown = mock.Mock(isVisible=True)
        args.selectedEntities = [h.points[0]]
        args.linearMarkingMenu.controls.itemById.return_value = shown
        cmd._marking_menu(args)
        self.assertFalse(shown.isVisible)
        # a screw boss's own sketch points can't be picked for new bosses
        h.open()
        pre = mock.Mock(isSelectable=True)
        pre.activeInput.id = "points"
        pre.selection.entity = sk.points[1]
        h.dlg.on_pre_select(pre)
        self.assertFalse(pre.isSelectable)
        pre.isSelectable, pre.selection.entity = True, h.points[0]
        h.dlg.on_pre_select(pre)
        self.assertTrue(pre.isSelectable)

    def test_edit(self):
        h = self.harness()
        sk, ext = self.make(h, holeDepth="6 mm")
        old_top, old_hole = h.comp.made[1], h.holes[0]
        tl = h.comp.timeline
        inputs = h.open(preselected=[old_hole])
        self.assertEqual(tl.markerPosition, tl.items.index(sk))  # rolled back to just before it
        self.assertTrue(inputs.itemById("editing").isVisible)
        self.assertEqual(inputs.itemById("points").sel, h.points)
        self.assertEqual(inputs.itemById("holeDepth").expression, "6 mm")
        h.set(points=h.points[:2], kind="Heat-set insert", height="12 mm")
        self.assertTrue(h.validate(), h.info())
        h.preview()
        self.assertNotIn("color:#c33", h.info())
        self.assertFalse(h.execute().executeFailed)
        self.assertEqual(h.comp.features.of(F.Extrude), [ext])  # the same extrude, repointed
        self.assertEqual((ext.edits, len(ext.profile), ext.extentOne.distance.expression), (1, 2, "12 mm"))
        self.assertEqual(len([c for c in sk.curves if c.isValid and c.kind == "circle"]), 2)
        self.assertEqual(len(sk.sketchDimensions.made), 2)  # the old dimension went with its circle
        self.assertFalse(old_top.isValid or old_hole.isValid)
        top, hole = h.comp.made[-1], h.holes[-1]
        self.assertTrue(top.isValid and hole.isValid)
        self.assertEqual((hole.input.dia, hole.input.depth, len(hole.input.points)), ("3.99 mm", "6.7 mm", 2))
        self.assertEqual([(g.name, g.members) for g in tl.groups], [("Screw Boss", [sk, ext, top, hole])])
        self.assertEqual(tl.markerPosition, tl.count)
        data = json.loads(ext.attributes.itemByName("GX_tool", cmd.ATTR).value)
        self.assertEqual((data["kind"], data["holeItems"]), ("insert", [top.entityToken, hole.entityToken]))
        self.assertEqual(hole.attributes.itemByName("GX_tool", cmd.OF_ATTR).value, ext.entityToken)
        # and it can be edited again
        inputs = h.open(preselected=[top])
        self.assertEqual(inputs.itemById("kind").selectedItem.name, "Heat-set insert")
        self.assertEqual(inputs.itemById("points").sel, h.points[:2])
        h.cancel()

    def test_edit_reads_changes_made_in_fusion(self):
        h = self.harness()
        sk, ext = self.make(h)
        inputs = h.open(preselected=[ext])  # nothing changed: as stored
        self.assertEqual([inputs.itemById(k).selectedItem.name for k in ("kind", "sizeBy", "end")],
                         ["Tap drill", "Wall thickness", "Drill point (118°)"])
        h.cancel()
        hole = h.holes[0]
        ext.extentOne = F.DistExt("15 mm")
        hole.holeDiameter = types.SimpleNamespace(expression="3 mm", value=0.3)
        hole.tipAngle.value = math.pi
        hole.extentDefinition = F.DistExt("8 mm")
        dim = sk.sketchDimensions.made[0].parameter
        dim.expression, dim.value = "9 mm", 0.9
        inputs = h.open(preselected=[ext])
        self.assertEqual([inputs.itemById(k).selectedItem.name for k in ("kind", "sizeBy", "end")],
                         ["Custom diameter", "Outer diameter", "Flat"])
        self.assertEqual([inputs.itemById(k).expression for k in ("height", "holeDia", "holeDepth", "od")],
                         ["15 mm", "3 mm", "8 mm", "9 mm"])
        h.cancel()

    def test_cancel_puts_the_timeline_back(self):
        h = self.harness()
        sk, ext = self.make(h)
        tl = h.comp.timeline
        items, groups = list(tl.items), [(g.name, g.members) for g in tl.groups]
        # Fusion may not roll into a group: then it's ungrouped while the dialog is open
        real_roll = sk.timelineObject.rollTo

        def roll(before):
            if sk.timelineObject.parentGroup is not None:
                raise RuntimeError("can't roll into a group")
            real_roll(before)
        sk.timelineObject.rollTo = roll
        h.open(preselected=[tl.groups[0]])
        self.assertEqual((tl.markerPosition, tl.groups), (tl.items.index(sk), []))
        h.set(height="3 mm")
        h.preview()
        h.cancel()
        self.assertEqual(tl.items, items)
        self.assertEqual(tl.markerPosition, tl.count)
        self.assertEqual([(g.name, g.members) for g in tl.groups], groups)

    def test_edit_with_the_timeline_rolled_back(self):
        h = self.harness()
        sk, ext = self.make(h)
        tl = h.comp.timeline
        tl.items.append("Fillet1")
        tl.markerPosition = tl.count - 1  # rolled back before the fillet
        h.open(preselected=[ext])
        h.tops = [TopFace(1.0, [(1.0, 1.0), (4.0, 1.0)], 0.4), TopFace(1.5, [(4.0, 3.0)], 0.4)]
        self.assertFalse(h.execute().executeFailed)
        self.assertEqual(len(tl.groups[0].members), 6)  # two tops and two holes now
        self.assertEqual(tl.markerPosition, tl.items.index("Fillet1"))

    def test_edit_when_the_extrude_cant_be_repointed(self):
        h = self.harness()
        sk, ext = self.make(h)
        h.open(preselected=[ext])
        ext.setOneSideExtent = mock.Mock(side_effect=RuntimeError("can't"))
        self.assertFalse(h.execute().executeFailed)
        (new,) = h.comp.features.of(F.Extrude)
        self.assertFalse(ext.isValid)
        self.assertEqual(h.comp.timeline.groups[0].members[:2], [sk, new])
        self.assertTrue(new.attributes.itemByName("GX_tool", cmd.ATTR))
        self.assertIs(cmd.screw_boss_of(h.holes[-1]), new)

    def test_edit_when_points_are_gone(self):
        h = self.harness()
        sk, ext = self.make(h)
        h.points[2].isValid = False
        inputs = h.open(preselected=[ext])
        self.assertIn("points are gone", inputs.itemById("editing").formattedText)
        self.assertEqual(inputs.itemById("points").sel, h.points[:2])
        h.cancel()
        self.assertEqual(h.comp.timeline.markerPosition, h.comp.timeline.count)


if __name__ == "__main__":
    unittest.main()
