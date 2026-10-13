"""Drives the Polygrid command (Solid > Modify) against fake Fusion objects: the grid sketch, the
extrude for each operation and extent, the timeline group, and editing (the same extrude is
repointed; the marker and group come back). No Fusion needed. Run: python -m unittest discover tests
"""

import contextlib
import json
import os
import sys
import unittest
import unittest.mock as mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_fakes as F  # noqa: E402

from GX_tool.commands import polygrid as cmd  # noqa: E402
from GX_tool.commands import polygrid_dialog as D  # noqa: E402
W = F.W


def data_of(ext):
    return json.loads(ext.attributes.itemByName("GX_tool", D.SOLID_ATTR).value)


class Polygrid(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        F.fake_fusion(self.stack)

    def tearDown(self):
        self.stack.close()

    def harness(self, **kw):
        return F.Harness(self.stack, cmd, **kw)

    def make(self, h, op="Cut holes", extent="Distance", bodies=True, extrude_as=None, face=None, **values):
        inputs = h.open()
        inputs.itemById("shape").pick("Square")
        h.changed("shape")
        inputs.itemById("side").expression = "6 mm"
        inputs.itemById("rib").expression = "1.5 mm"
        h.changed("rib")
        inputs.itemById("area").addSelection(h.profile)
        h.changed("area")
        inputs.itemById("op").pick(op)
        h.changed("op")
        if extrude_as:
            inputs.itemById("extrudeAs").pick(extrude_as)
            h.changed("extrudeAs")
        if bodies and inputs.itemById("bodies").isVisible:
            inputs.itemById("bodies").addSelection(h.body)
        inputs.itemById("extent").pick(extent)
        h.changed("extent")
        if face:
            inputs.itemById("face").addSelection(face)
        for id_, v in values.items():
            if id_ == "flip":
                inputs.itemById("flip").value = v
            else:
                inputs.itemById(id_).expression = v
            h.changed(id_)
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.preview()
        self.assertTrue(h.graphics.get(D.PREVIEW_KEY))
        h.execute()
        self.assertEqual((len(h.comp.made), len(h.comp.features.made)), (1, 1))
        return h.comp.made[0], h.comp.features.made[0]

    def check_group(self, h, sk, ext):
        tl = h.comp.timeline
        self.assertEqual(len(tl.groups), 1)
        self.assertEqual(tl.groups[0].members, [sk, ext])
        self.assertEqual(tl.groups[0].name, "Polygrid")
        self.assertEqual(tl.markerPosition, tl.count)

    def test_cut_holes(self):
        h = self.harness()
        sk, ext = self.make(h, depth="4 mm")
        res = h.dlg.res[0]
        self.assertEqual(sk.name, "Polygrid sketch")
        self.assertEqual(ext.name, "Polygrid")
        self.assertEqual(ext.operation, "cut")
        self.assertIsInstance(ext.extentOne, F.DistExt)
        self.assertEqual((ext.extentOne.distance.expression, ext.direction), ("4 mm", "pos"))
        self.assertEqual(ext.participantBodies, [h.body])
        # one profile per pocket; the disks inside the pockets' bosses (holes in the area) are left out
        self.assertEqual(len(ext.profile), len(res["pockets"]))
        in_pocket = W.pocket_finder(res)
        self.assertTrue(all(in_pocket(p.pt) for p in ext.profile))
        self.assertGreaterEqual(sk.profiles.count, len(ext.profile))
        F.check_closed(self, sk)
        self.check_group(h, sk, ext)
        data = data_of(ext)
        self.assertEqual((data["op"], data["extent"], data["depth"], data["sketch"]),
                         ("cut_holes", "distance", "4 mm", sk.entityToken))
        self.assertEqual(data["bodies"], [h.body.entityToken])
        self.assertEqual(sk.attributes.itemByName("GX_tool", D.SOLID_SKETCH_ATTR).value, ext.entityToken)

    def test_hole_inside_a_pocket(self):
        # big pockets around the area's holes: each is cut as a ring around the hole's boss; the
        # disk inside the hole isn't cut
        h = self.harness()
        inputs = h.open()
        inputs.itemById("shape").pick("Square")
        h.changed("shape")
        inputs.itemById("side").expression = "40 mm"
        inputs.itemById("vline").addSelection(F.FakeLine((3, -5), (3, 10)))
        inputs.itemById("area").addSelection(h.profile)
        inputs.itemById("bodies").addSelection(h.body)
        h.changed("area")
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.execute()
        sk, ext = h.comp.made[0], h.comp.features.made[0]
        res = h.dlg.res[0]
        ring = [pk for pk in res["pockets"] if len(pk["loops"]) == 2]
        self.assertEqual(len(ring), 2)  # both holes (the round one and the spline one) sit in a pocket
        self.assertEqual(len(ext.profile), len(res["pockets"]))
        self.assertEqual(sk.profiles.count, len(ext.profile) + 2)  # + the disks inside the holes
        in_area = W.area_finder(res)
        self.assertTrue(all(in_area(p.pt) for p in ext.profile))

    def test_rib_operations(self):
        for op, extrude_as, expect in (("Cut ribs", None, "cut"), ("Extrude ribs", "Join", "join"),
                                       ("Extrude ribs", "New body", "new")):
            h = self.harness()
            sk, ext = self.make(h, op=op, extrude_as=extrude_as)
            res = h.dlg.res[0]
            self.assertEqual(ext.operation, expect)
            self.assertEqual(ext.participantBodies, [] if expect == "new" else [h.body])
            in_pocket, in_area = W.pocket_finder(res), W.area_finder(res)
            self.assertTrue(ext.profile)
            self.assertTrue(all(in_area(p.pt) and not in_pocket(p.pt) for p in ext.profile))
            # every bit of rib web is there: the profiles left over are pockets or the area's holes
            rest = [p for p in sk.profiles.items if not any(p.pt == q.pt for q in ext.profile)]
            self.assertTrue(all(in_pocket(p.pt) or not in_area(p.pt) for p in rest))
            self.check_group(h, sk, ext)

    def test_extrude_holes(self):
        h = self.harness()
        sk, ext = self.make(h, op="Extrude holes", extrude_as="Join")
        self.assertEqual(ext.operation, "join")
        self.assertEqual(len(ext.profile), len(h.dlg.res[0]["pockets"]))

    def test_extents(self):
        h = self.harness()
        sk, ext = self.make(h, extent="Through all", flip=True)
        self.assertIsInstance(ext.extentOne, F.AllExt)
        self.assertEqual(ext.direction, "neg")
        h = self.harness()
        face = F.Face()
        sk, ext = self.make(h, extent="To face", face=face, offset="-1 mm")
        self.assertIsInstance(ext.extentOne, F.ToExt)
        self.assertIs(ext.extentOne.entity, face)
        self.assertEqual(ext.extentOne.offset.expression, "-1 mm")
        self.assertEqual(data_of(ext)["face"], face.entityToken)

    def test_edit_repoints_the_same_extrude(self):
        h = self.harness()
        sk, ext = self.make(h)
        tl = h.comp.timeline
        n_pockets = len(ext.profile)
        inputs = h.open(preselected=[tl.groups[0]])  # selecting the timeline group
        self.assertIs(h.dlg.ext, ext)
        self.assertEqual(tl.markerPosition, tl.items.index(sk))  # rolled back to just before the polygrid
        h.activate_args.command.beginStep.assert_called_once()
        self.assertEqual(inputs.itemById("area").sel, [h.profile])
        self.assertEqual(inputs.itemById("bodies").sel, [h.body])
        self.assertTrue(inputs.itemById("editing").isVisible)
        inputs.itemById("side").expression = "8 mm"
        h.changed("side")
        inputs.itemById("op").pick("Cut ribs")
        h.changed("op")
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.preview()
        h.execute()
        self.assertEqual((len(h.comp.made), len(h.comp.features.made)), (1, 1))  # same sketch, same extrude
        self.assertTrue(ext.isValid)
        self.assertEqual(ext.edits, 1)
        res = h.dlg.res[0]
        in_pocket = W.pocket_finder(res)
        self.assertTrue(all(not in_pocket(p.pt) for p in ext.profile))
        self.assertNotEqual(len(ext.profile), n_pockets)
        F.check_closed(self, sk)
        self.check_group(h, sk, ext)
        self.assertEqual((data_of(ext)["op"], data_of(ext)["side"]), ("cut_ribs", "8 mm"))

    def test_edit_from_the_sketch_and_live_values(self):
        h = self.harness()
        sk, ext = self.make(h, depth="4 mm")
        ext.extentOne.distance.expression = "7 mm"  # changed in Fusion's own Extrude dialog
        inputs = h.open(preselected=[sk.curves[3]])
        self.assertIs(h.dlg.ext, ext)
        self.assertEqual(inputs.itemById("depth").expression, "7 mm")

    def test_cancel_puts_the_timeline_back(self):
        h = self.harness()
        sk, ext = self.make(h)
        tl = h.comp.timeline
        h.open(preselected=[ext])
        self.assertEqual(tl.markerPosition, tl.items.index(sk))
        h.cancel()
        self.check_group(h, sk, ext)
        self.assertEqual(ext.edits, 0)

    def test_group_that_cant_be_rolled_into(self):
        # if Fusion won't roll into the group, it's taken apart for the edit and put back after
        h = self.harness()
        sk, ext = self.make(h)
        tl = h.comp.timeline
        real = F.TimelineObject.rollTo

        def strict(obj, before):
            if obj.parentGroup is not None:
                raise RuntimeError("can't roll into a group")
            real(obj, before)
        with mock.patch.object(F.TimelineObject, "rollTo", strict):
            h.open(preselected=[ext])
            self.assertEqual(tl.groups, [])
            h.cancel()
            self.check_group(h, sk, ext)
            h.open(preselected=[ext])
            h.inputs.itemById("rib").expression = "2 mm"
            h.changed("rib")
            self.assertTrue(h.validate())
            h.execute()
            self.check_group(h, sk, ext)
            self.assertEqual(ext.edits, 1)

    def test_errors(self):
        h = self.harness()
        inputs = h.open()
        inputs.itemById("area").addSelection(h.profile)
        h.changed("area")
        self.assertFalse(h.validate())
        self.assertIn("Pick the bodies", inputs.itemById("info").formattedText)
        inputs.itemById("bodies").addSelection(h.body)
        inputs.itemById("extent").pick("To face")
        h.changed("extent")
        self.assertFalse(h.validate())
        self.assertIn("face to stop at", inputs.itemById("info").formattedText)
        inputs.itemById("face").addSelection(F.Face())
        self.assertTrue(h.validate())
        self.assertFalse(inputs.itemById("depth").isVisible)
        self.assertTrue(inputs.itemById("offset").isVisible)

    def test_preview_solid(self):
        h = self.harness()
        inputs = h.open()
        inputs.itemById("area").addSelection(h.profile)
        inputs.itemById("bodies").addSelection(h.body)
        h.changed("area")
        inputs.itemById("previewSolid").value = True
        self.assertTrue(h.validate())
        h.preview()  # builds the real thing (Fusion would take it away again)
        self.assertEqual((len(h.comp.made), len(h.comp.features.made)), (1, 1))

    def test_marking_menu(self):
        h = self.harness()
        sk, ext = self.make(h)
        self.assertEqual(D.polygrid_of(h.comp.timeline.groups[0]), ("solid", ext))
        self.assertEqual(D.polygrid_of(sk.curves[0]), ("solid", ext))
        self.assertEqual(D.polygrid_of(h.profile), (None, None))
        handler = D.marking_menu(cmd.EDIT_ID, "solid")
        args = mock.Mock()
        args.selectedEntities = [ext]
        args.linearMarkingMenu.controls.itemById.return_value = None
        handler(args)
        args.linearMarkingMenu.controls.addCommand.assert_called_once()


if __name__ == "__main__":
    unittest.main()
