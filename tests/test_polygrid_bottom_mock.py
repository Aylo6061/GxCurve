"""Polygrid's bottom blends against fake Fusion objects: swept G3/G4 rings (one sweep per distinct
pocket outline, copies for the rest, one Combine), the G2 fallback, radius / G2 fillets, and
rebuilding them on edit. No Fusion needed. Run: python -m unittest discover tests
"""

import contextlib
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_fakes as F  # noqa: E402

from GX_tool.commands import polygrid as cmd  # noqa: E402
from GX_tool.commands import polygrid_dialog as D  # noqa: E402
W = F.W
G3 = "G3 curve (7 points)"


class Bottom(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        F.fake_fusion(self.stack)

    def tearDown(self):
        self.stack.close()

    def open(self, op="Cut holes", bottom=G3, radius="0.8 mm", corner="1.5 mm", depth="4 mm", extrude_as=None):
        h = F.Harness(self.stack, cmd)
        inputs = h.open()
        inputs.itemById("shape").pick("Square")
        h.changed("shape")
        for id_, v in (("side", "6 mm"), ("rib", "1.5 mm"), ("corner", corner), ("depth", depth)):
            inputs.itemById(id_).expression = v
            h.changed(id_)
        inputs.itemById("area").addSelection(h.profile)
        h.changed("area")
        inputs.itemById("op").pick(op)
        h.changed("op")
        if extrude_as:
            inputs.itemById("extrudeAs").pick(extrude_as)
            h.changed("extrudeAs")
        if inputs.itemById("bodies").isVisible:
            inputs.itemById("bodies").addSelection(h.body)
        if inputs.itemById("bottom").isVisible:
            inputs.itemById("bottom").pick(bottom)
            h.changed("bottom")
            inputs.itemById("bottomRadius").expression = radius
            h.changed("bottomRadius")
        return h, inputs

    def make(self, **kw):
        h, inputs = self.open(**kw)
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.preview()
        h.execute()
        feats = h.comp.features
        return h, h.comp.made[0], feats.of(F.Extrude)[0], feats

    def test_g3_rings(self):
        h, sk, ext, feats = self.make()
        res = h.dlg.res[0]
        loops = W.blend_loops(res, "cut_holes")
        groups = W.shape_groups([lp for _, _, lp, _ in loops])
        sweeps = feats.of(F.Sweep)
        self.assertEqual(len(sweeps), len(groups))
        self.assertLess(len(groups), len(loops))  # whole pockets share one sweep
        base = feats.of(F.TimelineFeature, "base")
        self.assertEqual(len(base), 1)
        self.assertEqual(base[0].bodies.count, len(loops) - len(groups))
        combines = feats.of(F.TimelineFeature, "combine")
        self.assertEqual(len(combines), 1)
        ci = combines[0].input
        self.assertEqual((ci.target, len(ci.tools), ci.operation, ci.isKeepToolBodies), (h.body, len(loops), "join", False))
        self.assertEqual(feats.of(F.TimelineFeature, "fillet"), [])
        in_pocket = W.pocket_finder(res)
        for sw in sweeps:
            self.assertEqual(sw.input.orientation, "perp")
            psk = sw.input.profile.parentSketch
            spline = [c for c in psk.curves if c.kind == "cspline"][0]
            ctrl = [F.xy(q) for q in spline.ctrl]
            self.assertEqual(len(ctrl), 7)
            self.assertEqual(W.g.end_continuity_to_line(ctrl, 5), 3)  # G3 to the floor...
            self.assertEqual(W.g.end_continuity_to_line(ctrl, 5, from_end=True), 3)  # ...and to the wall
            floor = F.v3(psk.sketchToModelSpace(F.Pt(*ctrl[0], 0.0)))
            wall = F.v3(psk.sketchToModelSpace(F.Pt(*ctrl[-1], 0.0)))
            self.assertAlmostEqual(floor[2], 0.4, places=9)       # on the floor (4 mm down the cut)
            self.assertLess(wall[2], 0.4)                          # up the wall, toward the opening
            self.assertTrue(in_pocket(floor[:2]))                  # inside the pocket
            self.assertFalse(sw.input.path.entities[0].parentSketch is psk)
        # the group holds everything, in order, and the settings remember the bottom features
        tl = h.comp.timeline
        members = tl.groups[0].members
        self.assertIs(members[0], sk)
        self.assertIs(members[1], ext)
        self.assertIs(members[-1], combines[0])
        self.assertEqual(tl.markerPosition, tl.count)
        data = json.loads(ext.attributes.itemByName("GX_tool", D.SOLID_ATTR).value)
        self.assertEqual(len(data["bottomItems"]), len(members) - 2)
        self.assertEqual(data["bottom"], "g3")

    def test_rings_go_all_the_way_round(self):
        # regression: Fusion's chaining from one curve picked up only that curve, so the G3 bottom
        # covered one corner. The path is now every curve of the outline, whatever chaining does.
        h, inputs = self.open()
        h.comp.chain_from = lambda ent: [ent]
        self.assertTrue(h.validate())
        h.execute()
        res = h.dlg.res[0]
        loops = W.blend_loops(res, "cut_holes")
        sweeps = h.comp.features.of(F.Sweep)
        self.assertEqual(len(sweeps), len(W.shape_groups([lp for _, _, lp, _ in loops])))
        for sw in sweeps:
            path = sw.input.path
            self.assertGreater(path.count, 1)
            self.assertTrue(path.isClosed)
            self.assertEqual(path.count, len(path.entities[0].parentSketch.chain(path.entities[0])))

    def test_failure_messages_are_plain(self):
        from GX_tool.commands import polygrid_bottom as B
        cfg = dict(bottom="g3", bottomRexpr="1 mm")
        msg = B.explain(RuntimeError("3 : ASM_SELF_INTER -   The body would intersect itself.\n  Try ..."), cfg)
        self.assertIn("too big for the pocket corners", msg)
        msg = B.explain(RuntimeError("3 : Fillet1 / Compute Failed // ASM_BL_UNFIN_SHEET -   The fillet/chamfer "
                                     "could not be created at the requested size."), cfg)
        self.assertIn("doesn't fit", msg)
        self.assertIn("1 mm", msg)
        msg = B.explain(ValueError("the extrude has no end face (it goes right through)"), cfg)
        self.assertIn("right through", msg)
        self.assertEqual(B.explain(RuntimeError("3 : Foo / Bar // ASM_X - Something odd happened."), cfg),
                         "Something odd happened.")

    def test_fillet_failure_is_explained(self):
        # all sweeps fail, then the G2 fallback fillet fails too: both reasons reach the user
        h, inputs = self.open()
        h.comp.sweep_fails = lambda ent: True

        def boom(fi):
            raise RuntimeError("3 : Fillet1 / Compute Failed // ASM_BL_UNFIN_SHEET -   The fillet/chamfer "
                               "could not be created at the requested size.")
        h.comp.features.filletFeatures.add.side_effect = boom
        self.assertTrue(h.validate())
        F.futil.ui.messageBox.reset_mock()
        h.execute()
        text = F.futil.ui.messageBox.call_args[0][0]
        self.assertIn("G2 fillet instead", text)
        self.assertIn("doesn't fit", text)
        self.assertNotIn("ASM_", text)

    def test_all_sweeps_fail_falls_back_to_g2(self):
        h, inputs = self.open()
        h.comp.sweep_fails = lambda ent: True
        self.assertTrue(h.validate())
        F.futil.ui.messageBox.reset_mock()
        h.execute()
        feats = h.comp.features
        res = h.dlg.res[0]
        self.assertEqual(feats.of(F.Sweep), [])
        self.assertTrue(all(not p.isValid for p in h.comp.timeline.items if isinstance(p, F.Plane)))
        fillets = feats.of(F.TimelineFeature, "fillet")
        self.assertEqual(len(fillets), 1)
        es = fillets[0].input.sets[0]
        self.assertEqual(es.continuity, "G2")
        self.assertEqual(len(es.edges), sum(len(lp) for pk in res["pockets"] for lp in pk["loops"]))
        self.assertIn("G2 fillet instead", F.futil.ui.messageBox.call_args[0][0])

    def test_some_sweeps_fail(self):
        # the whole pockets (6 mm across: 4 sides, 4 corner arcs) sweep; the cut ones fall back
        h, inputs = self.open()

        def cut(ent):
            chain = ent.parentSketch.chain(ent)
            x0, y0, x1, y1 = W.bbox([q for c in chain for q in c.polyline()])
            return len(chain) != 8 or abs(x1 - x0 - 0.6) > 1e-6 or abs(y1 - y0 - 0.6) > 1e-6
        h.comp.sweep_fails = cut
        self.assertTrue(h.validate())
        h.execute()
        feats = h.comp.features
        res = h.dlg.res[0]
        self.assertEqual(len(feats.of(F.Sweep)), 1)
        fillet = feats.of(F.TimelineFeature, "fillet")[0]
        edge_curves = {id(e.curve) for e in fillet.input.sets[0].edges}
        whole = sum(1 for pk in res["pockets"] if pk["full"])
        cut = [lp for pk in res["pockets"] if not pk["full"] for lp in pk["loops"]]
        self.assertEqual(len(edge_curves), sum(len(lp) for lp in cut))
        self.assertEqual(feats.of(F.TimelineFeature, "base")[0].bodies.count, whole - 1)

    def test_radius_and_g2(self):
        for name, continuity in (("Radius", None), ("Curvature (G2)", "G2")):
            h, sk, ext, feats = self.make(bottom=name)
            res = h.dlg.res[0]
            self.assertEqual(feats.of(F.Sweep), [])
            fillets = feats.of(F.TimelineFeature, "fillet")
            self.assertEqual(len(fillets), 1)
            es = fillets[0].input.sets[0]
            self.assertEqual((es.continuity, es.radius), (continuity, "0.8 mm"))
            self.assertTrue(fillets[0].input.isRollingBallCorner)
            self.assertEqual(len(es.edges), sum(len(lp) for pk in res["pockets"] for lp in pk["loops"]))
            self.assertIs(h.comp.timeline.groups[0].members[-1], fillets[0])

    def test_extruded_ribs_blend_into_the_pockets(self):
        h, sk, ext, feats = self.make(op="Extrude ribs", extrude_as="Join")
        res = h.dlg.res[0]
        in_pocket = W.pocket_finder(res)
        sweeps = feats.of(F.Sweep)
        self.assertTrue(sweeps)
        for sw in sweeps:
            psk = sw.input.profile.parentSketch
            ctrl = [F.xy(q) for q in [c for c in psk.curves if c.kind == "cspline"][0].ctrl]
            floor = F.v3(psk.sketchToModelSpace(F.Pt(*ctrl[0], 0.0)))
            wall = F.v3(psk.sketchToModelSpace(F.Pt(*ctrl[-1], 0.0)))
            self.assertAlmostEqual(floor[2], 0.0, places=9)  # on the body the ribs stand on
            self.assertGreater(wall[2], 0.0)                 # up the rib
            self.assertTrue(in_pocket(floor[:2]))
        # a radius goes on the ribs' bottom edges along the pockets, not along the area's outline
        h, sk, ext, feats = self.make(op="Extrude ribs", extrude_as="Join", bottom="Radius")
        res = h.dlg.res[0]
        es = feats.of(F.TimelineFeature, "fillet")[0].input.sets[0]
        self.assertEqual(len(es.edges), sum(len(pk["loops"][0]) for pk in res["pockets"]))

    def test_edit_rebuilds_the_bottom(self):
        h, sk, ext, feats = self.make()
        old = list(h.comp.timeline.groups[0].members[2:])
        inputs = h.open(preselected=[ext])
        self.assertEqual(inputs.itemById("bottom").selectedItem.name, G3)
        inputs.itemById("rib").expression = "2 mm"
        h.changed("rib")
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.execute()
        self.assertTrue(all(not x.isValid for x in old))
        members = h.comp.timeline.groups[0].members
        self.assertEqual(members[:2], [sk, ext])
        self.assertTrue(all(x.isValid for x in members))
        self.assertTrue(any(isinstance(x, F.Sweep) for x in members))
        self.assertEqual(h.comp.timeline.markerPosition, h.comp.timeline.count)
        self.assertEqual(ext.edits, 1)

    def test_checks(self):
        h, inputs = self.open(depth="1 mm", radius="1 mm")
        self.assertFalse(h.validate())
        self.assertIn("more than the depth", inputs.itemById("info").formattedText)
        h, inputs = self.open(corner="0 mm")
        self.assertTrue(h.validate())
        h.preview()
        self.assertIn("tighter than the blend", inputs.itemById("info").formattedText)
        h, inputs = self.open()
        inputs.itemById("extent").pick("Through all")
        h.changed("extent")
        self.assertFalse(inputs.itemById("bottom").isVisible)
        h, inputs = self.open(op="Extrude holes", extrude_as="New body")
        self.assertFalse(inputs.itemById("bottom").isVisible)


if __name__ == "__main__":
    unittest.main()
