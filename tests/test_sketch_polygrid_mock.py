"""Drives the Sketch Polygrid command against fake Fusion objects: make a grid, check the sketch it
draws (closed loops, shared end points, stored settings), then edit it. No Fusion needed.
Run: python -m unittest discover tests
"""

import contextlib
import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_fakes as F  # noqa: E402

from GX_tool.commands import sketch_polygrid as cmd  # noqa: E402
from GX_tool.commands import polygrid_dialog as D  # noqa: E402
W = F.W


def sketch_data(sk):
    return json.loads(sk.attributes.itemByName("GX_tool", D.SKETCH_ATTR).value)


class SketchPolygrid(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        F.fake_fusion(self.stack)

    def tearDown(self):
        self.stack.close()

    def harness(self, **kw):
        return F.Harness(self.stack, cmd, **kw)

    def run_create(self, h, mode=None, shape="Hexagon", corner=None, style=None, area=None):
        inputs = h.open()
        if mode:
            inputs.itemById("mode").pick(mode)
        if corner:
            inputs.itemById("corner").expression = corner
            h.changed("corner")
            self.assertTrue(inputs.itemById("cornerStyle").isVisible)
            inputs.itemById("cornerStyle").pick(style)
            h.changed("cornerStyle")
        inputs.itemById("shape").pick(shape)
        h.changed("shape")
        inputs.itemById("side").expression = "6 mm"
        inputs.itemById("rib").expression = "1.5 mm"
        h.changed("rib")
        for e in area or [h.profile]:
            inputs.itemById("area").addSelection(e)
        h.changed("area")
        self.assertTrue(h.validate(), inputs.itemById("info").formattedText)
        h.preview()
        self.assertTrue(h.graphics.get(D.PREVIEW_KEY))
        self.assertIn("pockets", inputs.itemById("info").formattedText)
        h.execute()
        self.assertEqual(len(h.comp.made), 1)
        return h.comp.made[0]

    def test_create_and_edit(self):
        h = self.harness()
        sk = self.run_create(h)
        self.assertEqual(sk.name, "Sketch Polygrid")
        self.assertGreater(len(sk.curves), 50)
        self.assertTrue(all(c.isFixed for c in sk.curves))
        F.check_closed(self, sk)
        kinds = {c.kind for c in sk.curves}
        self.assertTrue({"line", "arc", "spline"} <= kinds, kinds)
        data = sketch_data(sk)
        self.assertEqual((data["shape"], data["src"], data["area"]), ("hexagon", h.src.entityToken, {"type": "profiles"}))
        self.assertTrue(W.area_contains(W.prepare_area(h.profile.loops), tuple(data["seeds"][0])))
        n_before = len([c for c in sk.curves if c.isValid])

        # edit: start with one of the grid's curves selected
        inputs = h.open(preselected=[sk.curves[0]])
        self.assertIs(h.dlg.edit, sk)
        self.assertEqual(inputs.itemById("area").sel, [h.profile])
        self.assertEqual(inputs.itemById("shape").selectedItem.name, "Hexagon")
        self.assertTrue(inputs.itemById("editing").isVisible)
        inputs.itemById("rib").expression = "4 mm"
        h.changed("rib")
        self.assertEqual(inputs.itemById("border").expression, "4 mm")  # border follows the rib
        self.assertTrue(h.validate())
        h.preview()
        self.assertFalse(sk.isVisible)
        h.execute()
        self.assertTrue(sk.isVisible)
        self.assertEqual(len(h.comp.made), 1)  # same sketch, redrawn
        self.assertNotEqual(len([c for c in sk.curves if c.isValid]), n_before)
        F.check_closed(self, sk)
        self.assertEqual(sketch_data(sk)["rib"], "4 mm")

    def test_outline_curves(self):
        # the area given as its outline curves: same grid as from the profile
        h = self.harness()
        sk = self.run_create(h, area=h.outline)
        data = sketch_data(sk)
        self.assertEqual(data["area"]["type"], "curves")
        self.assertEqual(len(data["area"]["curves"]), len(h.outline))
        n_curves = len([c for c in sk.curves if c.isValid])
        h2 = self.harness()
        sk2 = self.run_create(h2, area=[h2.outline[0], h2.outline[3], h2.outline[1], h2.outline[2], h2.outline[4],
                                        h2.outline[5], h2.outline[6]])  # any order
        self.assertEqual(len([c for c in sk2.curves if c.isValid]), n_curves)
        inputs = h.open(preselected=[sk])  # edit: the outline is picked again
        self.assertEqual(inputs.itemById("area").sel, h.outline)

    def test_area_sketch_on_a_face(self):
        for at_end, face_found in ((True, True), (False, True), (True, False)):
            h = self.harness()
            h.src.on_face = True
            tl = h.comp.timeline
            if not at_end:
                tl.items.append("Fillet1")
                tl.markerPosition = tl.count - 1  # rolled back before the fillet
            if not face_found:
                h.design.findEntityByToken.side_effect = lambda t: []
            marker = tl.markerPosition
            sk = self.run_create(h)
            self.assertIsInstance(sk.plane_from, F.Face)
            if face_found:  # at the marker, on the same face
                self.assertEqual(tl.items.index(sk), marker)
            else:  # the face isn't there: just before the area's sketch
                self.assertEqual(tl.items.index(sk), tl.items.index(h.src) - 1)
            self.assertEqual(tl.markerPosition, tl.count if at_end else tl.count - 1)
            F.check_closed(self, sk)

    def test_g3_corners(self):
        h = self.harness()
        sk = self.run_create(h, shape="Square", corner="1 mm", style="G3 curve (7 points)")
        corners = [c for c in sk.curves if c.kind == "cspline"]
        self.assertGreater(len(corners), 20)
        self.assertTrue(all(len(c.ctrl) == 7 for c in corners))
        F.check_closed(self, sk)
        data = sketch_data(sk)
        self.assertEqual((data["corner"], data["cornerStyle"]), ("1 mm", "g3"))
        inputs = h.open(preselected=[sk])  # edit: the style comes back
        self.assertEqual(inputs.itemById("cornerStyle").selectedItem.name, "G3 curve (7 points)")
        self.assertEqual(inputs.itemById("corner").expression, "1 mm")
        self.assertTrue(h.validate())

    def test_ribs_mode_draws_outline(self):
        h = self.harness()
        sk = self.run_create(h, mode="Ribs (pockets + area outline)", shape="Square")
        circles = [c for c in sk.curves if c.kind == "circle"]
        self.assertTrue(any(abs(c.radius - 1.0) < 1e-12 for c in circles))  # the hole, copied
        F.check_closed(self, sk)

    def test_arcs_from_plain_points(self):
        h = self.harness(arcs_take_points=False)
        sk = self.run_create(h)
        self.assertFalse(F.fg._arc_by_points[0])
        self.assertGreater(sk.merges, 0)
        F.check_closed(self, sk)

    def test_symmetry_line(self):
        h = self.harness(offset=(5.0, -2.0))
        inputs = h.open()
        inputs.itemById("area").addSelection(h.profile)
        h.changed("area")
        # a vertical line through x = 5 in sketch space (model x = 10)
        inputs.itemById("vline").addSelection(F.FakeLine((10, -5), (10, 8)))
        h.changed("vline")
        self.assertTrue(h.validate())
        res, err = h.dlg.compute()
        cents = [W.bbox([q for p in lp for q in W.prim_polyline(p)])
                 for lp in (pk["loops"][0] for pk in res["pockets"] if pk["full"])]
        xs = sorted(round((b[0] + b[2]) / 2 - 5, 6) + 0.0 for b in cents)
        self.assertEqual(xs, sorted(round(-x, 6) + 0.0 for x in xs))

    def test_size_by_switch_keeps_pocket(self):
        h = self.harness()
        inputs = h.open()
        inputs.itemById("shape").pick("Hexagon")
        h.changed("shape")
        inputs.itemById("side").expression = "10 mm"
        inputs.itemById("sizeBy").pick("Inscribed circle diameter")
        h.changed("sizeBy")
        self.assertAlmostEqual(inputs.itemById("inscribed").value, 1.0 * math.sqrt(3), places=9)
        self.assertFalse(inputs.itemById("side").isVisible)
        self.assertTrue(inputs.itemById("inscribed").isVisible)

    def test_errors(self):
        h = self.harness()
        inputs = h.open()
        self.assertFalse(h.validate())
        inputs.itemById("area").addSelection(h.profile)
        inputs.itemById("rib").expression = "0 mm"
        self.assertFalse(h.validate())
        self.assertIn("above zero", inputs.itemById("info").formattedText)
        inputs.itemById("rib").expression = "2 mm"
        inputs.itemById("area").addSelection(h.outline[0])
        self.assertFalse(h.validate())
        self.assertIn("not both", inputs.itemById("info").formattedText)


if __name__ == "__main__":
    unittest.main()
