import unittest

try:
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.BRepClass import BRepClass_FaceClassifier
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_ON, TopAbs_WIRE
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopoDS import TopoDS_Compound
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
    from geometry import generate_face_obb, segment_model
    from surface_segmentation import is_surface_patch
    OCC_AVAILABLE = True
except ImportError:
    OCC_AVAILABLE = False


def _face_with_most_wires(shape):
    selected = None
    selected_wire_count = -1
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = explorer.Current()
        wire_count = 0
        wire_explorer = TopExp_Explorer(face, TopAbs_WIRE)
        while wire_explorer.More():
            wire_count += 1
            wire_explorer.Next()
        if wire_count > selected_wire_count:
            selected = face
            selected_wire_count = wire_count
        explorer.Next()
    return selected, selected_wire_count


def _single_face_compound(face):
    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    builder.Add(compound, face)
    return compound


@unittest.skipUnless(OCC_AVAILABLE, "pythonOCC is not available in this test environment")
class SurfaceSegmentationTests(unittest.TestCase):
    def test_auto_preserves_equal_param_for_untrimmed_shape(self):
        shape = BRepPrimAPI_MakeBox(20.0, 15.0, 10.0).Shape()
        patches, diagnostics = segment_model(
            shape, 2, 2, return_diagnostics=True, strategy="auto")
        self.assertEqual(diagnostics["strategy"], "equal_param")
        self.assertEqual(len(patches), 24)
        self.assertFalse(any(is_surface_patch(patch) for patch in patches))

    def test_trimmed_face_uses_connected_mesh_grid_regions(self):
        plate = BRepPrimAPI_MakeBox(200.0, 120.0, 2.0).Shape()
        axis = gp_Ax2(gp_Pnt(100.0, 60.0, -1.0), gp_Dir(0.0, 0.0, 1.0))
        cutter = BRepPrimAPI_MakeCylinder(axis, 18.0, 4.0).Shape()
        shape = BRepAlgoAPI_Cut(plate, cutter).Shape()
        face, wire_count = _face_with_most_wires(shape)
        self.assertEqual(wire_count, 2)

        patches, diagnostics = segment_model(
            _single_face_compound(face), 8, 6,
            return_diagnostics=True, strategy="auto",
            mesh_linear_deflection=2.0)

        self.assertEqual(diagnostics["strategy"], "mesh_grid")
        self.assertEqual(diagnostics["occupied_grid_cells"], 48)
        self.assertGreater(diagnostics["connected_component_splits"], 0)
        self.assertAlmostEqual(diagnostics["area_ratio"], 1.0, delta=0.01)
        self.assertTrue(patches)
        self.assertTrue(all(is_surface_patch(patch) for patch in patches))
        for patch in patches:
            classifier = BRepClass_FaceClassifier(
                patch.source_face, patch.center, 1e-6, True)
            self.assertIn(classifier.State(), (TopAbs_IN, TopAbs_ON))
            self.assertAlmostEqual(patch.normal.Magnitude(), 1.0, places=6)
            self.assertGreater(patch.area, 0.0)

        obb, box_shape, _ = generate_face_obb(patches[0])
        self.assertIsNotNone(obb)
        self.assertIsNotNone(box_shape)


if __name__ == "__main__":
    unittest.main()
