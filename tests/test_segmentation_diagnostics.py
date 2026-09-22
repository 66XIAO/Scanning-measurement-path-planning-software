import unittest

try:
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
    from geometry import _split_areas_conserve, segment_model
    from cad_io import combine_shapes
    from renderer import _build_patch_edge_compound
    from OCC.Extend.TopologyUtils import TopologyExplorer
    OCC_AVAILABLE = True
except ImportError:
    OCC_AVAILABLE = False


@unittest.skipUnless(OCC_AVAILABLE, "pythonOCC is not available in this test environment")
class SegmentationDiagnosticsTests(unittest.TestCase):
    def test_split_area_tolerance_accepts_step_boolean_noise_only(self):
        source_area = 132433.49978190195
        self.assertTrue(_split_areas_conserve(source_area, 132423.40918445335))
        self.assertFalse(_split_areas_conserve(source_area, source_area * 1.002))

    def test_multiple_reader_roots_are_combined(self):
        first = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
        second = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape()
        compound = combine_shapes([first, second])
        patches, diagnostics = segment_model(
            compound, 1, 1, return_diagnostics=True)
        self.assertEqual(diagnostics["original_face_count"], 12)
        self.assertEqual(len(patches), 12)

    def test_box_segmentation_preserves_area(self):
        shape = BRepPrimAPI_MakeBox(20.0, 15.0, 10.0).Shape()
        patches, diagnostics = segment_model(
            shape, 2, 2, return_diagnostics=True)
        self.assertEqual(len(patches), 24)
        self.assertGreater(diagnostics["split_attempts"], 0)
        self.assertEqual(diagnostics["area_rejected_splits"], 0)
        self.assertAlmostEqual(diagnostics["area_ratio"], 1.0, places=6)
        self.assertFalse(any("Area conservation failed" in warning
                             for warning in diagnostics["warnings"]))

        boundary_shape, edge_count = _build_patch_edge_compound(patches)
        self.assertGreater(edge_count, 0)
        self.assertGreater(len(list(TopologyExplorer(boundary_shape).edges())), 0)

    def test_trimmed_and_periodic_faces_are_diagnosed_without_area_loss(self):
        plate = BRepPrimAPI_MakeBox(20.0, 20.0, 2.0).Shape()
        axis = gp_Ax2(gp_Pnt(10.0, 10.0, -1.0), gp_Dir(0.0, 0.0, 1.0))
        cutter = BRepPrimAPI_MakeCylinder(axis, 3.0, 4.0).Shape()
        shape = BRepAlgoAPI_Cut(plate, cutter).Shape()
        patches, diagnostics = segment_model(
            shape, 2, 2, return_diagnostics=True)
        self.assertTrue(patches)
        self.assertGreaterEqual(diagnostics["periodic_faces"], 1)
        self.assertAlmostEqual(diagnostics["area_ratio"], 1.0, places=5)
        # OCCT releases can accept or reject different intermediate boolean
        # splits; the stable contract is that splitting was attempted and the
        # final surface area was conserved.
        self.assertGreater(diagnostics["split_attempts"], 0)


if __name__ == "__main__":
    unittest.main()
