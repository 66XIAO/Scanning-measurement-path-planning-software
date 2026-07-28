import unittest

try:
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
    from geometry import segment_model
    OCC_AVAILABLE = True
except ImportError:
    OCC_AVAILABLE = False


@unittest.skipUnless(OCC_AVAILABLE, "pythonOCC is not available in this test environment")
class SegmentationDiagnosticsTests(unittest.TestCase):
    def test_box_segmentation_preserves_area(self):
        shape = BRepPrimAPI_MakeBox(20.0, 15.0, 10.0).Shape()
        patches, diagnostics = segment_model(
            shape, 2, 2, return_diagnostics=True)
        self.assertTrue(patches)
        self.assertGreater(diagnostics["split_attempts"], 0)
        self.assertAlmostEqual(diagnostics["area_ratio"], 1.0, places=6)
        self.assertFalse(any("Area conservation failed" in warning
                             for warning in diagnostics["warnings"]))

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
        self.assertGreaterEqual(diagnostics["area_rejected_splits"], 1)


if __name__ == "__main__":
    unittest.main()
