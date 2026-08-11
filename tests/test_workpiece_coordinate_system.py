import math
import unittest

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox

from geometry import (
    calculate_workpiece_coordinate_size,
    display_workpiece_coordinate_system,
)
from renderer import LayerVisibility, render_scene
from state import AppState


class _FakeContext:
    def __init__(self, events=None):
        self.events = events if events is not None else []
        self.displayed = []
        self.deactivated = []

    def Display(self, obj, update):
        self.displayed.append((obj, update))
        self.events.append("display_workpiece_frame")

    def Deactivate(self, obj):
        self.deactivated.append(obj)
        self.events.append("deactivate_workpiece_frame")

    def SetAutoActivateSelection(self, enabled):
        raise AssertionError("The workpiece frame must not change global selection")


class _FakeDisplay:
    def __init__(self):
        self.events = []
        self.Context = _FakeContext(self.events)

    def EraseAll(self):
        self.events.append("erase_all")

    def FitAll(self):
        self.events.append("fit_all")

    def Repaint(self):
        self.events.append("repaint")


class WorkpieceCoordinateSystemTests(unittest.TestCase):
    def test_axis_size_tracks_model_bounding_box(self):
        shape = BRepPrimAPI_MakeBox(100.0, 200.0, 300.0).Shape()
        actual = calculate_workpiece_coordinate_size(shape)
        expected = math.sqrt(100.0 ** 2 + 200.0 ** 2 + 300.0 ** 2) * 0.08
        self.assertAlmostEqual(actual, expected, places=5)

    def test_frame_is_cad_origin_xyz_infinite_and_not_selectable(self):
        display = _FakeDisplay()
        trihedron = display_workpiece_coordinate_system(display, size=25.0)

        self.assertIsNotNone(trihedron)
        self.assertEqual(display.Context.displayed, [(trihedron, False)])
        self.assertEqual(display.Context.deactivated, [trihedron])
        self.assertTrue(trihedron.IsInfinite())
        self.assertAlmostEqual(trihedron.Size(), 25.0)

        placement = trihedron.Component()
        origin = placement.Location()
        z_axis = placement.Direction()
        x_axis = placement.XDirection()
        self.assertEqual((origin.X(), origin.Y(), origin.Z()), (0.0, 0.0, 0.0))
        self.assertEqual((x_axis.X(), x_axis.Y(), x_axis.Z()), (1.0, 0.0, 0.0))
        self.assertEqual((z_axis.X(), z_axis.Y(), z_axis.Z()), (0.0, 0.0, 1.0))

    def test_reset_clears_handle_without_changing_visibility_preference(self):
        app_state = AppState()
        app_state.show_workpiece_coordinate_system = False
        app_state.workpiece_coordinate_system_size = 25.0
        app_state.workpiece_coordinate_system_objects.append(object())

        app_state.reset_all()

        self.assertEqual(app_state.workpiece_coordinate_system_objects, [])
        self.assertEqual(app_state.workpiece_coordinate_system_size, 0.0)
        self.assertFalse(app_state.show_workpiece_coordinate_system)

    def test_scene_draws_frame_after_fit_all_and_tracks_handle(self):
        display = _FakeDisplay()
        shape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape()
        workpiece_handles = []

        render_scene(
            display=display,
            vis=LayerVisibility(model=False, workpiece_coordinate_system=True),
            current_shape=shape,
            current_faces=[],
            face_centers=[],
            face_normals=[],
            center_view_points=[],
            view_points=[],
            optimal_viewpoints=[],
            optimal_path=[],
            sensor_volumes_list=[],
            face_obbs=[],
            workpiece_coordinate_system_size=12.0,
            workpiece_coordinate_system_objects=workpiece_handles,
            coordinate_systems=[],
            optimal_path_objects=[],
            sensor_volume_objects=[],
            obb_visualizations=[],
            fit_all=True,
        )

        self.assertEqual(len(workpiece_handles), 1)
        self.assertLess(
            display.events.index("fit_all"),
            display.events.index("display_workpiece_frame"))
        self.assertEqual(display.events[-1], "repaint")


if __name__ == "__main__":
    unittest.main()
