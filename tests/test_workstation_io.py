import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Dir, gp_Pnt, gp_Vec

from state import AppState, ViewpointRecord
from surface_segmentation import SurfacePatch
from workstation_io import (
    MANIFEST_NAME, WorkstationError, capture_workstation,
    load_workstation, save_workstation,
)


class WorkstationRoundTripTests(unittest.TestCase):
    def _state(self, source_path):
        state = AppState()
        state.model_source_path = str(source_path)
        state.model_source_name = source_path.name
        state.model_source_format = "step"
        state.current_shape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape()
        source_face = BRepPrimAPI_MakeBox(4.0, 5.0, 6.0).Shape()
        patch = SurfacePatch(
            source_face=source_face, source_face_index=3,
            grid_u_index=1, grid_v_index=2, component_index=0,
            center=gp_Pnt(1.0, 2.0, 3.0), normal=gp_Vec(0.0, 0.0, 1.0),
            area=12.5, triangle_count=7,
            bounds=(0.0, 0.0, 0.0, 2.0, 4.0, 1.0),
            representative_uv=(0.25, 0.75))
        state.current_faces = [patch]
        state.surface_patches = [patch]
        state.segmentation_parameters = {"u": 9, "v": 11, "strategy": "mesh_grid"}
        state.face_centers = [gp_Pnt(1.0, 2.0, 3.0)]
        state.face_normals = [gp_Vec(0.0, 0.0, 1.0)]
        state.optimal_viewpoints = [gp_Pnt(1.0, 2.0, 83.0)]
        state.optimal_viewpoints_with_pose = [
            (state.optimal_viewpoints[0], (1.0, 0.0, 0.0, 0.0))]
        state.optimal_viewpoint_records = [ViewpointRecord(
            point=state.optimal_viewpoints[0], pose=(1.0, 0.0, 0.0, 0.0),
            face_index=0, kind="optimal", global_index=0, candidate_index=0)]
        state.optimal_path = [0]
        state.last_path_algorithm = "sequential"
        state.last_path_length = 0.0
        sensor_shape = BRepPrimAPI_MakeBox(2.0, 3.0, 4.0).Shape()
        vertices = [gp_Pnt(float(i), 0.0, 0.0) for i in range(8)]
        from OCC.Display.OCCViewer import rgb_color
        state.sensor_volumes_list = [(sensor_shape, vertices, rgb_color(0.2, 0.8, 0.2))]
        state.face_obbs = [Bnd_OBB(
            gp_Pnt(1.0, 2.0, 3.0), gp_Dir(1.0, 0.0, 0.0),
            gp_Dir(0.0, 1.0, 0.0), gp_Dir(0.0, 0.0, 1.0),
            1.0, 2.0, 3.0)]
        state.collision_results = [False]
        state.sensor_volumes_created = True
        state.obb_boxes_generated = True
        state.collision_detection_executed = True
        state.record_operation("fixture", {"u": 9}, summary="round trip")
        return state

    def test_complete_state_round_trip_is_independent_of_source_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "原始模型.step"
            source.write_bytes(b"ISO-10303-21; test fixture")
            state = self._state(source)
            archive, _manifest = save_workstation(
                root / "会话.swstation",
                capture_workstation(
                    state, {"collision_detection_enabled": True},
                    {"eye": [1.0, 2.0, 3.0], "scale": 5.0}))
            source.unlink()

            loaded = load_workstation(archive)

            self.assertFalse(loaded.state.current_shape.IsNull())
            self.assertEqual(len(loaded.state.surface_patches), 1)
            self.assertEqual(loaded.state.surface_patches[0].source_face_index, 3)
            self.assertEqual(loaded.state.segmentation_parameters["u"], 9)
            self.assertEqual(loaded.state.optimal_path, [0])
            self.assertEqual(loaded.state.collision_results, [False])
            self.assertEqual(len(loaded.state.face_obbs), 1)
            self.assertAlmostEqual(loaded.state.face_obbs[0].ZHSize(), 3.0)
            self.assertEqual(loaded.settings["collision_detection_enabled"], True)
            self.assertEqual(loaded.view_state["scale"], 5.0)
            self.assertFalse(loaded.state.workstation_dirty)

    def test_checksum_failure_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "model.step"
            source.write_bytes(b"source")
            archive, _ = save_workstation(
                root / "bad.swstation",
                capture_workstation(self._state(source)))
            corrupt = root / "corrupt.swstation"
            with zipfile.ZipFile(archive, "r") as source_zip, \
                    zipfile.ZipFile(corrupt, "w") as output:
                for info in source_zip.infolist():
                    payload = source_zip.read(info.filename)
                    output.writestr(
                        info, b"{}" if info.filename == "state.json" else payload)
            with self.assertRaises(WorkstationError):
                load_workstation(corrupt)

    def test_upstream_invalidation_clears_stale_downstream_results(self):
        state = AppState()
        state.optimal_viewpoints = [gp_Pnt(1.0, 2.0, 3.0)]
        state.optimal_path = [0]
        state.last_path_algorithm = "greedy"
        state.path_algorithm_diagnostics = {"best_length": 1.0}
        state.last_robodk_import = {"program": "old"}
        state.last_reachability_report = {"reachable": True}
        state.collision_results = [True]

        state.invalidate_after_viewpoints()

        self.assertEqual(state.optimal_viewpoints, [])
        self.assertEqual(state.optimal_path, [])
        self.assertEqual(state.path_algorithm_diagnostics, {})
        self.assertEqual(state.last_robodk_import, {})
        self.assertEqual(state.last_reachability_report, {})
        self.assertEqual(state.collision_results, [])

    def test_unsafe_archive_member_is_rejected_before_extraction(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsafe.swstation"
            manifest = {
                "format": "software_optimizing_workstation",
                "format_version": 1, "members": {}}
            with zipfile.ZipFile(path, "w") as output:
                output.writestr(MANIFEST_NAME, json.dumps(manifest))
                output.writestr("state.json", "{}")
                output.writestr("../escape.txt", "blocked")
            with self.assertRaises(WorkstationError):
                load_workstation(path)

    def test_failed_overwrite_preserves_previous_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "model.step"
            source.write_bytes(b"source")
            state = self._state(source)
            archive, _ = save_workstation(
                root / "atomic.swstation", capture_workstation(state))
            previous = Path(archive).read_bytes()

            with mock.patch("workstation_io._write_shape",
                            side_effect=OSError("simulated write failure")):
                with self.assertRaises(OSError):
                    save_workstation(archive, capture_workstation(state))

            self.assertEqual(Path(archive).read_bytes(), previous)

    def test_newer_format_version_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "newer.swstation"
            manifest = {
                "format": "software_optimizing_workstation",
                "format_version": 99, "members": {}}
            with zipfile.ZipFile(path, "w") as output:
                output.writestr(MANIFEST_NAME, json.dumps(manifest))
                output.writestr("state.json", "{}")
            with self.assertRaises(WorkstationError):
                load_workstation(path)


if __name__ == "__main__":
    unittest.main()
