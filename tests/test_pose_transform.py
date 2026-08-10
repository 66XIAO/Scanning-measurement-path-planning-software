import csv
import json
import math
import os
import tempfile
import unittest

import numpy as np

from pose_transform import (
    load_extrinsic_config, parse_extrinsic_config, pose_matrix, transform_pose_records,
    transform_scanner_pose_to_tool,
)
from export_utils import write_speed_plan_csv
from speed_planning_core import plan_speed_profile


def config_data(matrix=None, status="validated"):
    return {
        "schema_version": "1.1",
        "config_id": "test_calibration",
        "calibration_status": status,
        "mapping_mode": "separate_tool_frame",
        "source_pose_frame": "scanner",
        "command_pose_frame": "tool",
        "length_unit": "mm",
        "quaternion_order": "wxyz",
        "T_tool_scanner": matrix or np.eye(4).tolist(),
    }


def pose(z=500.0):
    return {"index": 1, "x": 10.0, "y": 20.0, "z": z,
            "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}


class PoseTransformTests(unittest.TestCase):
    def test_shipped_robodk_station_config(self):
        path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "calibration",
            "robodk_ur10_creaform_station.json"))
        config = load_extrinsic_config(path)
        self.assertEqual(config.mapping_mode, "robodk_tcp_is_scanner")
        self.assertEqual(config.robodk_tool_name, "Creaform MetraSCAN")
        self.assertTrue(config.validated)
        self.assertFalse(config.physically_validated)

    def test_identity_extrinsic_preserves_pose(self):
        config = parse_extrinsic_config(config_data())
        command = transform_scanner_pose_to_tool(pose(), config)
        np.testing.assert_allclose(
            [command[key] for key in ("x", "y", "z", "qw", "qx", "qy", "qz")],
            [10, 20, 500, 1, 0, 0, 0], atol=1e-9)

    def test_tool_offset_is_inverted(self):
        matrix = np.eye(4)
        matrix[2, 3] = 100.0
        config = parse_extrinsic_config(config_data(matrix.tolist()))
        command = transform_scanner_pose_to_tool(pose(), config)
        self.assertAlmostEqual(command["z"], 400.0)
        np.testing.assert_allclose(
            pose_matrix(command) @ config.matrix, pose_matrix(pose()), atol=1e-9)

    def test_rotation_round_trip(self):
        angle = math.pi / 2
        matrix = np.eye(4)
        matrix[:3, :3] = [[math.cos(angle), -math.sin(angle), 0],
                          [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
        config = parse_extrinsic_config(config_data(matrix.tolist()))
        command = transform_scanner_pose_to_tool(pose(), config)
        np.testing.assert_allclose(
            pose_matrix(command) @ config.matrix, pose_matrix(pose()), atol=1e-9)

    def test_invalid_rotation_is_rejected(self):
        data = config_data()
        data["T_tool_scanner"][0][0] = 2.0
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            parse_extrinsic_config(data)

    def test_status_controls_validation_flag_and_provenance(self):
        config = parse_extrinsic_config(config_data(status="example_only"))
        commands, metadata = transform_pose_records([pose()], config)
        self.assertFalse(config.validated)
        self.assertFalse(metadata["extrinsic_validated"])
        self.assertEqual(metadata["source_pose_records"][0], pose())
        self.assertEqual(len(commands), 1)

    def test_separate_flange_tool_is_optional_for_planning_and_preserved(self):
        legacy = parse_extrinsic_config(config_data())
        commands, legacy_metadata = transform_pose_records([pose()], legacy)
        self.assertEqual(len(commands), 1)
        self.assertIsNone(legacy.t_flange_tool)
        self.assertIsNone(legacy_metadata["T_flange_tool"])

        data = config_data()
        data["T_flange_tool"] = np.eye(4).tolist()
        config = parse_extrinsic_config(data)
        _commands, metadata = transform_pose_records([pose()], config)
        self.assertEqual(config.to_dict()["T_flange_tool"], np.eye(4).tolist())
        self.assertEqual(metadata["T_flange_tool"], np.eye(4).tolist())

    def test_robodk_scanner_tcp_mode_does_not_double_transform(self):
        data = config_data(status="station_verified")
        data.update({
            "mapping_mode": "robodk_tcp_is_scanner",
            "command_pose_frame": "scanner_tcp",
            "T_tool_scanner": None,
            "T_flange_scanner": [
                [0, 0, 1, 90], [0, 1, 0, 0], [-1, 0, 0, 300], [0, 0, 0, 1]],
            "robodk_station_name": "UR10_小曲面_test",
            "robodk_robot_name": "UR10",
            "robodk_tool_name": "Creaform MetraSCAN",
        })
        config = parse_extrinsic_config(data)
        commands, metadata = transform_pose_records([pose()], config)
        np.testing.assert_allclose(pose_matrix(commands[0]), pose_matrix(pose()), atol=1e-9)
        self.assertFalse(metadata["extrinsic_applied"])
        self.assertTrue(metadata["extrinsic_validated"])
        self.assertFalse(metadata["physical_calibration_validated"])
        self.assertEqual(metadata["command_pose_frame"], "scanner_tcp")

    def test_csv_preserves_source_and_command_pose_with_metadata(self):
        matrix = np.eye(4)
        matrix[2, 3] = 100.0
        config = parse_extrinsic_config(config_data(matrix.tolist()))
        sources = [pose(500.0), dict(pose(600.0), index=2)]
        commands, metadata = transform_pose_records(sources, config)
        result = plan_speed_profile(commands)
        result.diagnostics.update(metadata)
        path = os.path.join(tempfile.gettempdir(), "extrinsic_pose_contract.csv")
        write_speed_plan_csv(path, result)
        with open(path, encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(float(rows[0]["Z"]), 400.0)
        self.assertEqual(float(rows[0]["source_Z"]), 500.0)
        self.assertEqual(rows[0]["command_pose_frame"], "tool")
        with open(path + ".metadata.json", encoding="utf-8") as stream:
            sidecar = json.load(stream)
        self.assertEqual(sidecar["diagnostics"]["extrinsic_config_id"], "test_calibration")


if __name__ == "__main__":
    unittest.main()
