import unittest
from unittest.mock import patch

import numpy as np

import robodk_bridge
from pose_transform import transform_workpiece_pose_to_base


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]

TOOL_POSE = [
    [0.0, 0.0, 1.0, 80.076],
    [0.0, -1.0, 0.0, 1.743],
    [1.0, 0.0, 0.0, 307.0],
    [0.0, 0.0, 0.0, 1.0],
]


class FakeMatrix:
    def __init__(self, rows):
        self.rows = rows

    def __getitem__(self, key):
        row, column = key
        return self.rows[row][column]


class FakeItem:
    def __init__(self, name, item_type, pose_abs=None, tool_pose=None,
                 parent=None, valid=True):
        self.name = name
        self.item_type = item_type
        self.pose_abs = pose_abs or IDENTITY
        self.tool_pose = tool_pose or IDENTITY
        self.parent = parent
        self.valid = valid

    def Valid(self):
        return self.valid

    def Name(self):
        return self.name

    def Parent(self):
        return self.parent or FakeItem("", -1, valid=False)

    def PoseAbs(self):
        return FakeMatrix(self.pose_abs)

    def PoseTool(self):
        return FakeMatrix(self.tool_pose)


class FakeRDK:
    def __init__(self, wrong_tool_parent=False):
        self.station = FakeItem("Current Station", 99)
        self.base = FakeItem("UR10 Base", 2, pose_abs=[
            [1, 0, 0, 0], [0, 1, 0, 1900],
            [0, 0, 1, 0], [0, 0, 0, 1]])
        self.robot = FakeItem("UR10", 1, parent=self.base)
        self.frame = FakeItem("Frame 2", 2, pose_abs=[
            [1, 0, 0, 437], [0, 1, 0, 1190],
            [0, 0, 1, 150], [0, 0, 0, 1]])
        tool_parent = self.base if wrong_tool_parent else self.robot
        self.tool = FakeItem(
            "Creaform MetraSCAN", 3, tool_pose=TOOL_POSE,
            parent=tool_parent)
        self.items = [self.robot, self.frame, self.tool, self.base]

    def ItemList(self, item_type):
        return [item for item in self.items if item.item_type == item_type]

    def Item(self, name, item_type):
        for item in self.ItemList(item_type):
            if item.Name() == name:
                return item
        return FakeItem(name, item_type, valid=False)

    def ActiveStation(self):
        return self.station


def fake_api(rdk):
    return {
        "Robolink": lambda: rdk,
        "ITEM_TYPE_ROBOT": 1,
        "ITEM_TYPE_FRAME": 2,
        "ITEM_TYPE_TOOL": 3,
        "ITEM_TYPE_PROGRAM": 4,
        "ITEM_TYPE_TARGET": 5,
    }


class RoboDKStationCaptureTests(unittest.TestCase):
    def test_capture_reads_absolute_station_and_base_relationships(self):
        rdk = FakeRDK()
        with patch.object(
                robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            config = robodk_bridge.capture_robodk_station_mapping(
                captured_at_utc="2026-08-16T00:00:00Z")

        self.assertEqual(config.schema_version, "1.2")
        self.assertEqual(config.calibration_status, "station_verified")
        self.assertEqual(config.robodk_base_name, "UR10 Base")
        np.testing.assert_allclose(config.t_station_robot_base, [
            [1, 0, 0, 0], [0, 1, 0, 1900],
            [0, 0, 1, 0], [0, 0, 0, 1]])
        np.testing.assert_allclose(config.t_station_reference_frame, [
            [1, 0, 0, 437], [0, 1, 0, 1190],
            [0, 0, 1, 150], [0, 0, 0, 1]])
        np.testing.assert_allclose(config.t_base_workpiece, [
            [1, 0, 0, 437], [0, 1, 0, -710],
            [0, 0, 1, 150], [0, 0, 0, 1]])
        np.testing.assert_allclose(config.t_flange_scanner, TOOL_POSE)

        base_pose = transform_workpiece_pose_to_base({
            "index": 1, "x": 10, "y": 20, "z": 30,
            "qw": 1, "qx": 0, "qy": 0, "qz": 0,
        }, config)
        self.assertEqual(
            [base_pose[key] for key in ("x", "y", "z")],
            [447.0, -690.0, 180.0])

    def test_capture_rejects_tool_attached_to_another_parent(self):
        rdk = FakeRDK(wrong_tool_parent=True)
        with patch.object(
                robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "not attached to robot"):
                robodk_bridge.capture_robodk_station_mapping(
                    captured_at_utc="2026-08-16T00:00:00Z")


if __name__ == "__main__":
    unittest.main()
