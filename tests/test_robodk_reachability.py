import unittest
from unittest.mock import patch

import robodk_bridge


def identity(x=0.0):
    return [
        [1.0, 0.0, 0.0, float(x)],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


class FakeMatrix:
    def __init__(self, rows):
        self.rows = [list(row) for row in rows]

    def __getitem__(self, key):
        row, column = key
        return self.rows[row][column]

    def __mul__(self, other):
        return FakeMatrix([
            [sum(self.rows[row][inner] * other.rows[inner][column]
                 for inner in range(4))
             for column in range(4)]
            for row in range(4)
        ])

    def size(self):
        return len(self.rows), len(self.rows[0]) if self.rows else 0

    def tolist(self):
        if not self.rows or not self.rows[0]:
            return []
        return [row[0] for row in self.rows]


class FakeItem:
    def __init__(self, name, item_type, parent=None, pose=None):
        self.name = name
        self.item_type = item_type
        self.parent = parent
        self.pose = FakeMatrix(pose or identity())

    def Valid(self):
        return True

    def Name(self):
        return self.name

    def Parent(self):
        return self.parent

    def PoseAbs(self):
        return self.pose

    def PoseTool(self):
        return self.pose


class FakeRobot(FakeItem):
    def __init__(self, parent):
        super().__init__("UR10", 1, parent=parent)
        self.last_desired = None

    def Joints(self):
        return FakeMatrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    def JointLimits(self):
        lower = FakeMatrix([[-180.0] for _ in range(6)])
        upper = FakeMatrix([[180.0] for _ in range(6)])
        return lower, upper, 0.0

    def SolveIK_All(self, pose):
        self.last_desired = pose
        if pose[0, 3] > 1000.0:
            return FakeMatrix([[]])
        solutions = [
            [10.0, -20.0, 30.0, -40.0, 50.0, -60.0],
            [100.0, -100.0, 100.0, -100.0, 100.0, -100.0],
        ]
        return FakeMatrix([list(row) for row in zip(*solutions)])

    def SolveFK(self, _joints):
        return self.last_desired

    def JointsConfig(self, _joints):
        return FakeMatrix([[0.0], [1.0], [0.0], [0.0]])


class FakeRDK:
    def __init__(self):
        self.station = FakeItem("Station", 99)
        self.base = FakeItem("UR10 Base", 2, pose=identity())
        self.robot = FakeRobot(self.base)
        self.frame = FakeItem("Frame 2", 2, pose=identity(100.0))
        self.tool = FakeItem(
            "Creaform MetraSCAN", 3, parent=self.robot, pose=identity(50.0))
        self.items = [self.robot, self.frame, self.tool]

    def Item(self, name, item_type):
        for item in self.items:
            if item.item_type == item_type and item.Name() == name:
                return item
        invalid = FakeItem(name, item_type)
        invalid.Valid = lambda: False
        return invalid

    def ItemList(self, item_type):
        return [item for item in self.items if item.item_type == item_type]

    def ActiveStation(self):
        return self.station

    def Version(self):
        return "test"


def fake_api(rdk):
    return {
        "Robolink": lambda: rdk,
        "ITEM_TYPE_ROBOT": 1,
        "ITEM_TYPE_FRAME": 2,
        "ITEM_TYPE_TOOL": 3,
        "ITEM_TYPE_PROGRAM": 4,
        "ITEM_TYPE_TARGET": 5,
        "Mat": FakeMatrix,
        "transl": lambda x, y, z: FakeMatrix([
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]),
        "api_source": "fake",
    }


def metadata():
    return {
        "extrinsic_validated": True,
        "physical_calibration_validated": False,
        "tool_mapping_mode": "robodk_tcp_is_scanner",
        "expected_robodk_station_name": "Station",
        "expected_robodk_robot_name": "UR10",
        "expected_robodk_base_name": "UR10 Base",
        "expected_robodk_frame_name": "Frame 2",
        "expected_robodk_tool_name": "Creaform MetraSCAN",
        "T_station_robot_base": identity(),
        "T_station_reference_frame": identity(100.0),
        "T_base_workpiece": identity(100.0),
        "T_flange_scanner": identity(50.0),
        "tool_position_tolerance_mm": 0.01,
        "tool_orientation_tolerance_deg": 0.01,
    }


def pose(index, x):
    return {
        "index": index, "x": x, "y": 0.0, "z": 0.0,
        "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    }


class RoboDKReachabilityTests(unittest.TestCase):
    def test_uses_frame_and_tool_transforms_and_reports_unreachable_points(self):
        rdk = FakeRDK()
        with patch.object(
                robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.analyze_ur10_reachability(
                [pose(1, 200.0), pose(2, 2000.0)], metadata())

        self.assertTrue(report["read_only"])
        self.assertEqual(report["reachable_count"], 1)
        self.assertEqual(report["reachable_indices"], [1])
        self.assertEqual(report["unreachable_indices"], [2])
        self.assertEqual(report["points"][0]["selected_joints_deg"][0], 10.0)
        self.assertEqual(report["points"][0]["T_base_flange"][0][3], 250.0)
        self.assertEqual(report["points"][1]["reason"], "no_ik_solution")

    def test_requires_base_workpiece_before_opening_robodk(self):
        data = metadata()
        data["T_base_workpiece"] = None
        with patch.object(robodk_bridge, "_import_api") as import_api:
            with self.assertRaisesRegex(ValueError, "T_base_workpiece"):
                robodk_bridge.analyze_ur10_reachability(
                    [pose(1, 200.0)], data)
        import_api.assert_not_called()

    def test_rejects_non_ur10_before_opening_robodk(self):
        with patch.object(robodk_bridge, "_import_api") as import_api:
            with self.assertRaisesRegex(ValueError, "only robot UR10"):
                robodk_bridge.analyze_ur10_reachability(
                    [pose(1, 200.0)], metadata(), robot_name="UR5")
        import_api.assert_not_called()


if __name__ == "__main__":
    unittest.main()
