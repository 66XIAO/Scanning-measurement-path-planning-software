import unittest
from unittest.mock import patch

import robodk_bridge
from speed_planning_core import plan_speed_profile
from test_speed_planning_core import sample_records


class FakeMatrix:
    def __init__(self, values=None):
        self.values = values or [[1 if row == column else 0 for column in range(4)]
                                 for row in range(4)]

    def __mul__(self, other):
        return self

    def __getitem__(self, key):
        row, column = key
        return self.values[row][column]


class FakeItem:
    def __init__(self, rdk, name, item_type, valid=True, tool_pose=None):
        self.rdk = rdk
        self.name = name
        self.item_type = item_type
        self.valid = valid
        self.tool_pose = tool_pose

    def Valid(self):
        return self.valid

    def Name(self):
        return self.name

    def Delete(self):
        self.valid = False
        self.rdk.items.pop((self.item_type, self.name), None)

    def setName(self, name):
        self.rdk.items.pop((self.item_type, self.name), None)
        self.name = name
        self.rdk.items[(self.item_type, self.name)] = self

    def setPose(self, _pose):
        pass

    def setAsCartesianTarget(self):
        pass

    def PoseTool(self):
        return FakeMatrix(self.tool_pose)

    def Pose(self):
        return FakeMatrix(self.tool_pose)


class FakeProgram(FakeItem):
    def __init__(self, rdk, name, fail_move=False, fail_commit=False):
        super().__init__(rdk, name, 4)
        self.instructions = []
        self.speed_calls = []
        self.fail_move = fail_move
        self.fail_commit = fail_commit

    def setName(self, name):
        if self.fail_commit and name == "Plan":
            raise RuntimeError("simulated commit rename failure")
        super().setName(name)

    def setFrame(self, _frame):
        self.instructions.append(("Frame", 3, 0, 0, None, None))

    def setTool(self, _tool):
        self.instructions.append(("Tool", 4, 0, 0, None, None))

    def setSpeed(self, *values):
        self.speed_calls.append(values)
        self.instructions.append(("Set Speed", 2, 0, 0, None, None))

    def _move(self, name):
        if self.fail_move:
            raise RuntimeError("simulated move failure")
        self.instructions.append((name, 0, 1, 0, None, None))

    def MoveJ(self, _target):
        self._move("MoveJ")

    def MoveL(self, _target):
        self._move("MoveL")

    def InstructionCount(self):
        return len(self.instructions)

    def Instruction(self, index):
        return self.instructions[index]

    def setParam(self, _instruction_id):
        return "unsupported by legacy API"

    def ShowInstructions(self, _show):
        pass


class FakeRDK:
    def __init__(self, fail_move=False, fail_commit=False):
        self.items = {}
        self.fail_move = fail_move
        self.fail_commit = fail_commit
        for name, item_type in (("UR10", 1), ("Frame 2", 2)):
            self.items[(item_type, name)] = FakeItem(self, name, item_type)
        self.items[(3, "Tool")] = FakeItem(self, "Tool", 3, tool_pose=[
            [0, 0, 1, 90], [0, 1, 0, 0], [-1, 0, 0, 300], [0, 0, 0, 1]])

    def Item(self, name, item_type):
        return self.items.get((item_type, name), FakeItem(self, name, item_type, False))

    def AddProgram(self, name, _robot):
        item = FakeProgram(self, name, self.fail_move, self.fail_commit)
        self.items[(4, name)] = item
        return item

    def AddTarget(self, name, _frame, _robot):
        item = FakeItem(self, name, 5)
        self.items[(5, name)] = item
        return item

    def Render(self, _enabled):
        pass

    def ActiveStation(self):
        return FakeItem(self, "Station", 99)


def fake_api(rdk):
    return {
        "Robolink": lambda: rdk,
        "ITEM_TYPE_ROBOT": 1,
        "ITEM_TYPE_FRAME": 2,
        "ITEM_TYPE_TOOL": 3,
        "ITEM_TYPE_PROGRAM": 4,
        "ITEM_TYPE_TARGET": 5,
        "Mat": lambda values: FakeMatrix(values),
        "transl": lambda *_values: FakeMatrix(),
}


def allow_separate_tool_mapping(result):
    result.diagnostics.update({
        "extrinsic_validated": True,
        "physical_calibration_validated": False,
        "tool_mapping_mode": "separate_tool_frame",
        "expected_robodk_station_name": "Station",
        "expected_robodk_robot_name": "UR10",
        "expected_robodk_tool_name": "Tool",
    })


class RoboDKBridgeTests(unittest.TestCase):
    def test_joint_commands_are_passed_in_robodk_order(self):
        rdk = FakeRDK()
        result = plan_speed_profile(sample_records(3))
        allow_separate_tool_mapping(result)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_speed_plan(
                result, tool_name="Tool", program_name="Plan")
        program = rdk.Item("Plan", 4)
        self.assertEqual(program.speed_calls[0], (
            result.points[0].linear_speed,
            result.points[0].joint_speed,
            result.points[0].linear_accel,
            result.points[0].joint_accel,
        ))
        self.assertFalse(report["parameter_readback_supported"])

    def test_failed_temporary_build_preserves_existing_program(self):
        rdk = FakeRDK(fail_move=True)
        old = FakeProgram(rdk, "Plan")
        rdk.items[(4, "Plan")] = old
        result = plan_speed_profile(sample_records(3))
        allow_separate_tool_mapping(result)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "simulated move failure"):
                robodk_bridge.import_speed_plan(
                    result, tool_name="Tool", program_name="Plan", replace=True)
        self.assertTrue(old.Valid())
        self.assertIs(rdk.Item("Plan", 4), old)

    def test_unvalidated_extrinsic_is_rejected_before_connection(self):
        result = plan_speed_profile(sample_records(3))
        with patch.object(robodk_bridge, "_import_api") as import_api:
            with self.assertRaisesRegex(ValueError, "verified scanner TCP/tool mapping"):
                robodk_bridge.import_speed_plan(result, tool_name="Tool")
        import_api.assert_not_called()

    def test_failed_commit_restores_existing_program(self):
        rdk = FakeRDK(fail_commit=True)
        old = FakeProgram(rdk, "Plan")
        rdk.items[(4, "Plan")] = old
        result = plan_speed_profile(sample_records(3))
        allow_separate_tool_mapping(result)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "commit rename failure"):
                robodk_bridge.import_speed_plan(
                    result, tool_name="Tool", program_name="Plan", replace=True)
        self.assertTrue(old.Valid())
        self.assertIs(rdk.Item("Plan", 4), old)

    def test_scanner_tcp_mapping_is_verified_against_station_tool(self):
        rdk = FakeRDK()
        result = plan_speed_profile(sample_records(3))
        result.diagnostics.update({
            "extrinsic_validated": True,
            "physical_calibration_validated": False,
            "tool_mapping_mode": "robodk_tcp_is_scanner",
            "expected_robodk_station_name": "Station",
            "expected_robodk_robot_name": "UR10",
            "expected_robodk_tool_name": "Tool",
            "T_flange_scanner": [
                [0, 0, 1, 90], [0, 1, 0, 0], [-1, 0, 0, 300], [0, 0, 0, 1]],
            "tool_position_tolerance_mm": 0.01,
            "tool_orientation_tolerance_deg": 0.01,
        })
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_speed_plan(
                result, tool_name="Tool", program_name="Plan")
        verification = report["tool_mapping_verification"]
        self.assertEqual(verification["position_error_mm"], 0.0)
        self.assertEqual(verification["orientation_error_deg"], 0.0)

    def test_scanner_tcp_mismatch_is_rejected_before_program_creation(self):
        rdk = FakeRDK()
        result = plan_speed_profile(sample_records(3))
        result.diagnostics.update({
            "extrinsic_validated": True,
            "tool_mapping_mode": "robodk_tcp_is_scanner",
            "expected_robodk_tool_name": "Tool",
            "T_flange_scanner": [
                [0, 0, 1, 91], [0, 1, 0, 0], [-1, 0, 0, 300], [0, 0, 0, 1]],
            "tool_position_tolerance_mm": 0.01,
            "tool_orientation_tolerance_deg": 0.01,
        })
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "scanner TCP mismatch"):
                robodk_bridge.import_speed_plan(
                    result, tool_name="Tool", program_name="Plan")
        self.assertFalse(rdk.Item("Plan", 4).Valid())


if __name__ == "__main__":
    unittest.main()
