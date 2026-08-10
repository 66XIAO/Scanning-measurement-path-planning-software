import unittest
from unittest.mock import patch

from pose_transform import parse_extrinsic_config, transform_pose_records
import robodk_bridge
from speed_planning_core import plan_speed_profile
from test_speed_planning_core import sample_records


class FakeMatrix:
    def __init__(self, values=None):
        self.values = values or [[1 if row == column else 0 for column in range(4)]
                                 for row in range(4)]

    def __mul__(self, other):
        return FakeMatrix([
            [sum(self.values[row][inner] * other.values[inner][column]
                 for inner in range(4))
             for column in range(4)]
            for row in range(4)
        ])

    def __getitem__(self, key):
        row, column = key
        return self.values[row][column]


TOOL_POSE = [
    [0, 0, 1, 90],
    [0, 1, 0, 0],
    [-1, 0, 0, 300],
    [0, 0, 0, 1],
]


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
        if self.rdk.fail_delete and self.rdk.fail_delete(self):
            raise RuntimeError("simulated delete failure")
        self.valid = False
        key = (self.item_type, self.name)
        if self.rdk.items.get(key) is self:
            self.rdk.items.pop(key, None)
        if self in self.rdk.extra_items:
            self.rdk.extra_items.remove(self)

    def setName(self, name):
        if self.rdk.fail_rename and self.rdk.fail_rename(self, name):
            raise RuntimeError("simulated item rename failure")
        key = (self.item_type, self.name)
        if self.rdk.items.get(key) is self:
            self.rdk.items.pop(key, None)
        self.name = name
        if self not in self.rdk.extra_items:
            self.rdk.items[(self.item_type, self.name)] = self

    def setPose(self, pose):
        self.pose = pose

    def setAsCartesianTarget(self):
        pass

    def PoseTool(self):
        return FakeMatrix(self.tool_pose)

    def Pose(self):
        return FakeMatrix(self.tool_pose)


class FakeProgram(FakeItem):
    def __init__(self, rdk, name):
        super().__init__(rdk, name, 4)
        self.instructions = []
        self.speed_calls = []

    def setName(self, name):
        if (self.rdk.fail_commit and "__tmp_" in self.name
                and name in ("Plan", "Path")):
            raise RuntimeError("simulated commit rename failure")
        super().setName(name)

    def setFrame(self, _frame):
        if self.rdk.fail_set_frame:
            raise RuntimeError("simulated setFrame failure")
        self.instructions.append(("Frame", 3, 0, 0, None, None))

    def setTool(self, _tool):
        if self.rdk.fail_set_tool:
            raise RuntimeError("simulated setTool failure")
        self.instructions.append(("Tool", 4, 0, 0, None, None))

    def setSpeed(self, *values):
        self.speed_calls.append(values)
        self.instructions.append(("Set Speed", 2, 0, 0, None, None))

    def _move(self, name):
        if self.rdk.fail_move:
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
        if self.rdk.fail_show_instructions:
            raise RuntimeError("simulated ShowInstructions failure")


class FakeRDK:
    def __init__(self, fail_move=False, fail_commit=False,
                 fail_set_frame=False, fail_set_tool=False,
                 closest_match=False, item_list_enabled=True,
                 render_fail_calls=()):
        self.items = {}
        self.extra_items = []
        self.fail_move = fail_move
        self.fail_commit = fail_commit
        self.fail_set_frame = fail_set_frame
        self.fail_set_tool = fail_set_tool
        self.closest_match = closest_match
        self.fail_delete = None
        self.fail_rename = None
        self.fail_show_instructions = False
        self.render_fail_calls = set(render_fail_calls)
        self.render_calls = []
        if not item_list_enabled:
            self.ItemList = None
        for name, item_type in (("UR10", 1), ("Frame 2", 2)):
            self.items[(item_type, name)] = FakeItem(self, name, item_type)
        self.items[(3, "Tool")] = FakeItem(
            self, "Tool", 3, tool_pose=TOOL_POSE)

    def Item(self, name, item_type):
        exact = [item for item in self._all_items(item_type)
                 if item.Name() == name]
        if exact:
            return exact[0]
        if self.closest_match:
            candidates = self._all_items(item_type)
            if candidates:
                return candidates[-1]
        return FakeItem(self, name, item_type, False)

    def _all_items(self, item_type=None):
        values = list(self.items.values()) + list(self.extra_items)
        return [item for item in values if item.Valid()
                and (item_type is None or item.item_type == item_type)]

    def ItemList(self, item_type):
        return self._all_items(item_type)

    def AddProgram(self, name, _robot):
        item = FakeProgram(self, name)
        self.items[(4, name)] = item
        return item

    def AddTarget(self, name, _frame, _robot):
        item = FakeItem(self, name, 5)
        self.items[(5, name)] = item
        return item

    def Render(self, enabled):
        self.render_calls.append(enabled)
        if len(self.render_calls) in self.render_fail_calls:
            raise RuntimeError("simulated Render failure")

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
        "transl": lambda x, y, z: FakeMatrix([
            [1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1],
        ]),
    }


def separate_tool_mapping_metadata():
    return {
        "extrinsic_validated": True,
        "physical_calibration_validated": False,
        "tool_mapping_mode": "separate_tool_frame",
        "expected_robodk_station_name": "Station",
        "expected_robodk_robot_name": "UR10",
        "expected_robodk_tool_name": "Tool",
        "T_flange_tool": TOOL_POSE,
    }


def allow_separate_tool_mapping(result):
    result.diagnostics.update(separate_tool_mapping_metadata())


class RoboDKBridgeTests(unittest.TestCase):
    def test_blank_required_and_optional_names_are_rejected_before_connection(self):
        for kwargs, message in (
                ({"program_name": "   "}, "program name must not be blank"),
                ({"target_namespace": "\t"},
                 "target namespace name must not be blank")):
            with self.subTest(kwargs=kwargs):
                with patch.object(robodk_bridge, "_import_api") as import_api:
                    with self.assertRaisesRegex(ValueError, message):
                        robodk_bridge.import_planned_path(
                            sample_records(2), separate_tool_mapping_metadata(),
                            tool_name="Tool", **kwargs)
                import_api.assert_not_called()

    def test_required_lookup_rejects_robodk_closest_match(self):
        rdk = FakeRDK(closest_match=True, item_list_enabled=False)
        rdk.items.pop((2, "Frame 2"))
        rdk.items[(2, "Frame 3")] = FakeItem(rdk, "Frame 3", 2)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "exact name: Frame 2"):
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool")
        self.assertEqual(rdk.render_calls, [])

    def test_exact_inventory_rejects_duplicate_required_item(self):
        rdk = FakeRDK()
        rdk.extra_items.append(FakeItem(rdk, "UR10", 1))
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "Duplicate RoboDK robot"):
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool")
        self.assertEqual(rdk.render_calls, [])

    def test_exact_inventory_rejects_duplicate_generated_target(self):
        rdk = FakeRDK()
        first = FakeItem(rdk, "Path_P1", 5)
        duplicate = FakeItem(rdk, "Path_P1", 5)
        rdk.items[(5, "Path_P1")] = first
        rdk.extra_items.append(duplicate)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "Duplicate RoboDK targets"):
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Path", replace=True)
        self.assertTrue(first.Valid())
        self.assertTrue(duplicate.Valid())

    def test_unrelated_closest_names_are_never_replaced(self):
        rdk = FakeRDK(closest_match=True)
        other_program = FakeProgram(rdk, "Path Backup")
        other_target = FakeItem(rdk, "Path_extra_P1", 5)
        rdk.items[(4, other_program.Name())] = other_program
        rdk.items[(5, other_target.Name())] = other_target
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            robodk_bridge.import_planned_path(
                sample_records(2), separate_tool_mapping_metadata(),
                tool_name="Tool", program_name="Path", replace=True)
        self.assertTrue(other_program.Valid())
        self.assertEqual(other_program.Name(), "Path Backup")
        self.assertTrue(other_target.Valid())
        self.assertEqual(other_target.Name(), "Path_extra_P1")

    def test_replacing_shorter_path_removes_all_stale_namespace_targets_only(self):
        rdk = FakeRDK()
        old_program = FakeProgram(rdk, "Path")
        rdk.items[(4, "Path")] = old_program
        old_targets = []
        for index in range(1, 6):
            target = FakeItem(rdk, "Path_P{}".format(index), 5)
            rdk.items[(5, target.Name())] = target
            old_targets.append(target)
        similar = FakeItem(rdk, "Path_extra_P1", 5)
        rdk.items[(5, similar.Name())] = similar

        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_planned_path(
                sample_records(2), separate_tool_mapping_metadata(),
                tool_name="Tool", program_name="Path", replace=True)

        self.assertEqual(report["pose_count"], 2)
        self.assertFalse(old_program.Valid())
        self.assertTrue(all(not target.Valid() for target in old_targets))
        self.assertTrue(rdk.Item("Path_P1", 5).Valid())
        self.assertTrue(rdk.Item("Path_P2", 5).Valid())
        self.assertFalse(rdk.Item("Path_P3", 5).Valid())
        self.assertTrue(similar.Valid())

    def test_setframe_failure_removes_temp_program_and_restores_render(self):
        rdk = FakeRDK(fail_set_frame=True)
        old = FakeProgram(rdk, "Path")
        rdk.items[(4, "Path")] = old
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "setFrame failure"):
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Path", replace=True)
        self.assertIs(rdk.Item("Path", 4), old)
        self.assertEqual(rdk.render_calls, [False, True])
        self.assertEqual([item.Name() for item in rdk.ItemList(4)], ["Path"])

    def test_render_restore_failure_rolls_back_published_program_then_retries(self):
        rdk = FakeRDK(render_fail_calls=(2,))
        old_program = FakeProgram(rdk, "Path")
        old_target = FakeItem(rdk, "Path_P1", 5)
        rdk.items[(4, "Path")] = old_program
        rdk.items[(5, "Path_P1")] = old_target
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "Render failure"):
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Path", replace=True)
        self.assertIs(rdk.Item("Path", 4), old_program)
        self.assertIs(rdk.Item("Path_P1", 5), old_target)
        self.assertFalse(rdk.Item("Path_P2", 5).Valid())
        self.assertEqual(rdk.render_calls, [False, True, True])

    def test_rollback_restores_backups_even_when_new_object_delete_fails(self):
        rdk = FakeRDK()
        old_program = FakeProgram(rdk, "Path")
        old_targets = [FakeItem(rdk, "Path_P{}".format(index), 5)
                       for index in (1, 2)]
        rdk.items[(4, "Path")] = old_program
        for target in old_targets:
            rdk.items[(5, target.Name())] = target

        rdk.fail_rename = lambda item, name: (
            item.item_type == 5 and "__tmp_" in item.Name()
            and name == "Path_P2")
        rdk.fail_delete = lambda item: (
            item.item_type == 4 and "__rollback_" in item.Name())
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "item rename failure") as caught:
                robodk_bridge.import_planned_path(
                    sample_records(2), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Path", replace=True)
        self.assertIs(rdk.Item("Path", 4), old_program)
        self.assertIs(rdk.Item("Path_P1", 5), old_targets[0])
        self.assertIs(rdk.Item("Path_P2", 5), old_targets[1])
        self.assertTrue(any("delete new program" in error for error in
                            caught.exception.robodk_cleanup_errors))

    def test_post_commit_cleanup_and_show_failures_are_reported_as_warnings(self):
        rdk = FakeRDK()
        old_program = FakeProgram(rdk, "Path")
        old_target = FakeItem(rdk, "Path_P1", 5)
        rdk.items[(4, "Path")] = old_program
        rdk.items[(5, "Path_P1")] = old_target
        rdk.fail_delete = lambda item: "__backup_" in item.Name()
        rdk.fail_show_instructions = True

        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_planned_path(
                sample_records(2), separate_tool_mapping_metadata(),
                tool_name="Tool", program_name="Path", replace=True)

        self.assertTrue(rdk.Item("Path", 4).Valid())
        self.assertTrue(report["replace_committed"])
        self.assertTrue(any("program backup" in warning for warning in
                            report["backup_cleanup_warnings"]))
        self.assertTrue(any("show program instructions" in warning.lower()
                            for warning in report["backup_cleanup_warnings"]))

    def test_separate_tool_mapping_requires_and_verifies_flange_tool_pose(self):
        rdk = FakeRDK()
        missing = separate_tool_mapping_metadata()
        missing.pop("T_flange_tool")
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(ValueError, "T_flange_tool is required"):
                robodk_bridge.import_planned_path(
                    sample_records(2), missing, tool_name="Tool")

        mismatch = separate_tool_mapping_metadata()
        mismatch["T_flange_tool"] = [row[:] for row in TOOL_POSE]
        mismatch["T_flange_tool"][0][3] += 1.0
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "command-tool TCP mismatch"):
                robodk_bridge.import_planned_path(
                    sample_records(2), mismatch, tool_name="Tool")
        self.assertFalse(rdk.Item("IntegratedPlannedPath", 4).Valid())

    def test_planned_path_creates_moves_without_speed_instructions(self):
        rdk = FakeRDK()
        records = sample_records(3)
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_planned_path(
                records, separate_tool_mapping_metadata(),
                tool_name="Tool", program_name="Path")

        program = rdk.Item("Path", 4)
        self.assertEqual(program.speed_calls, [])
        self.assertEqual(
            [instruction[0] for instruction in program.instructions],
            ["Frame", "Tool", "MoveJ", "MoveL", "MoveL"])
        self.assertEqual(report["pose_count"], 3)
        self.assertEqual(report["instruction_count"], 5)
        self.assertEqual(report["source_pose_indices"], [1, 2, 3])
        self.assertFalse(report["speed_commands_applied"])

    def test_planned_path_rejects_unvalidated_mapping_before_connection(self):
        with patch.object(robodk_bridge, "_import_api") as import_api:
            with self.assertRaisesRegex(ValueError, "verified scanner TCP/tool mapping"):
                robodk_bridge.import_planned_path(sample_records(3), {})
        import_api.assert_not_called()

    def test_planned_path_rejects_invalid_pose_before_connection(self):
        records = sample_records(3)
        records[1].update({"qw": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0})
        with patch.object(robodk_bridge, "_import_api") as import_api:
            with self.assertRaisesRegex(ValueError, "Quaternion norm is zero"):
                robodk_bridge.import_planned_path(
                    records, separate_tool_mapping_metadata())
        import_api.assert_not_called()

    def test_planned_path_accepts_transformed_command_pose_contract(self):
        mapping = [
            [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 100], [0, 0, 0, 1],
        ]
        config = parse_extrinsic_config({
            "schema_version": "1.1",
            "config_id": "planned_path_test",
            "calibration_status": "station_verified",
            "mapping_mode": "separate_tool_frame",
            "source_pose_frame": "scanner",
            "command_pose_frame": "tool",
            "length_unit": "mm",
            "quaternion_order": "wxyz",
            "T_tool_scanner": mapping,
            "T_flange_tool": TOOL_POSE,
            "robodk_station_name": "Station",
            "robodk_robot_name": "UR10",
            "robodk_tool_name": "Tool",
            "robodk_frame_name": "Frame 2",
        })
        source_records = sample_records(2)
        source_records[0]["z"] = 500.0
        source_records[1]["z"] = 600.0
        command_records, metadata = transform_pose_records(source_records, config)
        rdk = FakeRDK()

        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            report = robodk_bridge.import_planned_path(
                command_records, metadata, tool_name="Tool", program_name="Path")

        first_target = rdk.Item("Path_P1", 5)
        second_target = rdk.Item("Path_P2", 5)
        self.assertAlmostEqual(first_target.pose[2, 3], 400.0)
        self.assertAlmostEqual(second_target.pose[2, 3], 500.0)
        self.assertEqual(
            report["tool_mapping_verification"]["mapping_mode"],
            "separate_tool_frame")

    def test_failed_planned_path_build_preserves_existing_program(self):
        rdk = FakeRDK(fail_move=True)
        old = FakeProgram(rdk, "Path")
        rdk.items[(4, "Path")] = old
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "simulated move failure"):
                robodk_bridge.import_planned_path(
                    sample_records(3), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Path", replace=True)
        self.assertTrue(old.Valid())
        self.assertIs(rdk.Item("Path", 4), old)

    def test_failed_planned_path_commit_restores_existing_program(self):
        rdk = FakeRDK(fail_commit=True)
        old = FakeProgram(rdk, "Plan")
        rdk.items[(4, "Plan")] = old
        with patch.object(robodk_bridge, "_import_api", return_value=fake_api(rdk)):
            with self.assertRaisesRegex(RuntimeError, "commit rename failure"):
                robodk_bridge.import_planned_path(
                    sample_records(3), separate_tool_mapping_metadata(),
                    tool_name="Tool", program_name="Plan", replace=True)
        self.assertTrue(old.Valid())
        self.assertIs(rdk.Item("Plan", 4), old)

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
