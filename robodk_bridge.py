"""RoboDK bridge for importing a SpeedPlanResult as Set Speed -> Move pairs."""

import math
import os
import re
import sys
import uuid
from typing import Dict, Optional


def _import_api():
    try:
        from robodk.robolink import (  # type: ignore
            ITEM_TYPE_FRAME, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT,
            ITEM_TYPE_TARGET, ITEM_TYPE_TOOL, Robolink,
        )
        from robodk.robomath import Mat, transl  # type: ignore
    except ImportError:
        candidates = [
            os.environ.get("ROBODK_API_PATH", ""),
            r"D:\RoboDK\Python37\lib\site-packages",
            r"C:\RoboDK\Python37\lib\site-packages",
            r"C:\Program Files\RoboDK\Python37\lib\site-packages",
        ]
        for path in candidates:
            if path and os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)
        try:
            from robolink import (  # type: ignore
                ITEM_TYPE_FRAME, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT,
                ITEM_TYPE_TARGET, ITEM_TYPE_TOOL, Robolink,
            )
            from robodk import Mat, transl  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "RoboDK API not found. Set ROBODK_API_PATH to RoboDK's site-packages."
            ) from exc
    return locals()


def inspect_station() -> Dict[str, object]:
    api = _import_api()
    rdk = api["Robolink"]()
    return {
        "station": rdk.ActiveStation().Name(),
        "robots": [item.Name() for item in rdk.ItemList(api["ITEM_TYPE_ROBOT"])],
        "frames": [item.Name() for item in rdk.ItemList(api["ITEM_TYPE_FRAME"])],
        "tools": [item.Name() for item in rdk.ItemList(api["ITEM_TYPE_TOOL"])],
        "programs": [item.Name() for item in rdk.ItemList(api["ITEM_TYPE_PROGRAM"])],
    }


def _require_item(rdk, name, item_type, label):
    item = rdk.Item(name, item_type)
    if not item.Valid():
        raise RuntimeError("RoboDK {} not found: {}".format(label, name))
    return item


def _pose(point, api):
    qw, qx, qy, qz = point.qw, point.qx, point.qy, point.qz
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is zero at point {}".format(point.index))
    qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
    rotation = api["Mat"]([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw), 0],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), 0],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy), 0],
        [0, 0, 0, 1],
    ])
    return api["transl"](point.x, point.y, point.z) * rotation


def _safe_name(value):
    cleaned = re.sub(r"[^0-9A-Za-z_\u4e00-\u9fff]+", "_", value.strip())
    return cleaned or "SpeedPlan"


def _delete_if_valid(item):
    if item is not None and item.Valid():
        item.Delete()


def _validate_program(program, points):
    """Validate structure and read speed parameters when this RoboDK supports it."""
    instructions = []
    for instruction_id in range(program.InstructionCount()):
        name, instruction_type, move_type, is_joint, _pose_value, _joints = program.Instruction(
            instruction_id)
        instructions.append({"id": instruction_id, "name": name, "type": instruction_type,
                             "move_type": move_type, "is_joint_target": is_joint})
    pairs = instructions[2:]
    if len(pairs) != 2 * len(points):
        raise RuntimeError("Unexpected RoboDK instruction count: {}".format(len(instructions)))

    expected = []
    readback = []
    readback_supported = True
    aliases = {
        "linear_speed": ("speed", "linearspeed"),
        "joint_speed": ("jointspeed", "speedjoints"),
        "linear_accel": ("accel", "linearaccel", "acceleration"),
        "joint_accel": ("jointaccel", "acceljoints"),
    }
    for i, point in enumerate(points):
        speed_instruction = pairs[2 * i]
        move_instruction = pairs[2 * i + 1]
        if speed_instruction["type"] != 2 or move_instruction["type"] != 0:
            raise RuntimeError("Pose {} is not Set Speed -> Move".format(i + 1))
        values = {
            "linear_speed": float(point.linear_speed),
            "joint_speed": float(point.joint_speed),
            "linear_accel": float(point.linear_accel),
            "joint_accel": float(point.joint_accel),
        }
        expected.append(values)
        try:
            raw = program.setParam(speed_instruction["id"])
        except Exception:
            raw = None
        if not isinstance(raw, dict):
            readback_supported = False
            continue
        normalised = {re.sub(r"[^a-z]", "", str(key).lower()): value
                      for key, value in raw.items()}
        actual = {}
        for logical_name, keys in aliases.items():
            matched = next((normalised[key] for key in keys if key in normalised), None)
            if matched is None:
                readback_supported = False
                break
            actual[logical_name] = float(matched)
            if not math.isclose(actual[logical_name], values[logical_name], rel_tol=1e-6,
                                abs_tol=1e-6):
                raise RuntimeError("RoboDK speed readback mismatch at pose {} for {}".format(
                    i + 1, logical_name))
        if len(actual) == 4:
            readback.append(actual)
    return instructions, expected, readback if readback_supported else None


def import_speed_plan(result, robot_name="UR10", frame_name="Frame 2",
                      tool_name="Creaform MetraSCAN", program_name="IntegratedSpeedPlan",
                      target_namespace: Optional[str] = None, first_move="movej",
                      replace=False):
    """Create a RoboDK program and verify one speed instruction per pose."""
    if not result.points:
        raise ValueError("Speed plan is empty")
    if not result.feasible:
        raise ValueError("Speed plan contains constraint violations")
    if not result.diagnostics.get("extrinsic_validated", False):
        raise ValueError(
            "RoboDK import requires a validated scanner-to-tool extrinsic calibration")
    if first_move not in ("movej", "movel"):
        raise ValueError("first_move must be movej or movel")
    for point in result.points:
        values = (point.linear_speed, point.linear_accel, point.joint_speed, point.joint_accel)
        if not all(math.isfinite(float(value)) and float(value) > 0 for value in values):
            raise ValueError("Pose {} has invalid RoboDK speed command values".format(point.index))
    api = _import_api()
    rdk = api["Robolink"]()
    robot = _require_item(rdk, robot_name, api["ITEM_TYPE_ROBOT"], "robot")
    frame = _require_item(rdk, frame_name, api["ITEM_TYPE_FRAME"], "frame")
    tool = _require_item(rdk, tool_name, api["ITEM_TYPE_TOOL"], "tool")

    namespace = target_namespace or (_safe_name(program_name) + "_")
    target_names = [namespace + "P{}".format(i + 1) for i in range(len(result.points))]
    existing_program = rdk.Item(program_name, api["ITEM_TYPE_PROGRAM"])
    existing_targets = [rdk.Item(name, api["ITEM_TYPE_TARGET"]) for name in target_names]
    existing_targets = [target for target in existing_targets if target.Valid()]
    if existing_program.Valid() and not replace:
        raise RuntimeError("Program already exists: {}".format(program_name))
    if existing_targets and not replace:
        raise RuntimeError("Target already exists: {}".format(existing_targets[0].Name()))

    token = uuid.uuid4().hex[:10]
    temp_program_name = "{}__tmp_{}".format(_safe_name(program_name), token)
    temp_target_names = ["{}__tmp_{}_P{}".format(_safe_name(namespace), token, i + 1)
                         for i in range(len(result.points))]
    program = rdk.AddProgram(temp_program_name, robot)
    program.setFrame(frame)
    program.setTool(tool)
    created = []
    rdk.Render(False)
    try:
        for i, (point, name) in enumerate(zip(result.points, temp_target_names)):
            target = rdk.AddTarget(name, frame, robot)
            target.setPose(_pose(point, api))
            target.setAsCartesianTarget()
            created.append(target)
            program.setSpeed(point.linear_speed, point.joint_speed,
                             point.linear_accel, point.joint_accel)
            if i == 0 and first_move == "movej":
                program.MoveJ(target)
            else:
                program.MoveL(target)
        instructions, expected_commands, parameter_readback = _validate_program(
            program, result.points)

        # Commit only after the temporary program is complete and validated.
        if existing_program.Valid():
            existing_program.Delete()
        for target in existing_targets:
            _delete_if_valid(target)
        program.setName(program_name)
        for target, final_name in zip(created, target_names):
            target.setName(final_name)
    except Exception:
        _delete_if_valid(program)
        for target in created:
            _delete_if_valid(target)
        raise
    finally:
        rdk.Render(True)

    program.ShowInstructions(True)
    return {
        "station": rdk.ActiveStation().Name(),
        "program": program.Name(),
        "target_namespace": namespace,
        "pose_count": len(result.points),
        "instruction_count": len(instructions),
        "instructions": instructions,
        "expected_speed_commands": expected_commands,
        "parameter_readback_supported": parameter_readback is not None,
        "parameter_readback": parameter_readback,
    }
