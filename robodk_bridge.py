"""RoboDK bridge for importing ordered pose paths and pose-plus-speed plans.

Both import paths use the same station, frame and tool-mapping verification and
the same temporary-program commit/rollback strategy.  ``import_planned_path``
deliberately emits only movement instructions; speed and acceleration commands
remain the responsibility of the later speed-planning stage.
"""

from dataclasses import dataclass
import math
import os
import re
import sys
import uuid
from typing import Callable, Dict, Iterable, Mapping, Optional


@dataclass(frozen=True)
class PlannedPathPoint:
    """Validated command pose used by the pose-only RoboDK importer.

    Positions are millimetres and quaternions use ``wxyz`` ordering.  These
    are command poses in the selected RoboDK reference frame, not raw scanner
    poses.  Callers must apply the configured scanner-to-tool mapping first.
    """

    index: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float


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


def _required_name(value, label):
    """Return a non-blank station-tree name suitable for exact lookup."""
    if not isinstance(value, str):
        raise ValueError("RoboDK {} name must be a string".format(label))
    name = value.strip()
    if not name:
        raise ValueError("RoboDK {} name must not be blank".format(label))
    return name


def _optional_name(value, label):
    if value is None:
        return None
    return _required_name(value, label)


def _is_valid(item):
    return item is not None and bool(item.Valid())


def _item_inventory(rdk, item_type):
    """Return valid items when this RoboDK API exposes ItemList.

    RoboDK ``Item(name, type)`` deliberately returns a closest match when an
    exact name does not exist.  ItemList lets us both avoid that behaviour and
    detect duplicate exact names.  A narrow exact-name fallback remains for
    older API shims which do not expose ItemList.
    """
    item_list = getattr(rdk, "ItemList", None)
    if not callable(item_list):
        return None
    return [item for item in item_list(item_type) if _is_valid(item)]


def _find_exact_items(rdk, name, item_type, label):
    name = _required_name(name, label)
    inventory = _item_inventory(rdk, item_type)
    if inventory is not None:
        matches = [item for item in inventory if item.Name() == name]
    else:
        candidate = rdk.Item(name, item_type)
        matches = ([candidate] if _is_valid(candidate)
                   and candidate.Name() == name else [])
    if len(matches) > 1:
        raise RuntimeError(
            "Duplicate RoboDK {} items have the exact name: {}".format(label, name))
    return matches


def _find_optional_item(rdk, name, item_type, label):
    matches = _find_exact_items(rdk, name, item_type, label)
    return matches[0] if matches else None


def _require_item(rdk, name, item_type, label):
    item = _find_optional_item(rdk, name, item_type, label)
    if item is None:
        raise RuntimeError("RoboDK {} not found with exact name: {}".format(
            label, name))
    return item


def _generated_target_inventory(rdk, item_type, namespace):
    """Return exactly ``<namespace>P<positive integer>`` targets.

    This inventory includes stale targets left by a previously longer path but
    excludes merely similar prefixes such as ``Path_extra_P1``.
    """
    namespace = _required_name(namespace, "target namespace")
    inventory = _item_inventory(rdk, item_type)
    if inventory is None:
        raise RuntimeError(
            "Safe RoboDK target replacement requires ItemList support")
    pattern = re.compile(r"^{}P([1-9][0-9]*)$".format(re.escape(namespace)))
    matches = []
    names = {}
    for item in inventory:
        name = item.Name()
        match = pattern.fullmatch(name)
        if match is None:
            continue
        names.setdefault(name, []).append(item)
        matches.append((int(match.group(1)), name, item))
    duplicates = sorted(name for name, items in names.items() if len(items) > 1)
    if duplicates:
        raise RuntimeError(
            "Duplicate RoboDK targets have the exact name: {}".format(
                duplicates[0]))
    matches.sort(key=lambda entry: (entry[0], entry[1]))
    return [entry[2] for entry in matches]


def _prepare_planned_path_points(records: Iterable[object]):
    """Normalise an ordered path before opening a RoboDK connection."""
    points = []
    previous_quaternion = None
    for fallback_index, record in enumerate(records, start=1):
        try:
            if isinstance(record, Mapping):
                index = int(record.get("index", fallback_index))
                position = (float(record["x"]), float(record["y"]), float(record["z"]))
                quaternion = (float(record["qw"]), float(record["qx"]),
                              float(record["qy"]), float(record["qz"]))
            elif all(hasattr(record, name) for name in
                     ("x", "y", "z", "qw", "qx", "qy", "qz")):
                index = int(getattr(record, "index", fallback_index))
                position = (float(record.x), float(record.y), float(record.z))
                quaternion = (float(record.qw), float(record.qx),
                              float(record.qy), float(record.qz))
            else:
                values = list(record)
                if len(values) != 8:
                    raise ValueError(
                        "pose tuples must be index,x,y,z,qw,qx,qy,qz")
                index = int(values[0])
                position = tuple(float(value) for value in values[1:4])
                quaternion = tuple(float(value) for value in values[4:8])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Planned path pose {} is invalid: {}".format(
                fallback_index, exc)) from exc

        if not all(math.isfinite(value) for value in position + quaternion):
            raise ValueError("Planned path pose {} contains non-finite values".format(index))
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-12:
            raise ValueError("Quaternion norm is zero at point {}".format(index))
        quaternion = tuple(value / norm for value in quaternion)
        if (previous_quaternion is not None and
                sum(first * second for first, second in
                    zip(previous_quaternion, quaternion)) < 0):
            quaternion = tuple(-value for value in quaternion)
        previous_quaternion = quaternion
        points.append(PlannedPathPoint(index, *position, *quaternion))

    if not points:
        raise ValueError("Planned path is empty")
    return points


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


def _matrix_rows(matrix):
    return [[float(matrix[row, column]) for column in range(4)] for row in range(4)]


def _tool_pose_errors(actual, expected):
    translation_error = math.sqrt(sum(
        (actual[row][3] - expected[row][3]) ** 2 for row in range(3)))
    # R_delta = R_expected^T * R_actual; angle from its trace.
    trace = 0.0
    for row in range(3):
        trace += sum(expected[k][row] * actual[k][row] for k in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    orientation_error = math.degrees(math.acos(cosine))
    return translation_error, orientation_error


def _verify_tool_mapping(rdk, robot, frame, tool, diagnostics,
                         robot_name, frame_name, tool_name):
    mode = diagnostics.get("tool_mapping_mode")
    if mode not in ("robodk_tcp_is_scanner", "separate_tool_frame"):
        raise ValueError("RoboDK import metadata is missing a recognised tool_mapping_mode")
    expected_station = diagnostics.get("expected_robodk_station_name", "")
    expected_robot = diagnostics.get("expected_robodk_robot_name", "")
    expected_tool = diagnostics.get("expected_robodk_tool_name", "")
    expected_frame = diagnostics.get("expected_robodk_frame_name", "")
    station_name = rdk.ActiveStation().Name()
    actual_robot_name = robot.Name()
    actual_tool_name = tool.Name()
    actual_frame_name = frame.Name()
    if expected_station and expected_station != station_name:
        raise RuntimeError("RoboDK station mismatch: expected {}, got {}".format(
            expected_station, station_name))
    if expected_robot and expected_robot != actual_robot_name:
        raise RuntimeError("RoboDK robot mismatch: expected {}, got {}".format(
            expected_robot, actual_robot_name))
    if expected_tool and expected_tool != actual_tool_name:
        raise RuntimeError("RoboDK tool mismatch: expected {}, got {}".format(
            expected_tool, actual_tool_name))
    if expected_frame and expected_frame != actual_frame_name:
        raise RuntimeError("RoboDK frame mismatch: expected {}, got {}".format(
            expected_frame, actual_frame_name))

    # Exact station-tree lookup is mandatory even when a calibration file does
    # not pin expected names.  This also documents the names that were actually
    # verified rather than merely echoing dialog input.
    if actual_robot_name != robot_name:
        raise RuntimeError("RoboDK robot exact-name lookup mismatch: requested {}, got {}"
                           .format(robot_name, actual_robot_name))
    if actual_tool_name != tool_name:
        raise RuntimeError("RoboDK tool exact-name lookup mismatch: requested {}, got {}"
                           .format(tool_name, actual_tool_name))
    if actual_frame_name != frame_name:
        raise RuntimeError("RoboDK frame exact-name lookup mismatch: requested {}, got {}"
                           .format(frame_name, actual_frame_name))

    verification = {
        "mapping_mode": mode,
        "station": station_name,
        "robot": actual_robot_name,
        "tool": actual_tool_name,
        "frame": actual_frame_name,
        "physical_calibration_validated": bool(
            diagnostics.get("physical_calibration_validated", False)),
    }
    expected_matrix = None
    matrix_label = None
    mismatch_label = None
    if mode == "robodk_tcp_is_scanner":
        expected_matrix = diagnostics.get("T_flange_scanner")
        matrix_label = "T_flange_scanner"
        mismatch_label = "scanner TCP"
    else:
        expected_matrix = diagnostics.get("T_flange_tool")
        matrix_label = "T_flange_tool"
        mismatch_label = "command-tool TCP"
    if expected_matrix is None:
        raise ValueError("{} is required for RoboDK {} mode".format(
            matrix_label, mode))

    if expected_matrix is not None:
        actual_matrix = _matrix_rows(tool.PoseTool())
        translation_error, orientation_error = _tool_pose_errors(
            actual_matrix, expected_matrix)
        position_tolerance = float(diagnostics.get("tool_position_tolerance_mm", 0.01))
        orientation_tolerance = float(
            diagnostics.get("tool_orientation_tolerance_deg", 0.01))
        if translation_error > position_tolerance or orientation_error > orientation_tolerance:
            raise RuntimeError(
                "RoboDK {} mismatch: position {:.6f} mm (limit {:.6f}), "
                "orientation {:.6f} deg (limit {:.6f})".format(
                    mismatch_label, translation_error, position_tolerance,
                    orientation_error, orientation_tolerance))
        verification.update({
            "{}_actual".format(matrix_label): actual_matrix,
            "position_error_mm": translation_error,
            "orientation_error_deg": orientation_error,
        })
    expected_frame_matrix = diagnostics.get("T_station_reference_frame")
    if expected_frame_matrix is not None:
        actual_frame_matrix = _matrix_rows(frame.Pose())
        frame_position_error, frame_orientation_error = _tool_pose_errors(
            actual_frame_matrix, expected_frame_matrix)
        position_tolerance = float(diagnostics.get("tool_position_tolerance_mm", 0.01))
        orientation_tolerance = float(
            diagnostics.get("tool_orientation_tolerance_deg", 0.01))
        if (frame_position_error > position_tolerance or
                frame_orientation_error > orientation_tolerance):
            raise RuntimeError(
                "RoboDK reference frame mismatch: position {:.6f} mm, "
                "orientation {:.6f} deg".format(
                    frame_position_error, frame_orientation_error))
        verification.update({
            "T_station_reference_frame_actual": actual_frame_matrix,
            "frame_position_error_mm": frame_position_error,
            "frame_orientation_error_deg": frame_orientation_error,
        })
    return verification


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


def _validate_planned_path_program(program, points):
    """Verify that a pose-only import contains one move and no speed per pose."""
    instructions = []
    for instruction_id in range(program.InstructionCount()):
        name, instruction_type, move_type, is_joint, _pose_value, _joints = program.Instruction(
            instruction_id)
        instructions.append({"id": instruction_id, "name": name, "type": instruction_type,
                             "move_type": move_type, "is_joint_target": is_joint})
    moves = instructions[2:]
    if len(moves) != len(points):
        raise RuntimeError("Unexpected RoboDK instruction count: {}".format(len(instructions)))
    for pose_number, instruction in enumerate(moves, start=1):
        if instruction["type"] != 0:
            raise RuntimeError(
                "Pose {} is not a movement-only instruction".format(pose_number))
    return instructions


def _assert_created_item(item, expected_name, label):
    if not _is_valid(item):
        raise RuntimeError("RoboDK failed to create {}: {}".format(
            label, expected_name))
    actual_name = item.Name()
    if actual_name != expected_name:
        raise RuntimeError(
            "RoboDK created {} with unexpected name: expected {}, got {}".format(
                label, expected_name, actual_name))


def _rename_exact(item, new_name, label):
    item.setName(new_name)
    actual_name = item.Name()
    if actual_name != new_name:
        raise RuntimeError(
            "RoboDK {} rename failed: expected {}, got {}".format(
                label, new_name, actual_name))


def _attach_cleanup_errors(original_error, cleanup_errors):
    """Preserve the original exception and make rollback problems visible."""
    if not cleanup_errors:
        return
    details = "RoboDK rollback cleanup issues:\n- " + "\n- ".join(cleanup_errors)
    try:
        original_error.robodk_cleanup_errors = tuple(cleanup_errors)
    except Exception:
        pass
    add_note = getattr(original_error, "add_note", None)
    if callable(add_note):
        add_note(details)
    else:  # Python 3.9/3.10: retain type/object and append details to str(exc).
        try:
            original_error.args = ("{}\n{}".format(str(original_error), details),)
        except Exception:
            pass


def _rollback_program_publish(program, created_targets, backup_items,
                              final_items, token):
    """Best-effort rollback that restores old names before deleting new items."""
    errors = []

    # First vacate final names occupied by newly published objects.  A rename is
    # preferable because it leaves the new object recoverable; Delete is only a
    # fallback when the final name cannot otherwise be released.
    for index, (item, final_name, label) in enumerate(final_items, start=1):
        try:
            if not _is_valid(item) or item.Name() != final_name:
                continue
            rollback_name = "{}__rollback_{}_{}".format(
                _safe_name(final_name), token, index)
            try:
                _rename_exact(item, rollback_name, label)
            except Exception as rename_error:
                errors.append("vacate {} {}: {}".format(
                    label, final_name, rename_error))
                try:
                    _delete_if_valid(item)
                except Exception as delete_error:
                    errors.append("delete {} {} while vacating: {}".format(
                        label, final_name, delete_error))
        except Exception as exc:
            errors.append("inspect {} {} while vacating: {}".format(
                label, final_name, exc))

    # Restore every old item independently.  backup_items are registered before
    # their rename is attempted, so a partially successful rename is recoverable.
    for item, original_name, _backup_name, label in reversed(backup_items):
        try:
            if _is_valid(item) and item.Name() != original_name:
                _rename_exact(item, original_name, "{} backup".format(label))
        except Exception as exc:
            errors.append("restore {} {}: {}".format(label, original_name, exc))

    # Only after old names are restored do we remove temporary/new objects.
    new_items = []
    if program is not None:
        new_items.append((program, "new program"))
    new_items.extend((target, "new target") for target in created_targets)
    for item, label in new_items:
        try:
            _delete_if_valid(item)
        except Exception as exc:
            errors.append("delete {}: {}".format(label, exc))
    return errors


def _transactional_program_import(
        rdk, api, robot, frame, tool, points, program_name, namespace,
        first_move, replace, configure_point: Callable,
        validate_program: Callable):
    """Build, validate, and atomically publish one generated RoboDK program."""
    target_names = [namespace + "P{}".format(i + 1)
                    for i in range(len(points))]
    existing_program = _find_optional_item(
        rdk, program_name, api["ITEM_TYPE_PROGRAM"], "program")
    existing_targets = _generated_target_inventory(
        rdk, api["ITEM_TYPE_TARGET"], namespace)
    if existing_program is not None and not replace:
        raise RuntimeError("Program already exists: {}".format(program_name))
    if existing_targets and not replace:
        raise RuntimeError("Generated target namespace already exists: {}".format(
            existing_targets[0].Name()))

    token = uuid.uuid4().hex[:10]
    temp_program_name = "{}__tmp_{}".format(_safe_name(program_name), token)
    temp_target_names = ["{}__tmp_{}_P{}".format(
        _safe_name(namespace), token, i + 1) for i in range(len(points))]
    program = None
    created_targets = []
    backup_items = []
    final_items = []
    validation_result = None

    try:
        rdk.Render(False)
        program = rdk.AddProgram(temp_program_name, robot)
        _assert_created_item(program, temp_program_name, "temporary program")
        program.setFrame(frame)
        program.setTool(tool)

        for index, (point, temp_name) in enumerate(
                zip(points, temp_target_names)):
            target = rdk.AddTarget(temp_name, frame, robot)
            created_targets.append(target)
            _assert_created_item(target, temp_name, "temporary target")
            target.setPose(_pose(point, api))
            target.setAsCartesianTarget()
            configure_point(program, point, index, target, first_move)

        validation_result = validate_program(program, points)

        old_items = []
        if existing_program is not None:
            old_items.append((existing_program, program_name, "program"))
        old_items.extend((target, target.Name(), "target")
                         for target in existing_targets)
        for backup_index, (item, original_name, label) in enumerate(
                old_items, start=1):
            backup_name = "{}__backup_{}_{}".format(
                _safe_name(original_name), token, backup_index)
            backup_record = (item, original_name, backup_name, label)
            backup_items.append(backup_record)
            _rename_exact(item, backup_name, "existing {}".format(label))

        final_items = [(program, program_name, "new program")]
        final_items.extend((target, final_name, "new target")
                           for target, final_name in zip(
                               created_targets, target_names))
        _rename_exact(program, program_name, "new program")
        for target, final_name in zip(created_targets, target_names):
            _rename_exact(target, final_name, "new target")

        # Render restoration is part of the transaction.  If it fails, keep the
        # backups and roll the publication back before retrying restoration.
        rdk.Render(True)
    except Exception as original_error:
        cleanup_errors = []
        try:
            cleanup_errors.extend(_rollback_program_publish(
                program, created_targets, backup_items, final_items, token))
        except Exception as rollback_error:
            cleanup_errors.append("rollback implementation failure: {}".format(
                rollback_error))
        try:
            rdk.Render(True)
        except Exception as render_error:
            cleanup_errors.append("restore rendering: {}".format(render_error))
        _attach_cleanup_errors(original_error, cleanup_errors)
        raise

    warnings = []
    for item, original_name, _backup_name, label in backup_items:
        try:
            _delete_if_valid(item)
        except Exception as exc:
            warnings.append("Could not delete {} backup for {}: {}".format(
                label, original_name, exc))
    try:
        program.ShowInstructions(True)
    except Exception as exc:
        warnings.append("Could not show program instructions: {}".format(exc))

    return {
        "program": program,
        "target_names": target_names,
        "validation_result": validation_result,
        "replace_committed": True,
        "warnings": warnings,
    }


def _add_move(program, target, index, first_move):
    if index == 0 and first_move == "movej":
        program.MoveJ(target)
    else:
        program.MoveL(target)


def _configure_speed_point(program, point, index, target, first_move):
    program.setSpeed(point.linear_speed, point.joint_speed,
                     point.linear_accel, point.joint_accel)
    _add_move(program, target, index, first_move)


def _configure_pose_only_point(program, _point, index, target, first_move):
    _add_move(program, target, index, first_move)


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
            "RoboDK import requires a verified scanner TCP/tool mapping")
    if first_move not in ("movej", "movel"):
        raise ValueError("first_move must be movej or movel")
    robot_name = _required_name(robot_name, "robot")
    frame_name = _required_name(frame_name, "frame")
    tool_name = _required_name(tool_name, "tool")
    program_name = _required_name(program_name, "program")
    target_namespace = _optional_name(target_namespace, "target namespace")
    for point in result.points:
        values = (point.linear_speed, point.linear_accel, point.joint_speed, point.joint_accel)
        if not all(math.isfinite(float(value)) and float(value) > 0 for value in values):
            raise ValueError("Pose {} has invalid RoboDK speed command values".format(point.index))
    api = _import_api()
    rdk = api["Robolink"]()
    robot = _require_item(rdk, robot_name, api["ITEM_TYPE_ROBOT"], "robot")
    frame = _require_item(rdk, frame_name, api["ITEM_TYPE_FRAME"], "frame")
    tool = _require_item(rdk, tool_name, api["ITEM_TYPE_TOOL"], "tool")
    tool_mapping_verification = _verify_tool_mapping(
        rdk, robot, frame, tool, result.diagnostics,
        robot_name, frame_name, tool_name)

    namespace = target_namespace or (_safe_name(program_name) + "_")
    publication = _transactional_program_import(
        rdk, api, robot, frame, tool, result.points, program_name, namespace,
        first_move, replace, _configure_speed_point, _validate_program)
    instructions, expected_commands, parameter_readback = publication[
        "validation_result"]
    return {
        "station": tool_mapping_verification["station"],
        "program": program_name,
        "target_namespace": namespace,
        "pose_count": len(result.points),
        "instruction_count": len(instructions),
        "instructions": instructions,
        "expected_speed_commands": expected_commands,
        "parameter_readback_supported": parameter_readback is not None,
        "parameter_readback": parameter_readback,
        "tool_mapping_verification": tool_mapping_verification,
        "replace_committed": publication["replace_committed"],
        "backup_cleanup_warnings": publication["warnings"],
    }


def import_planned_path(records: Iterable[object], diagnostics: Mapping[str, object],
                        robot_name="UR10", frame_name="Frame 2",
                        tool_name="Creaform MetraSCAN",
                        program_name="IntegratedPlannedPath",
                        target_namespace: Optional[str] = None,
                        first_move="movej", replace=False):
    """Create a movement-only RoboDK program from ordered command poses.

    ``records`` accepts mappings, pose-like objects, or 8-value iterables in
    ``index,x,y,z,qw,qx,qy,qz`` order.  The values must already be transformed
    from scanner poses into the command-pose frame represented by
    ``diagnostics``.  Use ``pose_transform.transform_pose_records`` for that
    conversion.  The same validated mapping metadata required by
    :func:`import_speed_plan` is mandatory here.

    No ``setSpeed`` call is made.  As a result, this program is suitable for
    path/station inspection before constrained speed planning, but it is not a
    production motion-speed validation artifact.
    """
    points = _prepare_planned_path_points(records)
    if not isinstance(diagnostics, Mapping):
        raise ValueError("Planned path diagnostics must be a mapping")
    diagnostics = dict(diagnostics)
    if not diagnostics.get("extrinsic_validated", False):
        raise ValueError(
            "RoboDK import requires a verified scanner TCP/tool mapping")
    if first_move not in ("movej", "movel"):
        raise ValueError("first_move must be movej or movel")
    robot_name = _required_name(robot_name, "robot")
    frame_name = _required_name(frame_name, "frame")
    tool_name = _required_name(tool_name, "tool")
    program_name = _required_name(program_name, "program")
    target_namespace = _optional_name(target_namespace, "target namespace")

    api = _import_api()
    rdk = api["Robolink"]()
    robot = _require_item(rdk, robot_name, api["ITEM_TYPE_ROBOT"], "robot")
    frame = _require_item(rdk, frame_name, api["ITEM_TYPE_FRAME"], "frame")
    tool = _require_item(rdk, tool_name, api["ITEM_TYPE_TOOL"], "tool")
    tool_mapping_verification = _verify_tool_mapping(
        rdk, robot, frame, tool, diagnostics,
        robot_name, frame_name, tool_name)

    namespace = target_namespace or (_safe_name(program_name) + "_")
    publication = _transactional_program_import(
        rdk, api, robot, frame, tool, points, program_name, namespace,
        first_move, replace, _configure_pose_only_point,
        _validate_planned_path_program)
    instructions = publication["validation_result"]
    return {
        "station": tool_mapping_verification["station"],
        "program": program_name,
        "target_namespace": namespace,
        "pose_count": len(points),
        "source_pose_indices": [point.index for point in points],
        "instruction_count": len(instructions),
        "instructions": instructions,
        "speed_commands_applied": False,
        "tool_mapping_verification": tool_mapping_verification,
        "replace_committed": publication["replace_committed"],
        "backup_cleanup_warnings": publication["warnings"],
    }
