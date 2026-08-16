"""Scanner pose mapping for RoboDK TCP and separate-tool configurations.

``T_A_B`` denotes frame B expressed in frame A. Two explicit modes avoid a
common double-transform error:

``robodk_tcp_is_scanner``
    The selected RoboDK tool TCP is the scanner measurement frame. Desired
    scanner poses are already RoboDK target poses; RoboDK applies
    ``T_flange_scanner`` internally during IK.

``separate_tool_frame``
    The commanded RoboDK tool frame differs from the scanner frame. Then
    ``T_world_tool = T_world_scanner @ inverse(T_tool_scanner)``.
"""

from dataclasses import dataclass
import json
import math
import os
import tempfile
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCHEMA_VERSION = "1.2"
SUPPORTED_SCHEMA_VERSIONS = ("1.0", "1.1", SCHEMA_VERSION)
EPS = 1e-9
TRANSFORM_CONVENTION = "T_A_B maps coordinates from frame B to frame A"


MatrixTuple = Tuple[Tuple[float, ...], ...]


@dataclass(frozen=True)
class ExtrinsicConfig:
    config_id: str
    calibration_status: str
    mapping_mode: str
    t_tool_scanner: Optional[MatrixTuple] = None
    t_flange_scanner: Optional[MatrixTuple] = None
    t_flange_tool: Optional[MatrixTuple] = None
    schema_version: str = SCHEMA_VERSION
    source_pose_frame: str = "scanner"
    command_pose_frame: str = "scanner_tcp"
    length_unit: str = "mm"
    quaternion_order: str = "wxyz"
    robodk_station_name: str = ""
    robodk_robot_name: str = ""
    robodk_tool_name: str = ""
    robodk_frame_name: str = ""
    robodk_base_name: str = ""
    t_station_robot_base: Optional[MatrixTuple] = None
    t_station_reference_frame: Optional[MatrixTuple] = None
    t_base_workpiece: Optional[MatrixTuple] = None
    transform_convention: str = TRANSFORM_CONVENTION
    captured_from: str = ""
    captured_at_utc: str = ""
    position_tolerance_mm: float = 0.01
    orientation_tolerance_deg: float = 0.01
    notes: str = ""

    @property
    def validated(self) -> bool:
        """True when usable for RoboDK simulation import."""
        return self.calibration_status in ("station_verified", "validated")

    @property
    def physically_validated(self) -> bool:
        return self.calibration_status == "validated"

    @property
    def matrix(self) -> np.ndarray:
        if self.t_tool_scanner is None:
            raise ValueError("This mapping does not use T_tool_scanner")
        return np.asarray(self.t_tool_scanner, dtype=float)

    @property
    def base_workpiece_matrix(self) -> np.ndarray:
        if self.t_base_workpiece is None:
            raise ValueError("This configuration does not define T_base_workpiece")
        return np.asarray(self.t_base_workpiece, dtype=float)

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "config_id": self.config_id,
            "calibration_status": self.calibration_status,
            "mapping_mode": self.mapping_mode,
            "source_pose_frame": self.source_pose_frame,
            "command_pose_frame": self.command_pose_frame,
            "length_unit": self.length_unit,
            "quaternion_order": self.quaternion_order,
            "T_tool_scanner": _rows_or_none(self.t_tool_scanner),
            "T_flange_scanner": _rows_or_none(self.t_flange_scanner),
            "T_flange_tool": _rows_or_none(self.t_flange_tool),
            "robodk_station_name": self.robodk_station_name,
            "robodk_robot_name": self.robodk_robot_name,
            "robodk_tool_name": self.robodk_tool_name,
            "robodk_frame_name": self.robodk_frame_name,
            "robodk_base_name": self.robodk_base_name,
            "T_station_robot_base": _rows_or_none(self.t_station_robot_base),
            "T_station_reference_frame": _rows_or_none(self.t_station_reference_frame),
            "T_base_workpiece": _rows_or_none(self.t_base_workpiece),
            "transform_convention": self.transform_convention,
            "captured_from": self.captured_from,
            "captured_at_utc": self.captured_at_utc,
            "position_tolerance_mm": self.position_tolerance_mm,
            "orientation_tolerance_deg": self.orientation_tolerance_deg,
            "notes": self.notes,
        }


def _rows_or_none(matrix):
    return None if matrix is None else [list(row) for row in matrix]


def _validate_rigid_transform(matrix: np.ndarray, label="transform") -> None:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("{} must be a finite 4x4 matrix".format(label))
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("{} last row must be [0,0,0,1]".format(label))
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("{} rotation must be orthonormal".format(label))
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ValueError("{} rotation determinant must be +1".format(label))


def _matrix_value(data, key, required):
    raw = data.get(key)
    if raw is None:
        if required:
            raise ValueError("{} is required for this mapping mode".format(key))
        return None
    matrix = np.asarray(raw, dtype=float)
    _validate_rigid_transform(matrix, key)
    return tuple(tuple(float(value) for value in row) for row in matrix)


def parse_extrinsic_config(data: Dict[str, object]) -> ExtrinsicConfig:
    version = str(data.get("schema_version", ""))
    if version not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError("Unsupported extrinsic schema_version")
    config_id = str(data.get("config_id", "")).strip()
    if not config_id:
        raise ValueError("Extrinsic config_id is required")
    status = str(data.get("calibration_status", "")).strip().lower()
    allowed_status = ("validated", "station_verified", "unvalidated", "example_only")
    if status not in allowed_status:
        raise ValueError("Unsupported calibration_status")
    mode = str(data.get("mapping_mode", "separate_tool_frame")).strip()
    if mode not in ("robodk_tcp_is_scanner", "separate_tool_frame"):
        raise ValueError("Unsupported mapping_mode")
    if data.get("source_pose_frame") != "scanner":
        raise ValueError("source_pose_frame must be scanner")
    expected_command_frame = "scanner_tcp" if mode == "robodk_tcp_is_scanner" else "tool"
    if data.get("command_pose_frame") != expected_command_frame:
        raise ValueError("command_pose_frame must be {} for {}".format(
            expected_command_frame, mode))
    if data.get("length_unit") != "mm":
        raise ValueError("Only millimetre translations are currently supported")
    if data.get("quaternion_order") != "wxyz":
        raise ValueError("quaternion_order must be wxyz")
    transform_convention = str(
        data.get("transform_convention", TRANSFORM_CONVENTION)).strip()
    if transform_convention != TRANSFORM_CONVENTION:
        raise ValueError("Unsupported transform_convention")

    t_tool_scanner = _matrix_value(
        data, "T_tool_scanner", mode == "separate_tool_frame")
    t_flange_scanner = _matrix_value(
        data, "T_flange_scanner", mode == "robodk_tcp_is_scanner")
    # Kept optional while parsing so existing separate-tool calibration files
    # remain usable for planning/export.  RoboDK import requires this matrix
    # and verifies it against the selected station Tool before publishing a
    # program; see robodk_bridge._verify_tool_mapping.
    t_flange_tool = _matrix_value(data, "T_flange_tool", False)
    t_station_robot_base = _matrix_value(
        data, "T_station_robot_base", False)
    t_station_reference_frame = _matrix_value(
        data, "T_station_reference_frame", False)
    t_base_workpiece = _matrix_value(data, "T_base_workpiece", False)
    coordinate_matrices = (
        t_station_robot_base, t_station_reference_frame, t_base_workpiece)
    has_explicit_base_chain = (
        t_station_robot_base is not None or t_base_workpiece is not None)
    if has_explicit_base_chain:
        if not all(matrix is not None for matrix in coordinate_matrices):
            raise ValueError(
                "T_station_robot_base, T_station_reference_frame and "
                "T_base_workpiece must be provided together")
        station_base = np.asarray(t_station_robot_base, dtype=float)
        station_workpiece = np.asarray(t_station_reference_frame, dtype=float)
        base_workpiece = np.asarray(t_base_workpiece, dtype=float)
        derived = np.linalg.inv(station_base) @ station_workpiece
        if not np.allclose(derived, base_workpiece, atol=1e-6):
            raise ValueError(
                "Coordinate chain mismatch: inv(T_station_robot_base) @ "
                "T_station_reference_frame must equal T_base_workpiece")
    tool_name = str(data.get("robodk_tool_name", "")).strip()
    if mode == "robodk_tcp_is_scanner" and not tool_name:
        raise ValueError("robodk_tool_name is required when RoboDK TCP is scanner")
    position_tolerance = float(data.get("position_tolerance_mm", 0.01))
    orientation_tolerance = float(data.get("orientation_tolerance_deg", 0.01))
    if not math.isfinite(position_tolerance) or position_tolerance <= 0:
        raise ValueError("position_tolerance_mm must be positive and finite")
    if not math.isfinite(orientation_tolerance) or orientation_tolerance <= 0:
        raise ValueError("orientation_tolerance_deg must be positive and finite")

    return ExtrinsicConfig(
        schema_version=version,
        config_id=config_id,
        calibration_status=status,
        mapping_mode=mode,
        t_tool_scanner=t_tool_scanner,
        t_flange_scanner=t_flange_scanner,
        t_flange_tool=t_flange_tool,
        command_pose_frame=expected_command_frame,
        robodk_station_name=str(data.get("robodk_station_name", "")).strip(),
        robodk_robot_name=str(data.get("robodk_robot_name", "")).strip(),
        robodk_tool_name=tool_name,
        robodk_frame_name=str(data.get("robodk_frame_name", "")).strip(),
        robodk_base_name=str(data.get("robodk_base_name", "")).strip(),
        t_station_robot_base=t_station_robot_base,
        t_station_reference_frame=t_station_reference_frame,
        t_base_workpiece=t_base_workpiece,
        transform_convention=transform_convention,
        captured_from=str(data.get("captured_from", "")).strip(),
        captured_at_utc=str(data.get("captured_at_utc", "")).strip(),
        position_tolerance_mm=position_tolerance,
        orientation_tolerance_deg=orientation_tolerance,
        notes=str(data.get("notes", "")),
    )


def load_extrinsic_config(file_path: str) -> ExtrinsicConfig:
    with open(file_path, "r", encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("Extrinsic configuration root must be an object")
    return parse_extrinsic_config(data)


def save_extrinsic_config(file_path: str, config: ExtrinsicConfig) -> str:
    """Validate and atomically save one scanner/station mapping JSON file."""
    if not isinstance(config, ExtrinsicConfig):
        raise TypeError("config must be an ExtrinsicConfig")
    # Round-trip through the parser before touching the destination so the
    # written file can always be loaded by this application version.
    data = config.to_dict()
    parse_extrinsic_config(data)
    destination = os.path.abspath(os.fspath(file_path))
    directory = os.path.dirname(destination)
    if not os.path.isdir(directory):
        raise ValueError("Configuration directory does not exist: {}".format(directory))
    fd, temporary = tempfile.mkstemp(
        prefix="." + os.path.basename(destination) + ".",
        suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    return destination


def quaternion_to_rotation(quaternion: Sequence[float]) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < EPS:
        raise ValueError("Quaternion norm is zero")
    qw, qx, qy, qz = q / norm
    return np.asarray([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=float)


def rotation_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("Rotation must be a finite 3x3 matrix")
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        qw, qx = 0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale
        qy, qz = (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            qw, qx = (rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale
            qy, qz = (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            qw, qx = (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[0, 1] + rotation[1, 0]) / scale
            qy, qz = 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            qw, qx = (rotation[1, 0] - rotation[0, 1]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale
            qy, qz = (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale
    q = np.asarray([qw, qx, qy, qz], dtype=float)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return tuple(float(value) for value in q)


def pose_matrix(record: Dict[str, object]) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_rotation(
        [record["qw"], record["qx"], record["qy"], record["qz"]])
    matrix[:3, 3] = [float(record["x"]), float(record["y"]), float(record["z"])]
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Pose contains non-finite values")
    return matrix


def matrix_pose(matrix: np.ndarray, index: int) -> Dict[str, object]:
    matrix = np.asarray(matrix, dtype=float)
    _validate_rigid_transform(matrix, "pose")
    qw, qx, qy, qz = rotation_to_quaternion(matrix[:3, :3])
    return {
        "index": int(index),
        "x": float(matrix[0, 3]), "y": float(matrix[1, 3]), "z": float(matrix[2, 3]),
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
    }


def transform_scanner_pose_to_tool(record: Dict[str, object],
                                   config: ExtrinsicConfig) -> Dict[str, object]:
    world_scanner = pose_matrix(record)
    if config.mapping_mode == "robodk_tcp_is_scanner":
        return matrix_pose(world_scanner, int(record["index"]))
    world_tool = world_scanner @ np.linalg.inv(config.matrix)
    return matrix_pose(world_tool, int(record["index"]))


def transform_workpiece_pose_to_base(record: Dict[str, object],
                                     config: ExtrinsicConfig) -> Dict[str, object]:
    """Express a pose from the workpiece/Frame coordinate system in robot Base.

    This is intentionally separate from :func:`transform_scanner_pose_to_tool`.
    RoboDK imports remain Frame-relative; Base poses are for offline robot
    kinematics, reachability and later D-H inverse-kinematics analysis.
    """
    base_pose = config.base_workpiece_matrix @ pose_matrix(record)
    return matrix_pose(base_pose, int(record["index"]))


def transform_pose_records(records: Iterable[Dict[str, object]],
                           config: ExtrinsicConfig) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    source_records = [dict(record) for record in records]
    command_records = [transform_scanner_pose_to_tool(record, config)
                       for record in source_records]
    return command_records, {
        "extrinsic_applied": config.mapping_mode == "separate_tool_frame",
        "extrinsic_validated": config.validated,
        "physical_calibration_validated": config.physically_validated,
        "extrinsic_config_id": config.config_id,
        "calibration_status": config.calibration_status,
        "tool_mapping_mode": config.mapping_mode,
        "source_pose_frame": "scanner",
        "command_pose_frame": config.command_pose_frame,
        "T_tool_scanner": _rows_or_none(config.t_tool_scanner),
        "T_flange_scanner": _rows_or_none(config.t_flange_scanner),
        "T_flange_tool": _rows_or_none(config.t_flange_tool),
        "expected_robodk_station_name": config.robodk_station_name,
        "expected_robodk_robot_name": config.robodk_robot_name,
        "expected_robodk_tool_name": config.robodk_tool_name,
        "expected_robodk_frame_name": config.robodk_frame_name,
        "expected_robodk_base_name": config.robodk_base_name,
        "T_station_robot_base": _rows_or_none(config.t_station_robot_base),
        "T_station_reference_frame": _rows_or_none(config.t_station_reference_frame),
        "T_base_workpiece": _rows_or_none(config.t_base_workpiece),
        "transform_convention": config.transform_convention,
        "captured_from": config.captured_from,
        "captured_at_utc": config.captured_at_utc,
        "tool_position_tolerance_mm": config.position_tolerance_mm,
        "tool_orientation_tolerance_deg": config.orientation_tolerance_deg,
        "source_pose_records": source_records,
    }
