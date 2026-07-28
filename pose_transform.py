"""Validated scanner-to-tool rigid transforms for ordered measurement poses.

Convention: ``T_A_B`` is the pose of frame B expressed in frame A and maps
coordinates from B into A.  Given a desired scanner pose ``T_world_scanner``
and the calibrated mounting transform ``T_tool_scanner``::

    T_world_tool = T_world_scanner @ inverse(T_tool_scanner)

Positions are millimetres and quaternions use ``(w, x, y, z)``.
"""

from dataclasses import dataclass
import json
import math
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


SCHEMA_VERSION = "1.0"
EPS = 1e-9


@dataclass(frozen=True)
class ExtrinsicConfig:
    config_id: str
    calibration_status: str
    t_tool_scanner: Tuple[Tuple[float, ...], ...]
    schema_version: str = SCHEMA_VERSION
    source_pose_frame: str = "scanner"
    command_pose_frame: str = "tool"
    length_unit: str = "mm"
    quaternion_order: str = "wxyz"
    notes: str = ""

    @property
    def validated(self) -> bool:
        return self.calibration_status == "validated"

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(self.t_tool_scanner, dtype=float)

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "config_id": self.config_id,
            "calibration_status": self.calibration_status,
            "source_pose_frame": self.source_pose_frame,
            "command_pose_frame": self.command_pose_frame,
            "length_unit": self.length_unit,
            "quaternion_order": self.quaternion_order,
            "T_tool_scanner": [list(row) for row in self.t_tool_scanner],
            "notes": self.notes,
        }


def _validate_rigid_transform(matrix: np.ndarray) -> None:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("T_tool_scanner must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("T_tool_scanner last row must be [0,0,0,1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("T_tool_scanner rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ValueError("T_tool_scanner rotation determinant must be +1")


def parse_extrinsic_config(data: Dict[str, object]) -> ExtrinsicConfig:
    if str(data.get("schema_version", "")) != SCHEMA_VERSION:
        raise ValueError("Unsupported extrinsic schema_version")
    config_id = str(data.get("config_id", "")).strip()
    if not config_id:
        raise ValueError("Extrinsic config_id is required")
    status = str(data.get("calibration_status", "")).strip().lower()
    if status not in ("validated", "unvalidated", "example_only"):
        raise ValueError("calibration_status must be validated, unvalidated or example_only")
    if data.get("source_pose_frame") != "scanner":
        raise ValueError("source_pose_frame must be scanner")
    if data.get("command_pose_frame") != "tool":
        raise ValueError("command_pose_frame must be tool")
    if data.get("length_unit") != "mm":
        raise ValueError("Only millimetre extrinsic translations are currently supported")
    if data.get("quaternion_order") != "wxyz":
        raise ValueError("quaternion_order must be wxyz")
    matrix = np.asarray(data.get("T_tool_scanner"), dtype=float)
    _validate_rigid_transform(matrix)
    return ExtrinsicConfig(
        config_id=config_id,
        calibration_status=status,
        t_tool_scanner=tuple(tuple(float(value) for value in row) for row in matrix),
        notes=str(data.get("notes", "")),
    )


def load_extrinsic_config(file_path: str) -> ExtrinsicConfig:
    with open(file_path, "r", encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("Extrinsic configuration root must be an object")
    return parse_extrinsic_config(data)


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
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
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
    _validate_rigid_transform(matrix)
    qw, qx, qy, qz = rotation_to_quaternion(matrix[:3, :3])
    return {
        "index": int(index),
        "x": float(matrix[0, 3]), "y": float(matrix[1, 3]), "z": float(matrix[2, 3]),
        "qw": qw, "qx": qx, "qy": qy, "qz": qz,
    }


def transform_scanner_pose_to_tool(record: Dict[str, object],
                                   config: ExtrinsicConfig) -> Dict[str, object]:
    world_scanner = pose_matrix(record)
    world_tool = world_scanner @ np.linalg.inv(config.matrix)
    return matrix_pose(world_tool, int(record["index"]))


def transform_pose_records(records: Iterable[Dict[str, object]],
                           config: ExtrinsicConfig) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    source_records = [dict(record) for record in records]
    command_records = [transform_scanner_pose_to_tool(record, config)
                       for record in source_records]
    return command_records, {
        "extrinsic_applied": True,
        "extrinsic_validated": config.validated,
        "extrinsic_config_id": config.config_id,
        "calibration_status": config.calibration_status,
        "source_pose_frame": "scanner",
        "command_pose_frame": "tool",
        "T_tool_scanner": [list(row) for row in config.t_tool_scanner],
        "source_pose_records": source_records,
    }
