"""Helpers for repairing unreachable UR10 path points.

The module stays free of Qt and OCC viewer state so it can be unit-tested
independently from the desktop application.
"""

from dataclasses import dataclass, field
import hashlib
import json
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from config import NUM_CANDIDATES_PER_FACE
from planning import point_distance
from state import ViewpointRecord
from viewpoints import ordered_face_candidates


@dataclass(frozen=True)
class ReplacementDecision:
    path_order_index: int
    path_position: int
    optimal_index: int
    face_index: int
    old_candidate_index: int
    new_candidate_index: int
    old_global_index: int
    new_global_index: int
    path_length_delta: float
    joint_jump_score: float
    selected_unreachable_index: int
    candidate_report: Mapping[str, object] = field(default_factory=dict)


@dataclass
class RepairResult:
    replacements: List[ReplacementDecision] = field(default_factory=list)
    unrepaired_indices: List[int] = field(default_factory=list)
    final_report: Dict[str, object] = field(default_factory=dict)
    updated_optimal_viewpoints: List[object] = field(default_factory=list)
    updated_optimal_viewpoints_with_pose: List[Tuple[object, object]] = field(default_factory=list)
    updated_optimal_viewpoint_records: List[ViewpointRecord] = field(default_factory=list)
    final_source_records: List[Dict[str, object]] = field(default_factory=list)


def build_viewpoint_records(viewpoints_with_pose: Sequence[Tuple[object, object]],
                            face_count: int,
                            candidates_per_face: int = NUM_CANDIDATES_PER_FACE) -> List[ViewpointRecord]:
    """Build provenance records for the current candidate viewpoint layout."""
    if face_count < 0:
        raise ValueError("face_count must be non-negative")
    expected = face_count * (1 + candidates_per_face)
    if len(viewpoints_with_pose) != expected:
        raise ValueError(
            "Candidate viewpoints are incomplete; regenerate candidate viewpoints before repairing reachability")

    records = []
    for global_index, (point, pose) in enumerate(viewpoints_with_pose):
        if global_index < face_count:
            face_index = global_index
            candidate_index = 0
            kind = "center"
        else:
            offset = global_index - face_count
            face_index = offset // candidates_per_face
            candidate_index = offset % candidates_per_face + 1
            kind = "candidate"
        if face_index >= face_count:
            raise ValueError(
                "Candidate viewpoint layout is inconsistent with the face count")
        records.append(ViewpointRecord(
            point=point,
            pose=pose,
            face_index=face_index,
            kind=kind,
            global_index=global_index,
            candidate_index=candidate_index,
        ))
    return records


def build_ordered_path_records(optimal_path: Sequence[int],
                               optimal_viewpoints_with_pose: Sequence[Tuple[object, object]],
                               optimal_viewpoint_records: Sequence[ViewpointRecord]) -> List[Dict[str, object]]:
    """Build ordered source records with provenance for the active path."""
    if not optimal_viewpoints_with_pose:
        raise ValueError("No path poses are available")
    if len(optimal_viewpoint_records) != len(optimal_viewpoints_with_pose):
        raise ValueError("Optimal viewpoint provenance is incomplete")

    ordered = list(optimal_path) if optimal_path else list(range(len(optimal_viewpoints_with_pose)))
    records = []
    for order_index, viewpoint_index in enumerate(ordered, start=1):
        if viewpoint_index < 0 or viewpoint_index >= len(optimal_viewpoints_with_pose):
            raise IndexError("Path index out of range: {}".format(viewpoint_index))
        point, pose = optimal_viewpoints_with_pose[viewpoint_index]
        record = optimal_viewpoint_records[viewpoint_index]
        qw, qx, qy, qz = pose
        records.append({
            "index": order_index,
            "x": point.X(), "y": point.Y(), "z": point.Z(),
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
            "optimal_index": viewpoint_index,
            "face_index": record.face_index,
            "candidate_index": record.candidate_index,
            "global_index": record.global_index,
            "kind": record.kind,
        })
    return records


def compute_path_signature(source_records: Sequence[Mapping[str, object]],
                           optimal_path: Sequence[int],
                           pose_metadata: Mapping[str, object]) -> str:
    payload = {
        "optimal_path": list(optimal_path),
        "source_records": [
            {
                "index": int(record["index"]),
                "x": float(record["x"]),
                "y": float(record["y"]),
                "z": float(record["z"]),
                "qw": float(record["qw"]),
                "qx": float(record["qx"]),
                "qy": float(record["qy"]),
                "qz": float(record["qz"]),
                "optimal_index": int(record.get("optimal_index", -1)),
                "face_index": int(record.get("face_index", -1)),
                "candidate_index": int(record.get("candidate_index", -1)),
                "global_index": int(record.get("global_index", -1)),
                "kind": str(record.get("kind", "")),
            }
            for record in source_records
        ],
        "pose_metadata": {
            "extrinsic_config_id": pose_metadata.get("extrinsic_config_id", ""),
            "extrinsic_config_sha256": pose_metadata.get("extrinsic_config_sha256", ""),
            "source_pose_frame": pose_metadata.get("source_pose_frame", ""),
            "command_pose_frame": pose_metadata.get("command_pose_frame", ""),
            "tool_mapping_mode": pose_metadata.get("tool_mapping_mode", ""),
        },
    }
    digest = hashlib.sha256(
        json.dumps(payload, sort_keys=True, ensure_ascii=False).encode("utf-8")
    ).hexdigest()
    return digest


def build_candidate_lookup(candidate_records: Sequence[ViewpointRecord],
                           face_count: int,
                           candidates_per_face: int = NUM_CANDIDATES_PER_FACE) -> Dict[int, List[ViewpointRecord]]:
    lookup = {}
    for face_index in range(face_count):
        lookup[face_index] = ordered_face_candidates(candidate_records, face_index, face_count, candidates_per_face)
    return lookup


def _point_distance(point_a, point_b):
    return point_distance(point_a, point_b)


def _local_path_delta(prev_point, old_point, new_point, next_point):
    old_total = 0.0
    new_total = 0.0
    if prev_point is not None:
        old_total += _point_distance(prev_point, old_point)
        new_total += _point_distance(prev_point, new_point)
    if next_point is not None:
        old_total += _point_distance(old_point, next_point)
        new_total += _point_distance(new_point, next_point)
    return new_total - old_total


def _joint_jump_score(report_point: Mapping[str, object], candidate_report: Mapping[str, object]) -> float:
    score = 0.0
    for key in ("joint_jump_l2_deg", "joint_jump_max_deg"):
        value = candidate_report.get(key)
        if isinstance(value, (int, float)):
            score += float(value)
    current_value = report_point.get("joint_jump_l2_deg")
    if isinstance(current_value, (int, float)):
        score += float(current_value) * 0.25
    return score


def repair_unreachable_points(
    source_records: Sequence[Mapping[str, object]],
    pose_metadata: Mapping[str, object],
    optimal_path: Sequence[int],
    optimal_viewpoints: Sequence[object],
    optimal_viewpoints_with_pose: Sequence[Tuple[object, object]],
    optimal_viewpoint_records: Sequence[ViewpointRecord],
    candidate_records: Sequence[ViewpointRecord],
    evaluate_fn: Callable[[Sequence[Mapping[str, object]]], Mapping[str, object]],
    face_count: int,
    candidates_per_face: int = NUM_CANDIDATES_PER_FACE,
) -> RepairResult:
    if not source_records:
        raise ValueError("No path records are available")
    if len(optimal_viewpoints) != len(optimal_viewpoints_with_pose):
        raise ValueError("Optimal viewpoint pose data is incomplete")
    if len(optimal_viewpoint_records) != len(optimal_viewpoints):
        raise ValueError("Optimal viewpoint provenance is incomplete")
    if len(source_records) != len(optimal_path):
        raise ValueError("Source records and path order do not match")

    working_viewpoints = list(optimal_viewpoints)
    working_viewpoints_with_pose = list(optimal_viewpoints_with_pose)
    working_records = list(optimal_viewpoint_records)
    current_report = dict(evaluate_fn(source_records))
    current_report_points = list(current_report.get("points", []))
    if len(current_report_points) != len(source_records):
        raise ValueError("Reachability report does not match the ordered path")

    result = RepairResult(
        final_report=current_report,
        updated_optimal_viewpoints=working_viewpoints,
        updated_optimal_viewpoints_with_pose=working_viewpoints_with_pose,
        updated_optimal_viewpoint_records=working_records,
        final_source_records=[dict(record) for record in source_records],
    )

    def build_trial_records():
        return build_ordered_path_records(optimal_path, working_viewpoints_with_pose, working_records)

    repaired_any = True
    while repaired_any:
        repaired_any = False
        current_report_points = list(current_report.get("points", []))
        unreachable_points = [point for point in current_report_points if not point.get("reachable", False)]
        if not unreachable_points:
            break

        for report_point in unreachable_points:
            path_order_index = int(report_point["index"])
            path_position = path_order_index - 1
            if path_position < 0 or path_position >= len(optimal_path):
                continue
            optimal_index = int(optimal_path[path_position])
            if optimal_index < 0 or optimal_index >= len(working_records):
                continue
            face_index = working_records[optimal_index].face_index
            candidates = ordered_face_candidates(candidate_records, face_index, face_count, candidates_per_face)
            current_record = working_records[optimal_index]
            current_global_index = current_record.global_index
            if not candidates:
                if path_order_index not in result.unrepaired_indices:
                    result.unrepaired_indices.append(path_order_index)
                continue

            prev_point = None
            next_point = None
            if path_position > 0:
                prev_point = working_viewpoints[int(optimal_path[path_position - 1])]
            if path_position + 1 < len(optimal_path):
                next_point = working_viewpoints[int(optimal_path[path_position + 1])]
            best_choice = None

            for candidate in candidates:
                if candidate.global_index == current_global_index:
                    continue
                trial_viewpoints = list(working_viewpoints)
                trial_viewpoints_with_pose = list(working_viewpoints_with_pose)
                trial_records = list(working_records)
                trial_viewpoints[optimal_index] = candidate.point
                trial_viewpoints_with_pose[optimal_index] = (candidate.point, candidate.pose)
                trial_records[optimal_index] = candidate
                trial_source_records = build_ordered_path_records(optimal_path, trial_viewpoints_with_pose, trial_records)
                candidate_report = dict(evaluate_fn(trial_source_records))
                trial_report_points = list(candidate_report.get("points", []))
                if len(trial_report_points) != len(trial_source_records):
                    continue
                trial_target_point = trial_report_points[path_position]
                if not trial_target_point.get("reachable", False):
                    continue

                current_point = working_viewpoints[optimal_index]
                path_delta = _local_path_delta(prev_point, current_point, candidate.point, next_point)
                joint_jump_score = _joint_jump_score(report_point, trial_target_point)
                score = (
                    int(candidate_report.get("unreachable_count", 0)),
                    float(path_delta if path_delta >= 0 else 0.0),
                    abs(float(path_delta)),
                    joint_jump_score,
                    _point_distance(current_point, candidate.point),
                    int(candidate.candidate_index),
                    int(candidate.global_index),
                )
                if best_choice is None or score < best_choice[0]:
                    best_choice = (score, candidate, trial_viewpoints, trial_viewpoints_with_pose,
                                   trial_records, trial_source_records, candidate_report, path_delta,
                                   joint_jump_score)

            if best_choice is None:
                if path_order_index not in result.unrepaired_indices:
                    result.unrepaired_indices.append(path_order_index)
                continue

            _, candidate, trial_viewpoints, trial_viewpoints_with_pose, trial_records, trial_source_records, candidate_report, path_delta, joint_jump_score = best_choice
            working_viewpoints = trial_viewpoints
            working_viewpoints_with_pose = trial_viewpoints_with_pose
            working_records = trial_records
            current_report = candidate_report
            current_report_points = list(current_report.get("points", []))
            result.replacements.append(ReplacementDecision(
                path_order_index=path_order_index,
                path_position=path_position,
                optimal_index=optimal_index,
                face_index=face_index,
                old_candidate_index=current_record.candidate_index,
                new_candidate_index=candidate.candidate_index,
                old_global_index=current_record.global_index,
                new_global_index=candidate.global_index,
                path_length_delta=path_delta,
                joint_jump_score=joint_jump_score,
                selected_unreachable_index=path_order_index,
                candidate_report=candidate_report,
            ))
            result.updated_optimal_viewpoints = working_viewpoints
            result.updated_optimal_viewpoints_with_pose = working_viewpoints_with_pose
            result.updated_optimal_viewpoint_records = working_records
            result.final_source_records = trial_source_records
            result.final_report = candidate_report
            repaired_any = True
            break

    if not result.final_report:
        result.final_report = current_report
    result.unrepaired_indices = [
        int(point["index"])
        for point in result.final_report.get("points", [])
        if not point.get("reachable", False)
    ]
    return result
