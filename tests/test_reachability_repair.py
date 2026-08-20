import unittest

from reachability_repair import (
    build_ordered_path_records,
    repair_unreachable_points,
)
from state import ViewpointRecord


class Point:
    def __init__(self, x, y=0.0, z=0.0):
        self._x = float(x)
        self._y = float(y)
        self._z = float(z)

    def X(self):
        return self._x

    def Y(self):
        return self._y

    def Z(self):
        return self._z


Q = (1.0, 0.0, 0.0, 0.0)


class ReachabilityRepairTests(unittest.TestCase):
    def _candidate_records(self, face_count, candidates_per_face):
        records = []
        for global_index in range(face_count * (1 + candidates_per_face)):
            if global_index < face_count:
                face_index = global_index
                candidate_index = 0
            else:
                offset = global_index - face_count
                face_index = offset // candidates_per_face
                candidate_index = offset % candidates_per_face + 1
            records.append(ViewpointRecord(
                point=Point(global_index),
                pose=Q,
                face_index=face_index,
                kind="center" if candidate_index == 0 else "candidate",
                global_index=global_index,
                candidate_index=candidate_index,
            ))
        return records

    def _evaluate(self, records, unreachable_x=30.0):
        points = []
        for record in records:
            reachable = abs(float(record["x"]) - unreachable_x) > 1e-9
            points.append({
                "index": record["index"],
                "reachable": reachable,
                "reason": "reachable" if reachable else "no_ik_solution",
                "joint_jump_l2_deg": 1.0 if reachable else None,
                "joint_jump_max_deg": 1.0 if reachable else None,
            })
        unreachable_count = sum(1 for point in points if not point["reachable"])
        return {
            "points": points,
            "reachable_indices": [point["index"] for point in points if point["reachable"]],
            "unreachable_indices": [point["index"] for point in points if not point["reachable"]],
            "reachable_count": len(points) - unreachable_count,
            "unreachable_count": unreachable_count,
            "all_reachable": unreachable_count == 0,
        }

    def test_repair_chooses_same_face_candidate_and_preserves_path_order(self):
        face_count = 4
        candidates_per_face = 2
        candidate_records = self._candidate_records(face_count, candidates_per_face)
        optimal_viewpoints = [Point(30.0)]
        optimal_viewpoints_with_pose = [(optimal_viewpoints[0], Q)]
        optimal_viewpoint_records = [ViewpointRecord(
            point=optimal_viewpoints[0], pose=Q, face_index=3, kind="optimal",
            global_index=3, candidate_index=0,
        )]
        optimal_path = [0]
        source_records = build_ordered_path_records(
            optimal_path, optimal_viewpoints_with_pose, optimal_viewpoint_records)

        result = repair_unreachable_points(
            source_records=source_records,
            pose_metadata={"extrinsic_validated": True},
            optimal_path=optimal_path,
            optimal_viewpoints=optimal_viewpoints,
            optimal_viewpoints_with_pose=optimal_viewpoints_with_pose,
            optimal_viewpoint_records=optimal_viewpoint_records,
            candidate_records=candidate_records,
            evaluate_fn=lambda records: self._evaluate(records, unreachable_x=30.0),
            face_count=face_count,
            candidates_per_face=candidates_per_face,
        )

        self.assertEqual(len(result.replacements), 1)
        replacement = result.replacements[0]
        self.assertEqual(replacement.face_index, 3)
        self.assertEqual(replacement.old_candidate_index, 0)
        self.assertEqual(replacement.new_candidate_index, 2)
        self.assertEqual(replacement.new_global_index, 11)
        self.assertEqual(result.unrepaired_indices, [])
        self.assertEqual(result.updated_optimal_viewpoint_records[0].candidate_index, 2)
        self.assertEqual(result.updated_optimal_viewpoints[0].X(), 11.0)
        self.assertEqual(result.updated_optimal_viewpoints_with_pose[0][0].X(), 11.0)
        self.assertEqual(result.final_report["unreachable_count"], 0)

    def test_repair_uses_face_provenance_not_optimal_index(self):
        face_count = 5
        candidates_per_face = 2
        candidate_records = self._candidate_records(face_count, candidates_per_face)
        optimal_viewpoints = [Point(30.0)]
        optimal_viewpoints_with_pose = [(optimal_viewpoints[0], Q)]
        optimal_viewpoint_records = [ViewpointRecord(
            point=optimal_viewpoints[0], pose=Q, face_index=4, kind="optimal",
            global_index=0, candidate_index=0,
        )]
        optimal_path = [0]
        source_records = build_ordered_path_records(
            optimal_path, optimal_viewpoints_with_pose, optimal_viewpoint_records)

        result = repair_unreachable_points(
            source_records=source_records,
            pose_metadata={"extrinsic_validated": True},
            optimal_path=optimal_path,
            optimal_viewpoints=optimal_viewpoints,
            optimal_viewpoints_with_pose=optimal_viewpoints_with_pose,
            optimal_viewpoint_records=optimal_viewpoint_records,
            candidate_records=candidate_records,
            evaluate_fn=lambda records: self._evaluate(records, unreachable_x=30.0),
            face_count=face_count,
            candidates_per_face=candidates_per_face,
        )

        self.assertEqual(len(result.replacements), 1)
        replacement = result.replacements[0]
        self.assertEqual(replacement.face_index, 4)
        self.assertEqual(replacement.new_global_index, 14)
        self.assertEqual(result.updated_optimal_viewpoint_records[0].face_index, 4)

    def test_repair_leaves_point_unrepaired_when_all_candidates_fail(self):
        face_count = 4
        candidates_per_face = 2
        candidate_records = self._candidate_records(face_count, candidates_per_face)
        optimal_viewpoints = [Point(30.0)]
        optimal_viewpoints_with_pose = [(optimal_viewpoints[0], Q)]
        optimal_viewpoint_records = [ViewpointRecord(
            point=optimal_viewpoints[0], pose=Q, face_index=3, kind="optimal",
            global_index=3, candidate_index=0,
        )]
        optimal_path = [0]
        source_records = build_ordered_path_records(
            optimal_path, optimal_viewpoints_with_pose, optimal_viewpoint_records)

        result = repair_unreachable_points(
            source_records=source_records,
            pose_metadata={"extrinsic_validated": True},
            optimal_path=optimal_path,
            optimal_viewpoints=optimal_viewpoints,
            optimal_viewpoints_with_pose=optimal_viewpoints_with_pose,
            optimal_viewpoint_records=optimal_viewpoint_records,
            candidate_records=candidate_records,
            evaluate_fn=lambda records: {
                "points": [
                    {"index": record["index"], "reachable": False,
                     "reason": "no_ik_solution",
                     "joint_jump_l2_deg": None,
                     "joint_jump_max_deg": None}
                    for record in records
                ],
                "reachable_indices": [],
                "unreachable_indices": [record["index"] for record in records],
                "reachable_count": 0,
                "unreachable_count": len(records),
                "all_reachable": False,
            },
            face_count=face_count,
            candidates_per_face=candidates_per_face,
        )

        self.assertEqual(result.replacements, [])
        self.assertEqual(result.unrepaired_indices, [1])
        self.assertEqual(result.final_report["unreachable_count"], 1)


if __name__ == "__main__":
    unittest.main()
