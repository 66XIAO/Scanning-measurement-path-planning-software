import csv
import math
import os
import sys
import tempfile
import unittest

from export_utils import write_speed_plan_csv
from speed_planning_core import ConstraintProfile, plan_speed_profile, prepare_pose_samples


def sample_records(count=8):
    records = []
    for i in range(count):
        angle = math.radians(i * 4.0)
        records.append({
            "index": i + 1,
            "x": i * 80.0,
            "y": 25.0 * math.sin(i * 0.5),
            "z": 5.0 * math.cos(i * 0.4),
            "qw": math.cos(angle / 2.0),
            "qx": 0.0,
            "qy": 0.0,
            "qz": math.sin(angle / 2.0),
        })
    return records


class SpeedPlanningTests(unittest.TestCase):
    def test_quaternion_sign_continuity(self):
        records = sample_records(3)
        records[1].update({"qw": -records[1]["qw"], "qz": -records[1]["qz"]})
        samples = prepare_pose_samples(records)
        dot = (samples[0].qw * samples[1].qw + samples[0].qz * samples[1].qz)
        self.assertGreaterEqual(dot, 0.0)

    def test_deterministic_respects_limits(self):
        profile = ConstraintProfile(max_linear_speed=220, max_linear_accel=350,
                                    max_angular_speed=45, max_angular_accel=120)
        result = plan_speed_profile(sample_records(), profile, "deterministic")
        self.assertTrue(result.feasible)
        self.assertEqual(len(result.points), 8)
        self.assertLessEqual(max(p.linear_speed for p in result.points), 220)
        self.assertLessEqual(result.diagnostics["max_linear_accel"], 350 + 1e-6)
        self.assertLessEqual(result.diagnostics["max_angular_speed"], 45 + 1e-6)

    def test_double_q_is_seed_reproducible(self):
        profile = ConstraintProfile(training_episodes=500, speed_levels=9, random_seed=21)
        first = plan_speed_profile(sample_records(6), profile, "double_q")
        second = plan_speed_profile(sample_records(6), profile, "double_q")
        self.assertEqual([p.linear_speed for p in first.points],
                         [p.linear_speed for p in second.points])

    def test_minimum_speed_conflict_is_reported(self):
        records = sample_records(3)
        records[0].update({"x": 0.0, "y": 0.0, "z": 0.0})
        records[1].update({"x": 1.0, "y": 0.0, "z": 0.0})
        records[2].update({"x": 1.0, "y": 1.0, "z": 0.0})
        profile = ConstraintProfile(min_linear_speed=20.0, max_lateral_accel=1.0)
        result = plan_speed_profile(records, profile)
        self.assertFalse(result.feasible)
        self.assertIn("MIN_SPEED_CONFLICT", result.points[1].violation_codes)
        self.assertLessEqual(result.diagnostics["max_lateral_accel"], 1.0 + 1e-6)

    def test_invalid_profile_values_are_rejected(self):
        invalid_profiles = [
            ConstraintProfile(max_linear_speed=float("nan")),
            ConstraintProfile(start_speed=9999.0),
            ConstraintProfile(learning_rate=0.0),
            ConstraintProfile(epsilon_decay=float("inf")),
            ConstraintProfile(speed_levels=3.5),
            ConstraintProfile(accel_margin_factor=0.9),
        ]
        for profile in invalid_profiles:
            with self.subTest(profile=profile):
                with self.assertRaises(ValueError):
                    plan_speed_profile(sample_records(3), profile)

    def test_acceleration_margin_is_configurable(self):
        baseline = plan_speed_profile(
            sample_records(), ConstraintProfile(accel_margin_factor=1.0))
        guarded = plan_speed_profile(
            sample_records(), ConstraintProfile(accel_margin_factor=1.2))
        self.assertEqual(guarded.diagnostics["accel_margin_factor"], 1.2)
        for base_point, guarded_point in zip(baseline.points, guarded.points):
            self.assertGreaterEqual(guarded_point.linear_accel, base_point.linear_accel)
            self.assertGreaterEqual(guarded_point.angular_accel, base_point.angular_accel)

    def test_duplicate_points_are_rejected(self):
        records = sample_records(3)
        records[1].update({"x": records[0]["x"], "y": records[0]["y"], "z": records[0]["z"]})
        with self.assertRaises(ValueError):
            plan_speed_profile(records)

    def test_csv_contract(self):
        result = plan_speed_profile(sample_records(4))
        path = os.path.join(tempfile.gettempdir(), "speed_plan_contract.csv")
        self.assertEqual(write_speed_plan_csv(path, result), 4)
        with open(path, encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 4)
        self.assertIn("linear_speed", rows[0])
        self.assertIn("linear_accel", rows[0])
        self.assertIn("joint_speed", rows[0])
        self.assertIn("joint_accel", rows[0])
        self.assertIn("tcp_angular_accel_deg_s2", rows[0])
        self.assertIn("source_X", rows[0])
        self.assertIn("extrinsic_validated", rows[0])
        self.assertEqual(float(rows[0]["joint_speed"]), result.points[0].joint_speed)
        self.assertTrue(os.path.isfile(path + ".metadata.json"))

        test_dir = os.path.abspath(os.path.dirname(__file__))
        importer_candidates = [
            os.path.abspath(os.path.join(test_dir, "..", "..", "..", "Output")),
            os.path.abspath(os.path.join(test_dir, "..", "..", "Speed planning", "Output")),
        ]
        importer_dir = next((path for path in importer_candidates
                             if os.path.isfile(os.path.join(path, "pose_speed_importer.py"))), None)
        self.assertIsNotNone(importer_dir, "Cannot locate Speed planning Output importer")
        sys.path.insert(0, importer_dir)
        try:
            from pose_speed_importer import load_pose_speed_csv
            imported = load_pose_speed_csv(path)
        finally:
            sys.path.remove(importer_dir)
        self.assertEqual(len(imported), 4)
        self.assertEqual(imported[0].angular_speed, result.points[0].joint_speed)
        self.assertEqual(imported[0].angular_accel, result.points[0].joint_accel)


if __name__ == "__main__":
    unittest.main()
