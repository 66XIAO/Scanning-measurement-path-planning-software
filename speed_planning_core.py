"""Constrained speed planning for ordered scanner poses.

The production baseline is a deterministic forward/backward time
parameterisation.  A layered Double-Q implementation is also provided as a
research mode matching the thesis' discrete state/action formulation.  Both
algorithms share the same feasibility envelope and output schema.

Units are explicit throughout: position mm, linear speed mm/s, linear
acceleration mm/s^2, angular speed deg/s and angular acceleration deg/s^2.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


EPS = 1e-9


@dataclass
class ConstraintProfile:
    min_linear_speed: float = 20.0
    max_linear_speed: float = 250.0
    max_linear_accel: float = 500.0
    max_lateral_accel: float = 300.0
    max_angular_speed: float = 60.0
    max_angular_accel: float = 180.0
    command_joint_speed: float = 60.0
    command_joint_accel: float = 180.0
    accel_margin_factor: float = 1.10
    start_speed: float = 20.0
    end_speed: float = 20.0
    safety_factor: float = 0.90
    speed_levels: int = 15
    training_episodes: int = 5000
    learning_rate: float = 0.20
    discount_factor: float = 1.0
    epsilon_start: float = 1.0
    epsilon_end: float = 0.02
    epsilon_decay: float = 0.999
    random_seed: int = 7

    def validate(self):
        finite_positive = (
            "min_linear_speed",
            "max_linear_speed",
            "max_linear_accel",
            "max_lateral_accel",
            "max_angular_speed",
            "max_angular_accel",
            "command_joint_speed",
            "command_joint_accel",
            "start_speed",
            "end_speed",
        )
        for name in finite_positive:
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError("{} must be a positive finite value".format(name))
        if self.min_linear_speed <= 0 or self.min_linear_speed > self.max_linear_speed:
            raise ValueError("min_linear_speed must be > 0 and <= max_linear_speed")
        if not 0 < self.safety_factor <= 1:
            raise ValueError("safety_factor must be in (0, 1]")
        if not math.isfinite(self.accel_margin_factor) or not 1 <= self.accel_margin_factor <= 2:
            raise ValueError("accel_margin_factor must be in [1, 2]")
        if not self.min_linear_speed <= self.start_speed <= self.max_linear_speed:
            raise ValueError("start_speed must be within linear speed limits")
        if not self.min_linear_speed <= self.end_speed <= self.max_linear_speed:
            raise ValueError("end_speed must be within linear speed limits")
        if not isinstance(self.speed_levels, int) or self.speed_levels < 3:
            raise ValueError("speed_levels must be at least 3")
        if not isinstance(self.training_episodes, int) or self.training_episodes < 1:
            raise ValueError("training_episodes must be positive")
        if not math.isfinite(self.learning_rate) or not 0 < self.learning_rate <= 1:
            raise ValueError("learning_rate must be in (0, 1]")
        if not math.isfinite(self.discount_factor) or not 0 <= self.discount_factor <= 1:
            raise ValueError("discount_factor must be in [0, 1]")
        for name in ("epsilon_start", "epsilon_end"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError("{} must be in [0, 1]".format(name))
        if self.epsilon_end > self.epsilon_start:
            raise ValueError("epsilon_end must not exceed epsilon_start")
        if not math.isfinite(self.epsilon_decay) or not 0 < self.epsilon_decay <= 1:
            raise ValueError("epsilon_decay must be in (0, 1]")
        if not isinstance(self.random_seed, int):
            raise ValueError("random_seed must be an integer")


@dataclass(frozen=True)
class PoseSample:
    index: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass(frozen=True)
class SpeedPlanPoint:
    index: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    path_s: float
    segment_length: float
    curvature: float
    orientation_delta_deg: float
    linear_speed: float
    linear_accel: float
    angular_speed: float
    angular_accel: float
    joint_speed: float
    joint_accel: float
    dt_to_next: float
    max_linear_speed: float
    feasible: bool
    violation_codes: str = ""


@dataclass
class SpeedPlanResult:
    algorithm: str
    points: List[SpeedPlanPoint]
    total_time: float
    feasible: bool
    warnings: List[str] = field(default_factory=list)
    diagnostics: Dict[str, object] = field(default_factory=dict)


def _normalise_quaternion(values: Sequence[float]) -> np.ndarray:
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must contain 4 finite values")
    norm = float(np.linalg.norm(q))
    if norm < EPS:
        raise ValueError("Quaternion norm is zero")
    return q / norm


def prepare_pose_samples(records: Iterable[object]) -> List[PoseSample]:
    """Convert dictionaries/tuples/PoseSample values to validated samples."""
    samples = []
    previous_q = None
    for fallback_index, record in enumerate(records):
        if isinstance(record, PoseSample):
            index = record.index
            xyz = (record.x, record.y, record.z)
            raw_q = (record.qw, record.qx, record.qy, record.qz)
        elif isinstance(record, dict):
            index = int(record.get("index", fallback_index))
            xyz = (record["x"], record["y"], record["z"])
            raw_q = (record["qw"], record["qx"], record["qy"], record["qz"])
        else:
            values = list(record)
            if len(values) != 8:
                raise ValueError("Pose tuple must be index,x,y,z,qw,qx,qy,qz")
            index = int(values[0])
            xyz = values[1:4]
            raw_q = values[4:8]
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("Pose {} position is invalid".format(index))
        q = _normalise_quaternion(raw_q)
        if previous_q is not None and float(np.dot(previous_q, q)) < 0:
            q = -q
        previous_q = q
        samples.append(PoseSample(index, float(xyz[0]), float(xyz[1]), float(xyz[2]),
                                  float(q[0]), float(q[1]), float(q[2]), float(q[3])))
    if len(samples) < 2:
        raise ValueError("Speed planning requires at least 2 ordered poses")
    return samples


def _path_geometry(samples: Sequence[PoseSample]):
    xyz = np.asarray([[p.x, p.y, p.z] for p in samples], dtype=float)
    quats = np.asarray([[p.qw, p.qx, p.qy, p.qz] for p in samples], dtype=float)
    delta = np.diff(xyz, axis=0)
    ds = np.linalg.norm(delta, axis=1)
    if np.any(ds <= EPS):
        bad = np.where(ds <= EPS)[0].tolist()
        raise ValueError("Adjacent duplicate path points at segments {}".format(bad))
    path_s = np.concatenate(([0.0], np.cumsum(ds)))

    turn_angle = np.zeros(len(samples), dtype=float)
    curvature = np.zeros(len(samples), dtype=float)
    for i in range(1, len(samples) - 1):
        u = delta[i - 1] / ds[i - 1]
        v = delta[i] / ds[i]
        angle = math.acos(float(np.clip(np.dot(u, v), -1.0, 1.0)))
        turn_angle[i] = angle
        curvature[i] = angle / max(0.5 * (ds[i - 1] + ds[i]), EPS)

    orientation_delta = np.zeros(len(samples), dtype=float)
    for i in range(len(samples) - 1):
        dot = abs(float(np.dot(quats[i], quats[i + 1])))
        orientation_delta[i] = 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))
    return xyz, quats, ds, path_s, curvature, orientation_delta


def _speed_caps(ds, curvature, orientation_delta, profile):
    n = len(curvature)
    caps = np.full(n, profile.max_linear_speed * profile.safety_factor, dtype=float)
    for i, kappa in enumerate(curvature):
        if kappa > EPS:
            caps[i] = min(caps[i], math.sqrt(profile.max_lateral_accel / kappa))
    omega_limit = math.radians(profile.max_angular_speed) * profile.safety_factor
    for i, theta in enumerate(orientation_delta[:-1]):
        if theta > EPS:
            segment_cap = omega_limit * ds[i] / theta
            caps[i] = min(caps[i], segment_cap)
            caps[i + 1] = min(caps[i + 1], segment_cap)
    # Preserve physical caps even when they fall below the scanner minimum.
    # Such points are infeasible and must be reported, not silently raised.
    caps[0] = min(caps[0], profile.start_speed)
    caps[-1] = min(caps[-1], profile.end_speed)
    return caps


def _forward_backward(caps, ds, profile):
    speeds = np.array(caps, dtype=float)
    speeds[0] = min(speeds[0], profile.start_speed)
    for i, distance in enumerate(ds):
        reachable = math.sqrt(max(0.0, speeds[i] ** 2 + 2.0 * profile.max_linear_accel * distance))
        speeds[i + 1] = min(speeds[i + 1], reachable)
    speeds[-1] = min(speeds[-1], profile.end_speed)
    for i in range(len(ds) - 1, -1, -1):
        reachable = math.sqrt(max(0.0, speeds[i + 1] ** 2 + 2.0 * profile.max_linear_accel * ds[i]))
        speeds[i] = min(speeds[i], reachable)
    return np.maximum(speeds, EPS)


def _segment_dynamics(speeds, ds, orientation_delta):
    dt = 2.0 * ds / np.maximum(speeds[:-1] + speeds[1:], EPS)
    linear_accel = np.abs(speeds[1:] ** 2 - speeds[:-1] ** 2) / (2.0 * ds)
    omega = orientation_delta[:-1] / np.maximum(dt, EPS)
    angular_accel = np.zeros_like(omega)
    if len(omega) > 1:
        avg_dt = 0.5 * (dt[:-1] + dt[1:])
        angular_accel[1:] = np.abs(np.diff(omega)) / np.maximum(avg_dt, EPS)
    return dt, linear_accel, omega, angular_accel


def _enforce_angular_acceleration(speeds, caps, ds, orientation_delta, profile):
    speeds = np.array(speeds, dtype=float)
    limit = math.radians(profile.max_angular_accel) * profile.safety_factor
    for _ in range(12):
        _dt, _acc, _omega, alpha = _segment_dynamics(speeds, ds, orientation_delta)
        violating = np.where(alpha > limit + 1e-8)[0]
        if not len(violating):
            break
        for seg in violating:
            ratio = math.sqrt(max(limit, EPS) / max(alpha[seg], EPS))
            for point_index in (seg, seg + 1):
                caps[point_index] = max(EPS, min(caps[point_index], speeds[point_index] * ratio))
        speeds = _forward_backward(caps, ds, profile)
    return speeds


def _transition_feasible(v1, v2, distance, cap2, profile):
    if v2 > cap2 + 1e-7:
        return False
    required = abs(v2 * v2 - v1 * v1) / max(2.0 * distance, EPS)
    return required <= profile.max_linear_accel + 1e-7


def _double_q_speeds(caps, ds, profile):
    rng = np.random.default_rng(profile.random_seed)
    levels = np.linspace(profile.min_linear_speed, profile.max_linear_speed, profile.speed_levels)
    k_count = len(levels)
    n = len(caps)
    q1 = np.full((n - 1, k_count, k_count), 1000.0, dtype=float)
    q2 = np.full_like(q1, 1000.0)
    start_idx = int(np.argmin(np.abs(levels - caps[0])))
    epsilon = profile.epsilon_start

    def actions_for(segment, current_idx):
        result = [a for a, speed in enumerate(levels)
                  if _transition_feasible(levels[current_idx], speed, ds[segment], caps[segment + 1], profile)]
        return result or [int(np.argmin(np.abs(levels - min(caps[segment + 1], levels[0]))))]

    for _episode in range(profile.training_episodes):
        current = start_idx
        for segment in range(n - 1):
            allowed = actions_for(segment, current)
            if rng.random() < epsilon:
                action = int(rng.choice(allowed))
            else:
                combined = q1[segment, current, allowed] + q2[segment, current, allowed]
                action = allowed[int(np.argmin(combined))]
            v1, v2 = levels[current], levels[action]
            cost = 2.0 * ds[segment] / max(v1 + v2, EPS)
            if not _transition_feasible(v1, v2, ds[segment], caps[segment + 1], profile):
                cost += 1e6

            update_q1 = rng.random() < 0.5
            if segment == n - 2:
                target = cost
            else:
                next_allowed = actions_for(segment + 1, action)
                if update_q1:
                    best = next_allowed[int(np.argmin(q1[segment + 1, action, next_allowed]))]
                    target = cost + profile.discount_factor * q2[segment + 1, action, best]
                else:
                    best = next_allowed[int(np.argmin(q2[segment + 1, action, next_allowed]))]
                    target = cost + profile.discount_factor * q1[segment + 1, action, best]

            if update_q1:
                q1[segment, current, action] += profile.learning_rate * (
                    target - q1[segment, current, action])
            else:
                q2[segment, current, action] += profile.learning_rate * (
                    target - q2[segment, current, action])
            current = action
        epsilon = max(profile.epsilon_end, epsilon * profile.epsilon_decay)

    indices = [start_idx]
    current = start_idx
    for segment in range(n - 1):
        allowed = actions_for(segment, current)
        combined = q1[segment, current, allowed] + q2[segment, current, allowed]
        current = allowed[int(np.argmin(combined))]
        indices.append(current)
    speeds = np.minimum(levels[np.asarray(indices, dtype=int)], caps)
    speeds = _forward_backward(np.minimum(caps, speeds), ds, profile)
    return speeds, {"final_epsilon": epsilon, "q_shape": list(q1.shape), "levels": levels.tolist()}


def _build_result(algorithm, samples, ds, path_s, curvature, orientation_delta,
                  caps, speeds, profile, diagnostics=None):
    if not (np.all(np.isfinite(caps)) and np.all(np.isfinite(speeds))):
        raise ValueError("Speed planner produced non-finite values")
    dt, linear_accel_seg, omega_seg, angular_accel_seg = _segment_dynamics(
        speeds, ds, orientation_delta)
    n = len(samples)
    linear_accel = np.zeros(n)
    angular_speed = np.zeros(n)
    angular_accel = np.zeros(n)
    linear_accel[:-1] = np.minimum(profile.max_linear_accel,
                                   np.maximum(1.0, linear_accel_seg * profile.accel_margin_factor))
    linear_accel[-1] = linear_accel[-2]
    angular_speed[:-1] = np.degrees(omega_seg)
    angular_speed[-1] = angular_speed[-2]
    angular_accel[:-1] = np.minimum(profile.max_angular_accel,
                                    np.maximum(1.0, np.degrees(angular_accel_seg) *
                                               profile.accel_margin_factor))
    angular_accel[-1] = angular_accel[-2]

    points = []
    feasible = True
    for i, sample in enumerate(samples):
        violations = []
        if speeds[i] < profile.min_linear_speed - 1e-6:
            violations.append("MIN_SPEED_CONFLICT")
        if speeds[i] > caps[i] + 1e-6:
            violations.append("SPEED_CAP")
        if speeds[i] * speeds[i] * curvature[i] > profile.max_lateral_accel + 1e-6:
            violations.append("LATERAL_ACCEL")
        if i < n - 1 and linear_accel_seg[i] > profile.max_linear_accel + 1e-6:
            violations.append("LINEAR_ACCEL")
        if i < n - 1 and math.degrees(omega_seg[i]) > profile.max_angular_speed + 1e-6:
            violations.append("ANGULAR_SPEED")
        if i < n - 1 and math.degrees(angular_accel_seg[i]) > profile.max_angular_accel + 1e-6:
            violations.append("ANGULAR_ACCEL")
        feasible = feasible and not violations
        points.append(SpeedPlanPoint(
            index=sample.index,
            x=sample.x, y=sample.y, z=sample.z,
            qw=sample.qw, qx=sample.qx, qy=sample.qy, qz=sample.qz,
            path_s=float(path_s[i]),
            segment_length=float(ds[i]) if i < n - 1 else 0.0,
            curvature=float(curvature[i]),
            orientation_delta_deg=float(math.degrees(orientation_delta[i])),
            linear_speed=float(speeds[i]),
            linear_accel=float(linear_accel[i]),
            angular_speed=float(angular_speed[i]),
            angular_accel=float(angular_accel[i]),
            joint_speed=float(profile.command_joint_speed),
            joint_accel=float(profile.command_joint_accel),
            dt_to_next=float(dt[i]) if i < n - 1 else 0.0,
            max_linear_speed=float(caps[i]),
            feasible=not violations,
            violation_codes=";".join(violations),
        ))
    warnings = [
        "Joint velocity, joint acceleration and torque limits are not yet validated; "
        "run RoboDK/robot validation before production execution.",
        "Input pose is treated as commanded tool pose. Configure scanner-to-tool extrinsics explicitly.",
    ]
    diag = dict(diagnostics or {})
    diag.update({
        "max_linear_speed": float(np.max(speeds)),
        "max_linear_accel": float(np.max(linear_accel_seg)),
        "max_angular_speed": float(np.max(np.degrees(omega_seg))),
        "max_angular_accel": float(np.max(np.degrees(angular_accel_seg))),
        "max_lateral_accel": float(np.max(speeds * speeds * curvature)),
        "dynamic_constraints_validated": False,
        "accel_margin_factor": float(profile.accel_margin_factor),
    })
    return SpeedPlanResult(algorithm, points, float(np.sum(dt)), feasible, warnings, diag)


def plan_speed_profile(records: Iterable[object], profile: Optional[ConstraintProfile] = None,
                       algorithm: str = "deterministic") -> SpeedPlanResult:
    """Plan one linear/angular speed and acceleration tuple per ordered pose."""
    profile = profile or ConstraintProfile()
    profile.validate()
    samples = prepare_pose_samples(records)
    _xyz, _quats, ds, path_s, curvature, orientation_delta = _path_geometry(samples)
    caps = _speed_caps(ds, curvature, orientation_delta, profile)
    diagnostics = {}
    if algorithm == "deterministic":
        speeds = _forward_backward(caps.copy(), ds, profile)
    elif algorithm == "double_q":
        speeds, diagnostics = _double_q_speeds(caps.copy(), ds, profile)
    else:
        raise ValueError("Unknown speed planning algorithm: {}".format(algorithm))
    speeds = _enforce_angular_acceleration(speeds, caps, ds, orientation_delta, profile)
    return _build_result(algorithm, samples, ds, path_s, curvature, orientation_delta,
                         caps, speeds, profile, diagnostics)


def result_rows(result: SpeedPlanResult) -> List[Dict[str, object]]:
    return [dict(vars(point), algorithm=result.algorithm, total_time=result.total_time,
                 plan_feasible=result.feasible) for point in result.points]
