"""Pure-compute adapter and independent numerical audit for Yang's local kernel.

The source paper bounds tool-axis deviation. This adapter additionally measures
full SO(3) deviation against the spatially corresponding original pose path.
Sampled checks are diagnostics, not a continuous mathematical certificate.
"""
import numpy as np
from .yang2020.bspline import BSpline as SourceBSpline
from pose_transform import quaternion_to_rotation, rotation_to_quaternion
from speed_planning_core import prepare_pose_samples
from .yang2020.full_pipeline import smooth
from .yang2020.orientation_decompose import compose, decompose
from .yang2020.synchronization_proto import build_sync_position_cps


class BSpline:
    def __init__(self, knots, points, degree):
        self.spline = SourceBSpline(np.asarray(points), np.asarray(knots), degree)

    def __call__(self, u, nu=0):
        if nu:
            return self.spline.evaluate_derivative(u, nu)
        return self.spline.evaluate(u)


def _spline(spec):
    return BSpline(spec['knots'], spec['control_points'], spec['degree'])


def _rotation_error(a, b):
    return float(np.degrees(np.arccos(np.clip((np.trace(a.T @ b)-1)/2, -1, 1))))


def _slerp(a, b, t):
    qa, qb = np.asarray(rotation_to_quaternion(a)), np.asarray(rotation_to_quaternion(b))
    if np.dot(qa, qb) < 0:
        qb = -qb
    theta = np.arccos(np.clip(np.dot(qa,qb),-1,1))
    q = ((1-t)*qa+t*qb) if theta < 1e-7 else (np.sin((1-t)*theta)*qa+np.sin(t*theta)*qb)/np.sin(theta)
    return quaternion_to_rotation(q / np.linalg.norm(q))


def smooth_and_audit(records, position_tolerance_mm=0.8,
                     orientation_tolerance_deg=1.0, samples_per_piece=33,
                     synchronized_orientation=True):
    if not np.isfinite([position_tolerance_mm, orientation_tolerance_deg]).all() or min(
            position_tolerance_mm, orientation_tolerance_deg) <= 0:
        raise ValueError('Tolerances must be positive and finite')
    if samples_per_piece < 5:
        raise ValueError('At least five samples per piece are required')
    source = prepare_pose_samples(records)
    xyz = np.array([[p.x, p.y, p.z] for p in source])
    rotations = np.array([quaternion_to_rotation([p.qw, p.qx, p.qy, p.qz]) for p in source])
    if np.any(np.linalg.norm(np.diff(xyz, axis=0), axis=1) < 1e-7):
        raise ValueError('Duplicate positions require a separate rotation-only planner')
    angles = np.array([decompose(r) for r in rotations])
    branch_crossings = int(np.count_nonzero(np.abs(np.diff(angles, axis=0)) > np.pi))
    angles = np.unwrap(angles, axis=0)
    if np.any(np.abs(np.cos(angles[:,1])) < .01):
        raise ValueError('ZYX near gimbal lock; use a different orientation representation')
    effective_position_tolerance = position_tolerance_mm
    if synchronized_orientation:
        angular_rate = np.linalg.norm(np.diff(angles, axis=0), axis=1)/np.linalg.norm(np.diff(xyz, axis=0), axis=1)
        # Conservative experiment setting, followed by full SO(3) audit below.
        # Geometry and orientation share identical B-spline basis coefficients.
        effective_position_tolerance = min(position_tolerance_mm,
            np.radians(orientation_tolerance_deg)/(4*max(float(max(angular_rate)),1e-12)))
    raw = smooth(xyz, rotations, effective_position_tolerance, np.radians(orientation_tolerance_deg),
                 continuous_angles=angles)
    if synchronized_orientation:
        def orientation_at_control(point, segments):
            choices = []
            for j in segments:
                v = xyz[j+1]-xyz[j]
                t = float(np.clip(np.dot(point-xyz[j],v)/np.dot(v,v),0,1))
                choices.append((float(np.linalg.norm(point-xyz[j]-t*v)),j,t))
            _,j,t = min(choices)
            return ((1-t)*angles[j]+t*angles[j+1]).tolist()
        for i,spec in enumerate(raw['position_corner_splines']):
            raw['orientation_corner_splines'][i]['control_points'] = [
                orientation_at_control(np.asarray(c),[i,i+1]) for c in spec['control_points']]
        for i,spec in enumerate(raw['sync_position_splines']):
            # Shared basis no longer needs the source's independent angular
            # length ratio, which degenerates for constant-orientation segments.
            start,end = np.asarray(spec['control_points'])[ [0,-1] ]
            mid = .5*(start+end)
            length = np.linalg.norm(end-start)/8
            spec['control_points'] = build_sync_position_cps(start,end,mid,length,length).tolist()
            raw['sync_orientation_splines'][i]['control_points'] = [
                orientation_at_control(np.asarray(c),[i+1]) for c in spec['control_points']]
    # Upstream field is a placeholder, not a measured result.
    raw.pop('continuity', None)
    pc = [_spline(s) for s in raw['position_corner_splines']]
    oc = [_spline(s) for s in raw['orientation_corner_splines']]
    ps = [_spline(s) for s in raw['sync_position_splines']]
    os = [_spline(s) for s in raw['sync_orientation_splines']]

    def line(a, b):
        return BSpline([0, 0, 1, 1], [a, b], 1)

    pieces = [(line(xyz[0], pc[0](0)), line(angles[0], oc[0](0)), [0])]
    for i, (p, o) in enumerate(zip(pc, oc)):
        pieces.append((p, o, [i, i+1]))
        if i < len(ps):
            pieces.append((ps[i], os[i], [i+1]))
    pieces.append((line(pc[-1](1), xyz[-1]), line(oc[-1](1), angles[-1]), [len(xyz)-2]))
    output, position_errors, orientation_errors, joins = [], [], [], []
    for piece_index, (p, o, reference_segments) in enumerate(pieces):
        if piece_index:
            previous_p, previous_o, _ = pieces[piece_index-1]
            tangent_left, tangent_right = previous_p(1, nu=1), p(0, nu=1)
            nl, nr = np.linalg.norm(tangent_left), np.linalg.norm(tangent_right)
            joins.append({'piece': piece_index,
                          'position_gap_mm': float(np.linalg.norm(previous_p(1)-p(0))),
                          'orientation_gap_deg': _rotation_error(compose(*previous_o(1)), compose(*o(0))),
                          'tangent_angle_deg': float(np.degrees(np.arccos(np.clip(np.dot(tangent_left,tangent_right)/max(nl*nr,1e-30),-1,1)))),
                          'orientation_dpsi_ds_gap': float(np.linalg.norm(previous_o(1,nu=1)/max(nl,1e-30)-o(0,nu=1)/max(nr,1e-30)))})
        for u in np.linspace(0, 1, samples_per_piece)[0 if piece_index == 0 else 1:]:
            point, rotation = p(u), compose(*o(u))
            choices = []
            for j in reference_segments:
                v = xyz[j+1]-xyz[j]
                fraction = float(np.clip(np.dot(point-xyz[j],v)/np.dot(v,v),0,1))
                choices.append((float(np.linalg.norm(point-xyz[j]-fraction*v)), j, fraction))
            distance, j, fraction = min(choices)
            reference = _slerp(rotations[j], rotations[j+1], fraction)
            angular_error = _rotation_error(reference, rotation)
            q = rotation_to_quaternion(rotation)
            output.append(dict(zip(['index','x','y','z','qw','qx','qy','qz'], [len(output)+1,*point,*q])))
            position_errors.append(distance)
            orientation_errors.append(angular_error)
    diagnostics = {'method':('yang2020_position_shared_orientation_basis' if synchronized_orientation else 'yang2020_local_reproduction'), 'sample_count':len(output),
                   'effective_position_tolerance_mm':effective_position_tolerance,
                   'unwrapped_euler_branch_crossings':branch_crossings,
                   'position_tolerance_mm': position_tolerance_mm,
                   'orientation_tolerance_deg':orientation_tolerance_deg,
                   'max_sampled_position_error_mm':max(position_errors),
                   'max_sampled_full_orientation_error_deg':max(orientation_errors),
                   'sampled_tolerances_pass': bool(max(position_errors)<=position_tolerance_mm+1e-7 and max(orientation_errors)<=orientation_tolerance_deg+1e-7),
                   'joins':joins, 'scope':'sampled geometric audit; no robot dynamics or execution certification'}
    return output, diagnostics, raw
