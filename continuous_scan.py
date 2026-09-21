"""Continuous scan reference plan and explicit incoming-move command adapter.

The reference speed profile is not a controller execution trace. All command
programs require RoboDK trajectory/export validation before acceptance.
"""
from dataclasses import asdict, dataclass, replace
import math
import numpy as np
from smoothing.yang_path import smooth_and_audit
from speed_planning_core import (ConstraintProfile, plan_speed_profile,
    prepare_pose_samples, _path_geometry, _speed_caps, _build_result)
from trajectory_kinematics import angular_vector_diagnostics


@dataclass(frozen=True)
class ScanCommand:
    index: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    linear_speed: float
    linear_accel: float
    joint_speed: float
    joint_accel: float
    rounding_mm: float
    incoming_segment: int | None


def build_commands(result, profile, rounding_mm=0.05, fixed_speed=None):
    if not math.isfinite(rounding_mm) or rounding_mm < 0:
        raise ValueError('rounding_mm must be finite and nonnegative')
    if fixed_speed is not None and (not math.isfinite(fixed_speed) or fixed_speed <= 0):
        raise ValueError('fixed_speed must be positive')
    points = result.points
    xyz = np.array([[p.x,p.y,p.z] for p in points])
    ds = np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    commands=[]
    for i,p in enumerate(points):
        # First target is a separate positioning move, excluded from scan timing.
        # A MoveL speed is a segment ceiling, not a destination velocity.
        # Taking the maximum endpoint speed lets a decelerating move enter its
        # constrained destination too fast. Use the lower positive reference
        # speed; endpoint rest is implemented by exact-stop rounding instead.
        speed = (profile.max_linear_speed*profile.safety_factor if i == 0 else
                 min(value for value in (points[i-1].linear_speed,p.linear_speed) if value>0))
        if fixed_speed is not None:
            speed = fixed_speed
        if speed <= 0:
            raise ValueError('Zero-speed incoming segment cannot be commanded')
        radius = 0.0 if i in (0,len(points)-1) else min(rounding_mm,.4*ds[i-1],.4*ds[i])
        accel=profile.max_linear_accel
        joint_audit=result.diagnostics.get('joint_constraint_audit')
        if joint_audit and i:
            accel=min(joint_audit['path_accel_caps_mm_s2'][i-1:i+1])
        # The installed legacy API sends rounding as integer micrometres.
        radius=math.floor(radius*1000)/1000
        if 0<i<len(points)-1 and rounding_mm>0 and radius<=0:
            raise ValueError('Sampling too dense for nonzero legacy API rounding')
        commands.append(ScanCommand(p.index,p.x,p.y,p.z,p.qw,p.qx,p.qy,p.qz,
                                    speed,accel,
                                    profile.command_joint_speed,profile.command_joint_accel,
                                    radius,None if i == 0 else i-1))
    return commands


def plan_continuous_scan(records, profile=None, position_tolerance_mm=1.5,
                         orientation_tolerance_deg=2.0, rounding_mm=.3,
                         samples_per_piece=17):
    profile = replace(profile or ConstraintProfile(min_linear_speed=.1), start_speed=0.,end_speed=0.)
    poses,audit,_ = smooth_and_audit(records,position_tolerance_mm,
                                    orientation_tolerance_deg,samples_per_piece)
    if not audit['sampled_tolerances_pass']:
        raise ValueError('Smoothing violates sampled position/orientation tolerances: {}'.format(audit))
    from smoothing.pose_decimation import decimate_poses
    dense_poses=poses
    poses,decimation=decimate_poses(poses,
        min(.03,(position_tolerance_mm-audit['max_sampled_position_error_mm'])/2),
        min(.03,(orientation_tolerance_deg-audit['max_sampled_full_orientation_error_deg'])/2))
    audit['decimation']=decimation
    if len(poses)==2:
        indices=[0,len(dense_poses)//2,len(dense_poses)-1]
        poses=[dict(dense_poses[j],index=i+1) for i,j in enumerate(indices)]
        decimation.update(output_count=3,retained_indices=indices)
    result=plan_speed_profile(poses,profile,'deterministic')
    # The legacy local angular-acceleration repair has a finite iteration cap.
    # A uniform time stretch scales all accelerations quadratically and preserves
    # both the path and endpoint rest states; remeasure rather than hide violations.
    angular=angular_vector_diagnostics(result.points)
    ratio=max(1., angular['max_tcp_angular_vector_accel_deg_s2']/profile.max_angular_accel,
              result.diagnostics['max_angular_accel']/profile.max_angular_accel,
              result.diagnostics['max_linear_accel']/profile.max_linear_accel)
    if ratio > 1:
        samples=prepare_pose_samples(poses)
        _,_,ds,path_s,curvature,theta=_path_geometry(samples)
        caps=_speed_caps(ds,curvature,theta,profile)
        scale=.99/math.sqrt(ratio)
        result=_build_result('deterministic',samples,ds,path_s,curvature,theta,caps,
                             np.array([p.linear_speed for p in result.points])*scale,profile,
                             {'uniform_speed_scale':scale})
    result.diagnostics.update(angular_vector_diagnostics(result.points))
    result.algorithm='continuous_yang_deterministic'
    result.diagnostics.update({'smoothing_audit':audit,'constraint_profile':asdict(profile),
                               'command_semantics':'incoming_segment',
                               'continuous_execution_validated':False,
                               'original_pose_count':len(records)})
    result.diagnostics['incoming_commands']=[asdict(c) for c in build_commands(result,profile,rounding_mm)]
    result.warnings.append('Continuous execution, blending deviation and joint constraints require RoboDK trajectory verification.')
    return result


def constrain_with_robodk_joints(result):
    """Read-only continuous IK/FK, followed by sampled joint-aware retiming."""
    import robodk_bridge as b
    from joint_speed_constraints import retime_joint_path
    records=[{k:getattr(p,k) for k in ('index','x','y','z','qw','qx','qy','qz')} for p in result.points]
    report=b.analyze_ur10_reachability(records,result.diagnostics)
    if not report['all_reachable']:
        raise ValueError('Unreachable smoothed poses: {}'.format(report['unreachable_indices']))
    q=[r['selected_joints_deg'] for r in report['points']]
    if np.max(np.abs(np.diff(q,axis=0)))>45:
        raise ValueError('IK branch discontinuity exceeds 45 degrees between samples')
    profile=ConstraintProfile(**result.diagnostics['constraint_profile'])
    v,audit=retime_joint_path([p.path_s for p in result.points],q,
        [p.linear_speed for p in result.points],profile.max_linear_accel,
        profile.command_joint_speed,profile.command_joint_accel)
    samples=prepare_pose_samples(records)
    _,_,ds,s,k,theta=_path_geometry(samples)
    caps=_speed_caps(ds,k,theta,profile)
    revised=_build_result(result.algorithm,samples,ds,s,k,theta,caps,v,profile)
    angular=angular_vector_diagnostics(revised.points)
    angular_ratio=angular['max_tcp_angular_vector_accel_deg_s2']/profile.max_angular_accel
    if angular_ratio>1:
        stretch=.99/math.sqrt(angular_ratio)
        revised=_build_result(result.algorithm,samples,ds,s,k,theta,caps,v*stretch,profile)
        audit['pre_tcp_stretch_time_s']=audit['time_s']
        audit['time_s']=revised.total_time
        audit['estimated_joint_speed_deg_s']=[x*stretch for x in audit['estimated_joint_speed_deg_s']]
        audit['estimated_joint_accel_deg_s2']=[x*stretch**2 for x in audit['estimated_joint_accel_deg_s2']]
        audit['tcp_angular_speed_scale']=stretch
    revised.diagnostics={**result.diagnostics,**revised.diagnostics}
    revised.diagnostics.update(angular_vector_diagnostics(revised.points))
    revised.diagnostics['joint_constraint_audit']=audit
    revised.diagnostics['joint_ik_report']=report
    rounding=max(c['rounding_mm'] for c in result.diagnostics['incoming_commands'])
    revised.diagnostics['incoming_commands']=[asdict(c) for c in build_commands(revised,profile,rounding)]
    return revised


def import_continuous_scan(result, robot_name='UR10', frame_name='Frame 2',
                           tool_name='Creaform MetraSCAN', program_name='ContinuousScanV1',
                           target_namespace=None, first_move='movej',replace=False):
    import robodk_bridge as b
    if not result.feasible or not result.diagnostics.get('extrinsic_validated'):
        raise ValueError('A feasible plan and verified station mapping are required')
    if first_move != 'movej':
        raise ValueError('Continuous scan uses a separate MoveJ approach to its first point')
    commands=[ScanCommand(**r) for r in result.diagnostics['incoming_commands']]
    if len(commands)!=len(result.points):
        raise ValueError('Command/trajectory length mismatch')
    api=b._import_api(); rdk=api['Robolink']()
    robot=b._require_item(rdk,robot_name,api['ITEM_TYPE_ROBOT'],'robot')
    frame=b._require_item(rdk,frame_name,api['ITEM_TYPE_FRAME'],'frame')
    tool=b._require_item(rdk,tool_name,api['ITEM_TYPE_TOOL'],'tool')
    verification=b._verify_tool_mapping(rdk,robot,frame,tool,result.diagnostics,robot_name,frame_name,tool_name)

    def configure(program,point,i,target,first_move):
        if result.diagnostics.get('joint_constraint_audit'):
            target.setJoints(result.diagnostics['joint_constraint_audit']['joint_path_deg'][i])
        program.setSpeed(point.linear_speed,point.joint_speed,point.linear_accel,point.joint_accel)
        program.setRounding(point.rounding_mm)
        b._add_move(program,target,i,first_move)

    def validate(program,points):
        instructions=[program.Instruction(i) for i in range(program.InstructionCount())]
        if len(instructions)!=2+3*len(points):
            raise RuntimeError('Unexpected continuous program instruction count')
        for i in range(len(points)):
            triple=instructions[2+3*i:5+3*i]
            if [x[1] for x in triple] != [2,10,0]:
                raise RuntimeError('Expected Set Speed / Rounding / Move sequence, got {}'.format([x[1] for x in triple]))
        return len(instructions)

    publication=b._transactional_program_import(rdk,api,robot,frame,tool,commands,program_name,
        target_namespace or program_name+'_',first_move,replace,configure,validate)
    return {'program':program_name,'pose_count':len(commands),
            'instruction_count':publication['validation_result'],
            'tool_mapping_verification':verification,'semantics':'incoming_segment'}
