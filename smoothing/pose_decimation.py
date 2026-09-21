"""Reduce pose-polyline commands with explicit sampled position/SO(3) bounds."""
import numpy as np
from pose_transform import quaternion_to_rotation
from .yang_path import _slerp,_rotation_error


def decimate_poses(records, position_error_mm=.03, orientation_error_deg=.03):
    if min(position_error_mm,orientation_error_deg)<=0:
        raise ValueError('Decimation bounds must be positive')
    p=np.array([[r[k] for k in ['x','y','z']] for r in records])
    rotations=np.array([quaternion_to_rotation([r[k] for k in ['qw','qx','qy','qz']]) for r in records])
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    keep={0,len(records)-1}; stack=[(0,len(records)-1)]
    while stack:
        left,right=stack.pop()
        if right-left<2: continue
        worst=0.; selected=None
        for j in range(left+1,right):
            u=(s[j]-s[left])/max(s[right]-s[left],1e-12)
            pe=np.linalg.norm(p[j]-((1-u)*p[left]+u*p[right]))
            oe=_rotation_error(rotations[j],_slerp(rotations[left],rotations[right],u))
            score=max(pe/position_error_mm,oe/orientation_error_deg)
            if score>worst: worst=score; selected=j
        if worst>1:
            keep.add(selected); stack.extend([(left,selected),(selected,right)])
    indices=sorted(keep)
    maxp=maxo=0.
    for left,right in zip(indices[:-1],indices[1:]):
        for j in range(left,right+1):
            u=(s[j]-s[left])/max(s[right]-s[left],1e-12)
            maxp=max(maxp,float(np.linalg.norm(p[j]-(1-u)*p[left]-u*p[right])))
            maxo=max(maxo,_rotation_error(rotations[j],_slerp(rotations[left],rotations[right],u)))
    output=[dict(records[j],index=i+1) for i,j in enumerate(indices)]
    return output,{'input_count':len(records),'output_count':len(output),'retained_indices':indices,
                   'max_sampled_position_deviation_mm':maxp,'max_sampled_orientation_deviation_deg':maxo,
                   'position_budget_mm':position_error_mm,'orientation_budget_deg':orientation_error_deg,
                   'scope':'bounds checked against all dense reference samples; no continuous certificate'}
