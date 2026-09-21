"""Frame-consistent TCP angular diagnostics for a sampled pose/time trajectory."""
import numpy as np
from pose_transform import quaternion_to_rotation, rotation_to_quaternion


def angular_vector_diagnostics(points):
    rotations=[quaternion_to_rotation([p.qw,p.qx,p.qy,p.qz]) for p in points]
    dt=np.array([p.dt_to_next for p in points[:-1]],dtype=float)
    if len(dt)<1 or np.any(dt<=0) or not np.isfinite(dt).all():
        raise ValueError('Positive finite segment times required')
    vectors=[]
    for left,right in zip(rotations[:-1],rotations[1:]):
        q=np.array(rotation_to_quaternion(right@left.T))
        if q[0]<0: q=-q
        n=np.linalg.norm(q[1:])
        vectors.append(np.zeros(3) if n<1e-12 else 2*np.arctan2(n,q[0])*q[1:]/n)
    omega=np.degrees(np.asarray(vectors))/dt[:,None]
    alpha=np.diff(omega,axis=0)/(.5*(dt[1:]+dt[:-1]))[:,None]
    return {'max_tcp_angular_vector_speed_deg_s':float(np.max(np.linalg.norm(omega,axis=1))),
            'max_tcp_angular_vector_accel_deg_s2':float(np.max(np.linalg.norm(alpha,axis=1))) if len(alpha) else 0.,
            'angular_vector_frame':'fixed command reference frame',
            'angular_vector_scope':'segment-average rotation vectors and central differences; not a continuous bound'}
