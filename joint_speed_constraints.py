"""Deterministic retiming of a sampled joint path in mm/degree units.

These are configurable simulation bounds, not validated UR10 hardware ratings.
Half the acceleration budget is reserved for q_ss*v^2 and half for q_s*a.
Independent trajectory checks remain necessary after controller interpolation.
"""
import numpy as np


def retime_joint_path(path_s, joints_deg, tcp_caps, linear_accel,
                      joint_speed_deg_s=60., joint_accel_deg_s2=180., margin=.8):
    s=np.asarray(path_s,dtype=float)
    q=np.asarray(joints_deg,dtype=float)
    caps=np.asarray(tcp_caps,dtype=float).copy()
    vmax=np.broadcast_to(np.asarray(joint_speed_deg_s,dtype=float),(6,))
    amax=np.broadcast_to(np.asarray(joint_accel_deg_s2,dtype=float),(6,))
    if s.ndim!=1 or len(s)<3 or q.shape!=(len(s),6) or caps.shape!=s.shape:
        raise ValueError('Expected N path coordinates, N x 6 joints, N speed caps')
    if not all(np.isfinite(x).all() for x in [s,q,caps,vmax,amax]) or np.any(np.diff(s)<=0):
        raise ValueError('Path data must be finite with strictly increasing arc length')
    if min(vmax.min(),amax.min(),linear_accel,margin)<=0 or margin>1 or caps.min()<0:
        raise ValueError('Invalid simulation limits')
    qs=np.gradient(q,s,axis=0,edge_order=2)
    qss=np.gradient(qs,s,axis=0,edge_order=2)
    caps=np.minimum(caps,np.min(margin*vmax/np.maximum(abs(qs),1e-12),axis=1))
    caps=np.minimum(caps,np.sqrt(np.min(.5*margin*amax/np.maximum(abs(qss),1e-12),axis=1)))
    accel_caps=np.minimum(linear_accel,np.min(.5*margin*amax/np.maximum(abs(qs),1e-12),axis=1))
    v=caps.copy(); v[0]=v[-1]=0
    ds=np.diff(s)
    for i,distance in enumerate(ds):
        v[i+1]=min(v[i+1],np.sqrt(v[i]**2+2*min(accel_caps[i:i+2])*distance))
    for i in range(len(v)-2,-1,-1):
        v[i]=min(v[i],np.sqrt(v[i+1]**2+2*min(accel_caps[i:i+2])*ds[i]))
    a=np.gradient(v*v,s,edge_order=2)/2
    qd=qs*v[:,None]
    qdd=qss*v[:,None]**2+qs*a[:,None]
    ratio=max(1.,float(np.max(abs(qd)/vmax)),float(np.sqrt(np.max(abs(qdd)/amax))))
    if ratio>1:
        v*=.99/ratio
        a=np.gradient(v*v,s,edge_order=2)/2
        qd=qs*v[:,None]; qdd=qss*v[:,None]**2+qs*a[:,None]
    dt=2*ds/np.maximum(v[:-1]+v[1:],1e-12)
    return v, {'joint_speed_limits_deg_s':vmax.tolist(),'joint_accel_limits_deg_s2':amax.tolist(),
               'estimated_joint_speed_deg_s':np.max(abs(qd),axis=0).tolist(),
               'estimated_joint_accel_deg_s2':np.max(abs(qdd),axis=0).tolist(),
               'time_s':float(dt.sum()),'joint_path_deg':q.tolist(),
               'path_accel_caps_mm_s2':accel_caps.tolist(),
               'sampled_joint_constraints_pass':bool(np.all(abs(qd)<=vmax+1e-7) and np.all(abs(qdd)<=amax+1e-7)),
               'scope':'finite-difference sampled reference; requires executed trajectory verification'}
