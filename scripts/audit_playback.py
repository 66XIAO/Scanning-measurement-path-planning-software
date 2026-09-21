"""Audit observed simulator playback joints; keep this distinct from robot execution."""
import json
import sys
from pathlib import Path
import numpy as np


def audit(samples, speed_limit=60., accel_limit=180.):
    z=np.asarray(samples,dtype=float)
    if z.ndim!=2 or z.shape[1]!=7 or len(z)<5 or not np.isfinite(z).all():
        raise ValueError('Expected at least five finite timestamp + six-joint samples')
    t,q=z[:,0],z[:,1:]
    if np.any(np.diff(t)<=0): raise ValueError('Playback times must increase strictly')
    qd=np.gradient(q,t,axis=0,edge_order=2)
    qdd=np.gradient(qd,t,axis=0,edge_order=2)
    vmax=np.max(abs(qd),axis=0); amax=np.max(abs(qdd),axis=0)
    stationary=np.linalg.norm(np.diff(q,axis=0),axis=1)<1e-5
    runs=[]; duration=0.
    for i,flag in enumerate(stationary):
        if flag and t[i]>.25 and t[i+1]<t[-1]-.25:
            duration+=t[i+1]-t[i]
        elif duration:
            runs.append(duration); duration=0
    if duration: runs.append(duration)
    return {'samples':len(t),'sample_dt_min_s':float(np.diff(t).min()),
            'sample_dt_max_s':float(np.diff(t).max()),
            'peak_observed_joint_speed_deg_s':vmax.tolist(),
            'peak_observed_joint_accel_deg_s2':amax.tolist(),
            'speed_limit_deg_s':speed_limit,'acceleration_limit_deg_s2':accel_limit,
            'sampled_joint_limits_pass':bool(np.all(vmax<=speed_limit) and np.all(amax<=accel_limit)),
            'stationary_runs_over_120ms':int(sum(d>=.12 for d in runs)),
            'max_stationary_run_s':float(max(runs,default=0.)),
            'scope':'Nonuniform-time finite differences of observed simulator joints, without filtering. No physical robot or sub-sample guarantee.'}


def main():
    root=Path(sys.argv[1]); d=json.loads((root/'playback.json').read_text(encoding='utf-8'))
    result={key:{'wall_time_s':entry['wall_time_s'],**audit(entry['samples'])} for key,entry in d.items()}
    (root/'playback_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__': main()
