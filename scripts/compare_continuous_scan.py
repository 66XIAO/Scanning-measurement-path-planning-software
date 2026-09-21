"""Reproducible three-arm RoboDK experiment; no hardware run/upload calls."""
import json
import sys
from dataclasses import asdict
from pathlib import Path
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import robodk_bridge as bridge
from continuous_scan import plan_continuous_scan,build_commands,import_continuous_scan
from continuous_export import export_continuous_scan
from speed_planning_core import ConstraintProfile,plan_speed_profile
from pose_transform import load_extrinsic_config,transform_pose_records


def trajectory_metrics(rows):
    r=np.asarray(rows,dtype=float)
    if r.ndim!=2 or r.shape[0]!=26 or not np.all(np.isfinite(r)):
        raise ValueError('Expected finite 26-row UR10 timing/velocity/acceleration trace')
    dt=r[10]; mask=(dt>1e-9)&(r[9]>1)
    distance=np.r_[0.,np.linalg.norm(np.diff(r[11:14],axis=1),axis=0)]
    speed=np.divide(distance,dt,out=np.zeros_like(dt),where=dt>1e-9)
    qd=np.diff(r[:6],axis=1)/np.maximum(dt[1:],1e-9)
    qdd=np.diff(qd,axis=1)/np.maximum((dt[2:]+dt[1:-1])/2,1e-9)
    inside=mask.copy(); inside[:2]=False; inside[-2:]=False
    return {'trace_total_time_s':float(dt.sum()),'trace_scan_time_s':float(dt[mask].sum()),
            'error_codes':sorted(set(r[6].astype(int).tolist())),
            'minimum_interior_sample_speed_mm_s':float(speed[inside].min()),
            'near_zero_interior_samples':int(np.count_nonzero(speed[inside]<.01)),
            'peak_tcp_speed_mm_s':float(speed.max()),
            'joint_speed_finite_difference_deg_s':np.max(abs(qd),axis=1).tolist(),
            'joint_accel_finite_difference_deg_s2':np.max(abs(qdd),axis=1).tolist(),
            'joint_speed_api_deg_s':np.max(abs(r[14:20]),axis=1).tolist(),
            'joint_accel_api_deg_s2':np.max(abs(r[20:26]),axis=1).tolist(),
            'interpretation':'time row treated as dt; TCP speed uses XYZ differences, not legacy distance field; finite differences include instruction boundaries; sampling trace is not wall-clock playback'}


def main():
    case=Path('artifacts/speed_v1/capture_20260917_03')
    source=json.loads((case/'source_poses.json').read_text(encoding='utf-8'))
    _,metadata=transform_pose_records(source,load_extrinsic_config(str(case/'station_mapping.json')))
    profile=ConstraintProfile(min_linear_speed=.1)
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    output=Path('artifacts/speed_v1')/('comparison_'+stamp); output.mkdir()
    api=bridge._import_api(); rdk=api['Robolink']()
    results={}
    for arm in ['A_stop','B_smooth_fixed','C_smooth_variable']:
        if arm=='A_stop':
            result=plan_speed_profile(source,profile)
            commands=build_commands(result,profile,rounding_mm=0,fixed_speed=150)
        else:
            result=plan_continuous_scan(source,profile)
            commands=build_commands(result,profile,rounding_mm=.05,
                                    fixed_speed=150 if arm=='B_smooth_fixed' else None)
        result.diagnostics.update(metadata)
        result.diagnostics['command_semantics']='incoming_segment'
        result.diagnostics['incoming_commands']=[asdict(c) for c in commands]
        name='V1_'+arm+'_'+stamp
        print(arm+': importing '+name,flush=True)
        summary=import_continuous_scan(result,program_name=name)
        program=rdk.Item(name,api['ITEM_TYPE_PROGRAM'])
        print(arm+': updating',flush=True)
        update=program.Update(timeout_sec=60)
        print(arm+': reading trajectory',flush=True)
        msg,matrix,status=program.InstructionListJoints(mm_step=2,deg_step=1,flags=4,time_step=.01)
        (output/(arm+'_trajectory.json')).write_text(json.dumps({'message':msg,'status':status,'rows':matrix.rows}),encoding='utf-8')
        print(arm+': exporting',flush=True)
        export=export_continuous_scan(name,result.diagnostics['incoming_commands'],output/arm)
        summary.update({'update':update,'trajectory':trajectory_metrics(matrix.rows),'export':export})
        results[arm]=summary
        (output/'comparison.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'arm':arm,'output':str(output),'metrics':summary['trajectory']},ensure_ascii=True),flush=True)


if __name__=='__main__':
    main()
