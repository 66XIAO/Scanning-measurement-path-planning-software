"""Small offline collinear probe to diagnose RoboDK speed/blending timing."""
import json
import sys
from pathlib import Path
from datetime import datetime
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import robodk_bridge as b
from pose_transform import load_extrinsic_config,transform_pose_records


def main():
    case=Path('artifacts/speed_v1/capture_20260917_03')
    records=json.loads((case/'source_poses.json').read_text(encoding='utf-8'))
    _,metadata=transform_pose_records(records,load_extrinsic_config(str(case/'station_mapping.json')))
    a=b._import_api(); r=a['Robolink']()
    robot=r.Item('UR10',2); frame=r.Item('Frame 2',3); tool=r.Item('Creaform MetraSCAN',4)
    b._verify_tool_mapping(r,robot,frame,tool,metadata,'UR10','Frame 2','Creaform MetraSCAN')
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    out=Path('artifacts/speed_v1')/('blending_probe_'+stamp); out.mkdir()
    start=np.array([records[0][k] for k in ['x','y','z']]); end=np.array([records[1][k] for k in ['x','y','z']])
    saved=robot.Joints()
    joints=json.loads(Path('artifacts/speed_v1/joint_constrained_01/plan.csv.metadata.json').read_text(encoding='utf-8'))['diagnostics']['joint_constraint_audit']['joint_path_deg'][0]
    report={}
    try:
        for mode in ['stop','once','each_rounding','each_speed_rounding']:
            name='Probe_'+mode+'_'+stamp
            p=r.AddProgram(name,robot); p.setFrame(frame); p.setTool(tool)
            p.setSpeed(150,60,500,180); p.setRounding(0)
            for i,u in enumerate(np.linspace(0,1,4)):
                data=dict(records[0]); data.update(dict(zip(['x','y','z'],(1-u)*start+u*end)))
                target=r.AddTarget(name+'_P'+str(i+1),frame,robot)
                target.setPose(b._pose(SimpleNamespace(**data),a)); target.setJoints(joints); target.setAsCartesianTarget()
                if i==0:
                    p.MoveJ(target)
                    if mode!='stop': p.setRounding(1)
                else:
                    if mode=='each_speed_rounding': p.setSpeed(150,60,500,180)
                    if mode in ['each_rounding','each_speed_rounding']: p.setRounding(1)
                    if i==3: p.setRounding(0)
                    p.MoveL(target)
            robot.setJoints(joints)
            update=p.Update(timeout_sec=30)
            robot.setJoints(joints)
            msg,m,status=p.InstructionListJoints(flags=4,time_step=.005)
            rows=np.asarray(m.rows)
            elapsed=float(rows[10].sum())
            xyzspeed=np.linalg.norm(np.diff(rows[11:14],axis=1),axis=0)/np.maximum(rows[10,1:],1e-9)
            report[mode]={'program':name,'update':update,'trace_time_s':elapsed,
                          'path_length_mm':float(np.linalg.norm(end-start)),
                          'minimum_interior_xyz_speed':float(xyzspeed[2:-2].min()),'status':status,'message':msg}
            (out/(mode+'_trajectory.json')).write_text(json.dumps({'rows':m.rows}),encoding='utf-8')
            (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(mode,report[mode],flush=True)
    finally:
        robot.setJoints(saved)


if __name__=='__main__': main()
