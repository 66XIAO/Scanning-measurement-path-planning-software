"""Recover observed simulator TCP poses with station-model FK and audit geometry."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import robodk_bridge as b
from pose_transform import load_extrinsic_config,transform_pose_records,quaternion_to_rotation,matrix_pose
from smoothing.yang_path import _slerp,_rotation_error


def main():
    root=Path(sys.argv[1]); playback=json.loads((root/'playback.json').read_text(encoding='utf-8'))
    case=Path('artifacts/speed_v1/capture_20260917_03')
    source=json.loads((case/'source_poses.json').read_text(encoding='utf-8'))
    config=load_extrinsic_config(str(case/'station_mapping.json'))
    _,metadata=transform_pose_records(source,config)
    api=b._import_api(); r=api['Robolink'](); robot=r.Item('UR10',2); frame=r.Item('Frame 2',3); tool=r.Item('Creaform MetraSCAN',4)
    b._verify_tool_mapping(r,robot,frame,tool,metadata,'UR10','Frame 2','Creaform MetraSCAN')
    inv_frame=np.linalg.inv(np.asarray(config.t_base_workpiece))
    tcp=np.asarray(b._matrix_rows(tool.PoseTool()))
    p=np.array([[a[k] for k in ['x','y','z']] for a in source])
    rot=np.array([quaternion_to_rotation([a[k] for k in ['qw','qx','qy','qz']]) for a in source])
    segment=np.diff(p,axis=0); length2=np.sum(segment*segment,axis=1)
    report={}
    for arm,entry in playback.items():
        rows=[]
        for i,sample in enumerate(entry['samples']):
            transform=inv_frame@np.asarray(b._matrix_rows(robot.SolveFK(sample[1:])))@tcp
            xyz=transform[:3,3]
            fractions=np.clip(np.sum((xyz-p[:-1])*segment,axis=1)/length2,0,1)
            distances=np.linalg.norm(xyz-p[:-1]-fractions[:,None]*segment,axis=1)
            j=int(np.argmin(distances)); reference=_slerp(rot[j],rot[j+1],float(fractions[j]))
            rows.append({'t':sample[0],'pose':matrix_pose(transform,i+1),'source_segment':j,
                         'position_error_mm':float(distances[j]),
                         'orientation_error_deg':_rotation_error(reference,transform[:3,:3])})
        (root/(arm+'_observed_poses.json')).write_text(json.dumps(rows),encoding='utf-8')
        report[arm]={'samples':len(rows),'max_position_error_mm':max(a['position_error_mm'] for a in rows),
                     'max_orientation_error_deg':max(a['orientation_error_deg'] for a in rows),
                     'scope':'Station-model FK of observed simulator joints, compared with closest original polyline point and corresponding SLERP orientation; no physical scan evidence.'}
        (root/'geometry_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(arm,report[arm],flush=True)


if __name__=='__main__': main()
