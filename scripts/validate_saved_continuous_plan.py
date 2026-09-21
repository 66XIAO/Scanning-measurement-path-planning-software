"""Import and validate a saved reference plan without rerunning IK or planning."""
import csv
import json
import sys
from pathlib import Path
from dataclasses import fields
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from speed_planning_core import SpeedPlanPoint,SpeedPlanResult
from continuous_scan import import_continuous_scan
from continuous_export import export_continuous_scan
from scripts.compare_continuous_scan import trajectory_metrics
import robodk_bridge as bridge


def main():
    path=Path(sys.argv[1])
    metadata=json.loads(Path(str(path)+'.metadata.json').read_text(encoding='utf-8'))
    aliases={'x':'X','y':'Y','z':'Z','qw':'w','qx':'x','qy':'y','qz':'z',
             'path_s':'path_s_mm','segment_length':'segment_length_mm','curvature':'curvature_1_mm',
             'angular_speed':'tcp_angular_speed_deg_s','angular_accel':'tcp_angular_accel_deg_s2',
             'dt_to_next':'dt_to_next_s','max_linear_speed':'max_linear_speed_mm_s'}
    points=[]
    with path.open(encoding='utf-8-sig',newline='') as stream:
        for row in csv.DictReader(stream):
            values={f.name:row[aliases.get(f.name,f.name)] for f in fields(SpeedPlanPoint)}
            values={k:(v=='True' if k=='feasible' else v if k=='violation_codes' else int(v) if k=='index' else float(v)) for k,v in values.items()}
            points.append(SpeedPlanPoint(**values))
    result=SpeedPlanResult(metadata['algorithm'],points,metadata['total_time_s'],metadata['feasible'],metadata['warnings'],metadata['diagnostics'])
    name='V1_Joint_'+datetime.now().strftime('%Y%m%d_%H%M%S')
    output=path.parent/name; output.mkdir()
    print('Importing '+name,flush=True)
    summary=import_continuous_scan(result,program_name=name)
    api=bridge._import_api(); rdk=api['Robolink'](); program=rdk.Item(name,8)
    print('Extracting trajectory',flush=True)
    summary['update']=program.Update(timeout_sec=60)
    msg,matrix,status=program.InstructionListJoints(mm_step=2,deg_step=1,flags=4,time_step=.01)
    (output/'trajectory.json').write_text(json.dumps({'message':msg,'status':status,'rows':matrix.rows}),encoding='utf-8')
    summary['trajectory']=trajectory_metrics(matrix.rows)
    summary['export']=export_continuous_scan(name,result.diagnostics['incoming_commands'],output/'export',result.diagnostics)
    (output/'validation.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary['trajectory']),flush=True)


if __name__=='__main__':
    main()
