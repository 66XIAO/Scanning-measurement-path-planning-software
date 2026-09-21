"""Update one retained experiment in place using the corrected MoveL ceiling.

Back up the current station first; reuse its namespace transactionally so that
repeated tests do not accumulate new targets in the working station.
"""
import argparse
import json
import sys
from pathlib import Path
from datetime import datetime
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from speed_planning_core import SpeedPlanPoint,SpeedPlanResult,ConstraintProfile
from continuous_scan import build_commands,import_continuous_scan
from continuous_export import export_continuous_scan
from export_utils import write_speed_plan_csv
import robodk_bridge as b


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('case_folder'); parser.add_argument('program')
    args=parser.parse_args()
    if not args.program.startswith('V1_Joint_'):
        raise ValueError('Only the retained V1_Joint experiment can be refreshed')
    case=Path(args.case_folder)
    data=json.loads((case/'plan.json').read_text(encoding='utf-8'))
    data['points']=[SpeedPlanPoint(**p) for p in data['points']]
    result=SpeedPlanResult(**data)
    profile=ConstraintProfile(**result.diagnostics['constraint_profile'])
    rounding=max(c['rounding_mm'] for c in result.diagnostics['incoming_commands'])
    result.diagnostics['incoming_commands']=[asdict(c) for c in build_commands(result,profile,rounding)]
    result.diagnostics['command_mapping_revision']='lower_positive_endpoint_v2'
    out=case/('incoming_v2_'+datetime.now().strftime('%Y%m%d_%H%M%S')); out.mkdir()
    api=b._import_api(); r=api['Robolink']()
    old=b._require_item(r,args.program,8,'retained program')
    if old.Busy(): raise RuntimeError('Retained program is running')
    targets_before=len(r.ItemList(6,True))
    r.Save(str((out/'before_update.rdk').resolve()))
    write_speed_plan_csv(str(out/'plan.csv'),result)
    (out/'plan.json').write_text(json.dumps(asdict(result)),encoding='utf-8')
    print('Replacing retained experiment transactionally',flush=True)
    report=import_continuous_scan(result,program_name=args.program,replace=True)
    if len(r.ItemList(6,True))!=targets_before:
        raise RuntimeError('Unexpected target count after namespace replacement')
    program=r.Item(args.program,8)
    report['update']=program.Update(timeout_sec=60)
    report['export']=export_continuous_scan(args.program,result.diagnostics['incoming_commands'],out/'export',result.diagnostics)
    (out/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('OUTPUT='+str(out),flush=True)


if __name__=='__main__': main()
