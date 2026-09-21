"""Build, constrain and validate the captured 32-point case in one reproducible run."""
import json
import sys
from pathlib import Path
from datetime import datetime
from dataclasses import asdict
import argparse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from continuous_scan import plan_continuous_scan,constrain_with_robodk_joints
from pose_transform import load_extrinsic_config,transform_pose_records
from export_utils import write_speed_plan_csv


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--position-tolerance',type=float,default=.8)
    parser.add_argument('--orientation-tolerance',type=float,default=1.)
    parser.add_argument('--rounding',type=float,default=.05)
    args=parser.parse_args()
    case=Path('artifacts/speed_v1/capture_20260917_03')
    source=json.loads((case/'source_poses.json').read_text(encoding='utf-8'))
    output=Path('artifacts/speed_v1')/('constrained_case_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir()
    result=plan_continuous_scan(source,position_tolerance_mm=args.position_tolerance,
        orientation_tolerance_deg=args.orientation_tolerance,rounding_mm=args.rounding)
    _,metadata=transform_pose_records(source,load_extrinsic_config(str(case/'station_mapping.json')))
    result.diagnostics.update(metadata)
    result.diagnostics['original_source_pose_records']=result.diagnostics.pop('source_pose_records',[])
    print('Continuous IK for {} samples'.format(len(result.points)),flush=True)
    result=constrain_with_robodk_joints(result)
    write_speed_plan_csv(str(output/'plan.csv'),result)
    (output/'plan.json').write_text(json.dumps(asdict(result)),encoding='utf-8')
    print('Reference feasible={}, time={:.3f}s'.format(result.feasible,result.total_time),flush=True)
    from scripts.validate_saved_continuous_plan import main as validate
    sys.argv=['validate_saved_continuous_plan.py',str(output/'plan.csv')]
    validate()


if __name__=='__main__': main()
