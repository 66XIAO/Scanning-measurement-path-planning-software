"""Offline tolerance sensitivity on the captured case, before execution tests."""
import json
import sys
from pathlib import Path
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from continuous_scan import plan_continuous_scan


def main():
    source=json.loads(Path('artifacts/speed_v1/capture_20260917_03/source_poses.json').read_text(encoding='utf-8'))
    output=Path('artifacts/speed_v1')/('tolerance_sweep_'+datetime.now().strftime('%Y%m%d_%H%M%S')); output.mkdir()
    rows=[]
    for position in [.8,1.5,3.]:
        for orientation in [1.,2.,3.]:
            row={'position_tolerance_mm':position,'orientation_tolerance_deg':orientation}
            try:
                result=plan_continuous_scan(source,position_tolerance_mm=position,
                                            orientation_tolerance_deg=orientation,rounding_mm=.3)
                audit=result.diagnostics['smoothing_audit']
                row.update(feasible=result.feasible,reference_time_s=result.total_time,
                    command_count=len(result.points),position_error_mm=audit['max_sampled_position_error_mm'],
                    orientation_error_deg=audit['max_sampled_full_orientation_error_deg'],
                    decimation=audit['decimation'])
            except ValueError as error:
                row.update(feasible=False,error=str(error))
            rows.append(row)
            print({k:v for k,v in row.items() if k!='decimation'},flush=True)
    (output/'sweep.json').write_text(json.dumps({'scope':'TCP reference only; not joint-constraint or execution acceptance','results':rows},indent=2),encoding='utf-8')


if __name__=='__main__': main()
