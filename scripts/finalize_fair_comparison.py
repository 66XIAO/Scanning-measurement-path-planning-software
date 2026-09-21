"""Finalize a native MoveL comparison with separated evidence domains.

Wall-clock playback measures simulator duration. RoboDK's time-based program
trajectory supplies model speeds/accelerations and error flags. Program
structure and exported URScript establish nonzero internal rounding. These do
not establish physical-robot timing, torque, collision or scan quality.
"""
import csv
import json
import re
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import robodk_bridge as b


def main():
    root=Path(sys.argv[1])
    config=json.loads((root/'comparison.json').read_text(encoding='utf-8'))
    playback=json.loads((root/'playback.json').read_text(encoding='utf-8'))
    geometry=json.loads((root/'geometry_audit.json').read_text(encoding='utf-8'))
    api=b._import_api(); rdk=api['Robolink']()
    report={}
    for arm,spec in config.items():
        program=b._require_item(rdk,spec['program'],8,'comparison program')
        update=program.Update(timeout_sec=60)
        message,matrix,status=program.InstructionListJoints(flags=4,time_step=.01)
        rows=np.asarray(matrix.rows,dtype=float)
        if rows.shape[0]!=26 or not np.isfinite(rows).all(): raise ValueError('Unexpected trajectory schema')
        errors=sorted(set(rows[6].astype(int).tolist()))
        native_v=np.max(abs(rows[14:20]),axis=1); native_a=np.max(abs(rows[20:26]),axis=1)
        commands=spec['incoming_commands']; radii=np.array([c['rounding_mm'] for c in commands])
        instructions=[program.Instruction(i)[1] for i in range(program.InstructionCount())]
        expected=[]
        for _ in commands: expected += [2,10,0]
        expected=[3,4]+expected
        if instructions!=expected: raise RuntimeError('Program instruction sequence changed')
        script=Path(spec['export']['script'])
        text=script.read_text(encoding='utf-8')
        move_lines=[line for line in text.splitlines() if re.match(r'\s*move[jl]\(',line)]
        exported_r=[float(re.search(r',r=([0-9.]+)\)',line).group(1))*1000 for line in move_lines]
        if len(exported_r)!=len(commands): raise RuntimeError('Export move count mismatch')
        metrics={'program':spec['program'],'wall_time_s':playback[arm]['wall_time_s'],
          'pose_count':len(commands),'instruction_count':program.InstructionCount(),
          'update_valid_ratio':update[3],'trajectory_status':status,'trajectory_message':message,
          'trajectory_error_codes':errors,'peak_native_joint_speed_deg_s':native_v.tolist(),
          'peak_native_joint_accel_deg_s2':native_a.tolist(),
          'native_joint_limits_pass':bool(max(native_v)<=60 and max(native_a)<=180 and errors==[0]),
          'internal_rounding_min_mm':float(radii[1:-1].min()),
          'internal_rounding_nonzero':bool(np.all(radii[1:-1]>0)),
          'endpoint_rounding_zero':bool(radii[0]==0 and radii[-1]==0),
          'export_rounding_matches_commands':bool(np.allclose(exported_r,radii,atol=5e-7)),
          **geometry[arm],
          'scope':'RoboDK 4.0 offline simulation, station model, native MoveL; no collision, physical calibration, torque, controller or scan-quality validation.'}
        report[arm]=metrics
        (root/(arm+'_official_trajectory.json')).write_text(json.dumps({'status':status,'message':message,'rows':matrix.rows}),encoding='utf-8')
    base=report['A_stop']['wall_time_s']
    report['comparison']={
      'B_vs_A_time_reduction_percent':100*(1-report['B_fixed']['wall_time_s']/base),
      'C_vs_A_time_reduction_percent':100*(1-report['C_variable']['wall_time_s']/base),
      'C_vs_B_time_reduction_percent':100*(1-report['C_variable']['wall_time_s']/report['B_fixed']['wall_time_s']),
      'all_native_constraints_pass':all(report[k]['native_joint_limits_pass'] for k in ['A_stop','B_fixed','C_variable']),
      'continuous_arms_structurally_pass':all(report[k]['internal_rounding_nonzero'] and report[k]['endpoint_rounding_zero'] for k in ['B_fixed','C_variable'])}
    (root/'final_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    with (root/'summary.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream); writer.writerow(['arm','wall_time_s','poses','max_joint_speed_deg_s','max_joint_accel_deg_s2','native_constraints_pass','position_error_mm','orientation_error_deg'])
        for arm in ['A_stop','B_fixed','C_variable']:
            m=report[arm]; writer.writerow([arm,m['wall_time_s'],m['pose_count'],max(m['peak_native_joint_speed_deg_s']),max(m['peak_native_joint_accel_deg_s2']),m['native_joint_limits_pass'],m['max_position_error_mm'],m['max_orientation_error_deg']])
    print(json.dumps(report['comparison']),flush=True)


if __name__=='__main__': main()
