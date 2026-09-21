"""Record motion-only playback with server clock and frame timestamp brackets.

GetSimTime protocol follows RoboDK's official SimulationTime implementation:
https://github.com/RoboDK/RoboDK-API/blob/master/Python/robodk/robolink.py
"""
import json
import sys
import time
from datetime import datetime
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robodk_bridge as b


def sim_time(r):
    r._send_line('GetSimTime')
    value=r._rec_int()/1000.
    r._check_status()
    return value


def main():
    source=Path(sys.argv[1]); selected=sys.argv[2] if len(sys.argv)>2 else None
    config=json.loads((source/'comparison.json').read_text(encoding='utf-8'))
    out=source/('clock_'+datetime.now().strftime('%Y%m%d_%H%M%S')); out.mkdir()
    api=b._import_api(); r=api['Robolink'](); robot=r.Item('UR10',2)
    saved=robot.Joints(); speed=r.SimulationSpeed(); mode=r.RunMode()
    seed=json.loads(Path('artifacts/speed_v1/joint_constrained_01/plan.csv.metadata.json').read_text(encoding='utf-8'))['diagnostics']['joint_constraint_audit']['joint_path_deg'][0]
    result={}
    try:
        r.setRunMode(1); r.setSimulationSpeed(1)
        for arm,spec in config.items():
            if selected and arm!=selected: continue
            p=r.Item(spec['program'],8); p.setRunType(1)
            if p.RunType()!=1 or r.RunMode()!=1: raise RuntimeError('Offline mode required')
            for i in range(p.InstructionCount()):
                if p.Instruction(i)[1] not in (0,2,3,4,10): raise ValueError('Non-motion instruction')
            robot.setJoints(seed)
            start=time.perf_counter(); sim_start=sim_time(r); p.RunProgram(); rows=[]
            print('Recording '+arm,flush=True)
            while p.Busy():
                wb=time.perf_counter()-start; sb=sim_time(r)
                fb=int(r.Command('LastRender')); joints=robot.Joints().tolist()
                fa=int(r.Command('LastRender')); sa=sim_time(r); wa=time.perf_counter()-start
                rows.append({'wall_before':wb,'wall_after':wa,'sim_before':sb-sim_start,'sim_after':sa-sim_start,
                             'frame_before_ms':fb,'frame_after_ms':fa,'joints_deg':joints})
                if wa>360: p.Stop(); raise RuntimeError('Playback timeout')
                time.sleep(.005)
            result[arm]={'program':spec['program'],'wall_time_s':time.perf_counter()-start,
                         'simulation_time_s':sim_time(r)-sim_start,'samples':rows}
            (out/'clock_playback.json').write_text(json.dumps(result),encoding='utf-8')
            print(arm,result[arm]['wall_time_s'],len(rows),'saved',str(out),flush=True)
    finally:
        robot.setJoints(saved); r.setSimulationSpeed(speed); r.setRunMode(mode)


if __name__=='__main__': main()
