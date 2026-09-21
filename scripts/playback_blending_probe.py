"""Measure wall-clock playback at 1x for our small, motion-only simulator probes."""
import json
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robodk_bridge as b


def main():
    folder=Path(sys.argv[1])
    if (folder/'validation.json').exists():
        probes={'joint_constrained':json.loads((folder/'validation.json').read_text(encoding='utf-8'))}
    else:
        input_file=folder/('comparison.json' if (folder/'comparison.json').exists() else 'report.json')
        probes=json.loads(input_file.read_text(encoding='utf-8'))
    api=b._import_api(); r=api['Robolink'](); robot=r.Item('UR10',2)
    saved=robot.Joints(); saved_speed=r.SimulationSpeed(); saved_mode=r.RunMode()
    seed=json.loads(Path('artifacts/speed_v1/joint_constrained_01/plan.csv.metadata.json').read_text(encoding='utf-8'))['diagnostics']['joint_constraint_audit']['joint_path_deg'][0]
    report={}
    try:
        r.setRunMode(1); r.setSimulationSpeed(1)
        if r.RunMode()!=1: raise RuntimeError('Simulator mode not active')
        for mode,details in probes.items():
            p=r.Item(details['program'],8); p.setRunType(1)
            if p.RunType()!=1: raise RuntimeError('Program is not configured for simulator')
            for i in range(p.InstructionCount()):
                if p.Instruction(i)[1] not in (0,2,3,4,10):
                    raise ValueError('Probe contains non-motion instructions')
            robot.setJoints(seed)
            start=time.perf_counter(); instructions=p.RunProgram(); samples=[]
            while p.Busy():
                samples.append([time.perf_counter()-start,*robot.Joints().tolist()])
                if time.perf_counter()-start>360:
                    p.Stop(); raise RuntimeError('Playback exceeded 360 seconds')
                time.sleep(.02)
            elapsed=time.perf_counter()-start
            report[mode]={'wall_time_s':elapsed,'simulation_speed':1,'instructions':instructions,'samples':samples}
            (folder/'playback.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(mode,elapsed,flush=True)
    finally:
        robot.setJoints(saved); r.setSimulationSpeed(saved_speed); r.setRunMode(saved_mode)


if __name__=='__main__': main()
