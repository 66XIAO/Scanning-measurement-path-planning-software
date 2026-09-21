"""Build motion-only variants sharing immutable targets for fair offline tests.

Each variant uses the same configurable six-axis simulation bounds. Scaling
is a declared experimental time stretch; no source program is edited.
"""
import json
import sys
from pathlib import Path
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robodk_bridge as b
from continuous_export import export_continuous_scan


def create_variant(r,api,source_name,commands,name,scale,fixed_speed=None):
    robot=r.Item('UR10',2); frame=r.Item('Frame 2',3); tool=r.Item('Creaform MetraSCAN',4)
    source=b._require_item(r,source_name,8,'source program')
    original_moves=[]
    for i in range(source.InstructionCount()):
        instruction=source.Instruction(i)
        if instruction[1]==0:
            original_moves.append(instruction)
    if len(original_moves)!=len(commands): raise ValueError('Source count mismatch')
    p=r.AddProgram(name,robot); p.setFrame(frame); p.setTool(tool)
    table=[]
    for i,c0 in enumerate(commands):
        c=dict(c0)
        if i:
            c['linear_speed']=(fixed_speed if fixed_speed is not None else c['linear_speed'])*scale
            c['linear_accel']*=scale**2
        target=b._require_item(r,source_name+'_P'+str(i+1),6,'shared target')
        error=b._tool_pose_errors(b._matrix_rows(target.Pose()),b._matrix_rows(original_moves[i][4]))
        if max(error)>.001: raise ValueError('Source target changed')
        p.setSpeed(c['linear_speed'],c['joint_speed'],c['linear_accel'],c['joint_accel'])
        p.setRounding(c['rounding_mm'])
        (p.MoveJ if i==0 else p.MoveL)(target)
        table.append(c)
    return table


def main():
    config=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    out=Path('artifacts/speed_v1')/('fair_playback_'+datetime.now().strftime('%Y%m%d_%H%M%S')); out.mkdir()
    api=b._import_api(); r=api['Robolink'](); result={}
    for arm,spec in config.items():
        metadata=json.loads(Path(spec['metadata']).read_text(encoding='utf-8'))
        commands=metadata['diagnostics']['incoming_commands']
        name=arm+'_'+out.name
        print('Building '+name,flush=True)
        table=create_variant(r,api,spec['program'],commands,name,spec['scale'],spec.get('fixed_speed'))
        result[arm]={'program':name,'source_program':spec['program'],'scale':spec['scale'],'incoming_commands':table,
                     'limits_deg_s':60,'limits_deg_s2':180}
        result[arm]['export']=export_continuous_scan(name,table,out/arm,metadata['diagnostics'])
        (out/'comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PLAYBACK_FOLDER='+str(out),flush=True)
    from scripts.playback_blending_probe import main as playback
    sys.argv=['playback_blending_probe.py',str(out)]
    playback()


if __name__=='__main__': main()
