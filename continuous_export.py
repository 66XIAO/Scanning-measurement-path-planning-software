"""Export offline URScript with command-table precision, preserving RoboDK poses.

RoboDK 4's stock UR post rounds metres to three decimals for blending, which
erases submillimetre radii. Keep its output as evidence and generate a separate
script with explicit, verified speed/acceleration/blending arguments.
"""
import hashlib
import json
import math
from pathlib import Path
import re


def precise_script(source, commands):
    lines=source.splitlines()
    moves=[i for i,line in enumerate(lines) if re.match(r'\s*move[jl]\(',line)]
    if len(moves)!=len(commands):
        raise ValueError('Exported movement count does not match incoming command table')
    for index,(line_index,c) in enumerate(zip(moves,commands)):
        line=lines[line_index]
        match=re.match(r'(\s*)(move[jl])\((.*\]),.*\)\s*$',line)
        if not match or match[2]!=('movej' if index==0 else 'movel'):
            raise ValueError('Unexpected URScript move syntax/order')
        keys=('joint_speed','joint_accel') if index==0 else ('linear_speed','linear_accel')
        factor=math.pi/180 if index==0 else .001
        speed=float(c[keys[0]])*factor
        accel=float(c[keys[1]])*factor
        radius=float(c['rounding_mm'])*.001
        if not all(math.isfinite(v) for v in (speed,accel,radius)) or min(speed,accel)<=0 or radius<0:
            raise ValueError('Invalid export command')
        if index in (0,len(commands)-1) and radius!=0:
            raise ValueError('Approach and final target must be exact stops')
        lines[line_index]=f'{match[1]}{match[2]}({match[3]},a={accel:.9f},v={speed:.9f},t=0,r={radius:.9f})'
    return '\n'.join(lines)+'\n'


def export_continuous_scan(program_name, commands, output_directory, diagnostics=None):
    import robodk_bridge as b
    from types import SimpleNamespace
    if not re.fullmatch(r'[A-Za-z0-9_]+',program_name):
        raise ValueError('Export program name must contain only letters, numbers and underscores')
    output=Path(output_directory).resolve()
    output.mkdir(parents=True,exist_ok=False)
    legacy=output/'stock_post_output'; legacy.mkdir()
    api=b._import_api(); rdk=api['Robolink']()
    program=b._require_item(rdk,program_name,api['ITEM_TYPE_PROGRAM'],'program')
    if diagnostics is not None:
        names=[diagnostics.get('expected_robodk_'+k+'_name',default) for k,default in
               [('robot','UR10'),('frame','Frame 2'),('tool','Creaform MetraSCAN')]]
        items=[b._require_item(rdk,name,api['ITEM_TYPE_'+kind],kind.lower()) for name,kind in
               zip(names,['ROBOT','FRAME','TOOL'])]
        b._verify_tool_mapping(rdk,*items,diagnostics,*names)
    moves=[]
    for i in range(program.InstructionCount()):
        instruction=program.Instruction(i)
        if instruction[1]==0:
            moves.append(instruction)
    if len(moves)!=len(commands):
        raise ValueError('Live program and command table have different movement counts')
    for instruction,command in zip(moves,commands):
        error=b._tool_pose_errors(b._matrix_rows(instruction[4]),
                                 b._matrix_rows(b._pose(SimpleNamespace(**command),api)))
        if error[0]>.001 or error[1]>.001:
            raise ValueError('Live program poses changed after planning')
    success,log,transfer=program.MakeProgram(legacy.as_posix()+'/',3)
    if not success or transfer:
        raise RuntimeError('Offline export failed or unexpected transfer: '+log)
    scripts=list(legacy.glob('*.script'))
    if len(scripts)!=1:
        raise ValueError('Expected exactly one stock URScript output')
    source=scripts[0].read_text(encoding='utf-8')
    result=precise_script(source,commands)
    path=output/(program_name+'.script')
    path.write_text(result,encoding='utf-8')
    report={'program':program_name,'move_count':len(commands),
            'stock_sha256':hashlib.sha256(source.encode()).hexdigest(),
            'precise_sha256':hashlib.sha256(result.encode()).hexdigest(),
            'method':'preserve stock post poses and order; explicit SI command arguments',
            'offline_only':True,'hardware_validated':False,
            'script':str(path),'minimum_internal_rounding_mm':min(c['rounding_mm'] for c in commands[1:-1])}
    (output/'export_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report
