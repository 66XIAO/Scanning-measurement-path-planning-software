"""Capture the active RoboDK case without moving the robot or editing its source.

Run with the application's Python/NumPy environment. Output is immutable per run.
"""
import argparse
import json
import sys
from pathlib import Path
from datetime import datetime, timezone

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import robodk_bridge as bridge
from pose_transform import matrix_pose, save_extrinsic_config, transform_pose_records
from speed_planning_core import ConstraintProfile, plan_speed_profile
from export_utils import write_speed_plan_csv


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--source-program', default='A')
    parser.add_argument('--output', required=True)
    parser.add_argument('--import-test', action='store_true',
                        help='Create a new offline test program; never replace source')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    api = bridge._import_api()
    rdk = api['Robolink']()
    config = bridge.capture_robodk_station_mapping()
    save_extrinsic_config(str(output / 'station_mapping.json'), config)
    program = bridge._require_item(rdk, args.source_program, api['ITEM_TYPE_PROGRAM'], 'source program')
    # Legacy RoboDK cannot query PoseFrame/PoseTool on program items. This case
    # uses named frame/tool instructions and Cartesian targets; verify all three.
    frame = bridge._require_item(rdk, config.robodk_frame_name, api['ITEM_TYPE_FRAME'], 'frame')
    tool = bridge._require_item(rdk, config.robodk_tool_name, api['ITEM_TYPE_TOOL'], 'tool')
    records, instructions = [], []
    frame_seen = tool_seen = False
    for i in range(program.InstructionCount()):
        name, kind, move, joint_target, pose, joints = program.Instruction(i)
        instructions.append({'id': i, 'name': name, 'type': kind, 'move_type': move})
        if kind in (3, 4):
            expected_name = config.robodk_frame_name if kind == 3 else config.robodk_tool_name
            if records or name.split(':', 1)[-1].strip() != expected_name:
                raise ValueError('Unexpected or changing source frame/tool: ' + name)
            if kind == 3:
                frame_seen = True
            else:
                tool_seen = True
        if kind == 0:
            if not (frame_seen and tool_seen):
                raise ValueError('Source frame/tool instructions missing')
            if joint_target:
                raise ValueError('Joint target source requires explicit FK conversion')
            target = bridge._require_item(rdk, args.source_program + '_P' + str(len(records)+1),
                                          api['ITEM_TYPE_TARGET'], 'source target')
            if target.Parent() != frame:
                raise ValueError('Target reference does not match source frame')
            errors = bridge._tool_pose_errors(bridge._matrix_rows(pose), bridge._matrix_rows(target.Pose()))
            if errors[0] > .01 or errors[1] > .01:
                raise ValueError('Instruction and target pose differ')
            records.append(matrix_pose(np.asarray(bridge._matrix_rows(pose)), len(records) + 1))
    if len(records) != 32:
        raise ValueError('Required case is 32 poses, got {}'.format(len(records)))
    (output / 'source_poses.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    _, metadata = transform_pose_records(records, config)
    result = plan_speed_profile(records, ConstraintProfile())
    result.diagnostics.update(metadata)
    write_speed_plan_csv(str(output / 'legacy_speed_plan.csv'), result)
    report = {'captured_at': datetime.now(timezone.utc).isoformat(),
              'source_program': args.source_program, 'station': config.robodk_station_name,
              'pose_count': len(records), 'source_instructions': instructions,
              'planned_time_s': result.total_time, 'feasible': result.feasible,
              'scope': 'offline; no hardware execution; legacy speed semantics pending migration'}
    if args.import_test:
        name = 'SpeedV1_Import_' + datetime.now().strftime('%Y%m%d_%H%M%S')
        report['import'] = bridge.import_speed_plan(result, program_name=name)
    (output / 'capture_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(output), 'pose_count': len(records),
                      'feasible': result.feasible, 'imported': report.get('import', {}).get('program')}, ensure_ascii=True))


if __name__ == '__main__':
    main()
