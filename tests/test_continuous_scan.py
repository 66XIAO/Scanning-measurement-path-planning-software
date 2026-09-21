import unittest
import csv
import json
import tempfile
from pathlib import Path
from dataclasses import replace
from continuous_scan import build_commands, plan_continuous_scan
from speed_planning_core import ConstraintProfile, plan_speed_profile


def path():
    return [dict(index=i+1,x=x,y=y,z=0,qw=1,qx=0,qy=0,qz=0)
            for i,(x,y) in enumerate([(0,0),(100,0),(100,100),(200,100)])]


class ContinuousScanTests(unittest.TestCase):
    def test_reference_and_command_csv_are_separate(self):
        from export_utils import write_speed_plan_csv
        result=plan_continuous_scan(path())
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'plan.csv'
            write_speed_plan_csv(str(target),result)
            with target.open(encoding='utf-8-sig') as stream:
                reference=list(csv.DictReader(stream))
            with target.with_name('plan.commands.csv').open(encoding='utf-8-sig') as stream:
                commands=list(csv.DictReader(stream))
            self.assertEqual(float(reference[0]['linear_speed']),0)
            self.assertTrue(all(float(c['linear_speed'])>0 for c in commands))
            self.assertEqual(reference[0]['source_X'],'')
            metadata=json.loads(Path(str(target)+'.metadata.json').read_text(encoding='utf-8'))
            self.assertEqual(metadata['schema_version'],'2.0')
            self.assertEqual(metadata['command_csv'],'plan.commands.csv')

    def test_reference_rest_and_positive_commands(self):
        result=plan_continuous_scan(path())
        self.assertTrue(result.feasible)
        self.assertEqual(result.points[0].linear_speed,0)
        self.assertEqual(result.points[-1].linear_speed,0)
        commands=result.diagnostics['incoming_commands']
        self.assertIsNone(commands[0]['incoming_segment'])
        self.assertEqual(commands[-1]['incoming_segment'],len(commands)-2)
        self.assertEqual(commands[0]['rounding_mm'],0)
        self.assertEqual(commands[-1]['rounding_mm'],0)
        self.assertTrue(all(c['rounding_mm']>0 for c in commands[1:-1]))
        self.assertTrue(all(c['linear_speed']>0 for c in commands))

    def test_incoming_segment_not_destination_speed(self):
        profile=ConstraintProfile()
        result=plan_speed_profile(path(),profile)
        result.points=[replace(p,linear_speed=v) for p,v in zip(result.points,[0,10,30,0])]
        commands=build_commands(result,profile)
        self.assertEqual([c.linear_speed for c in commands[1:]],[10,10,30])

    def test_deceleration_ceiling_does_not_keep_upstream_speed(self):
        profile=ConstraintProfile()
        result=plan_speed_profile(path(),profile)
        result.points=[replace(p,linear_speed=v) for p,v in zip(result.points,[0,100,10,0])]
        commands=build_commands(result,profile)
        self.assertEqual(commands[2].linear_speed,10)

    def test_invalid_blending_rejected(self):
        result=plan_speed_profile(path())
        with self.assertRaises(ValueError):
            build_commands(result,ConstraintProfile(),float('nan'))


if __name__=='__main__':
    unittest.main()
