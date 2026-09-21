import unittest
from continuous_export import precise_script


class PreciseExportTests(unittest.TestCase):
    def test_submillimetre_blending_survives(self):
        source='def demo():\n  movej([0,0,0,0,0,0],a,v,0,0)\n  movel(p[1,2,3,0,0,0],a,v,0,r)\n  movel(p[2,2,3,0,0,0],a,v,0,r)\nend\n'
        commands=[dict(linear_speed=25.123456,linear_accel=500,joint_speed=60,joint_accel=180,rounding_mm=r) for r in [0,.05,0]]
        result=precise_script(source,commands)
        self.assertIn('r=0.000050000',result)
        self.assertIn('v=0.025123456',result)
        self.assertIn('p[1,2,3,0,0,0]',result)
        self.assertEqual(result.count('r=0.000000000'),2)

    def test_mismatched_program_rejected(self):
        with self.assertRaises(ValueError):
            precise_script('def x():\nend', [{}])


if __name__=='__main__':
    unittest.main()
