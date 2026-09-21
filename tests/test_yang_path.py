import unittest
import numpy as np
from smoothing.yang_path import smooth_and_audit, _slerp, _rotation_error
from pose_transform import quaternion_to_rotation


def records():
    return [dict(index=i+1,x=x,y=y,z=0,qw=1,qx=0,qy=0,qz=0)
            for i,(x,y) in enumerate([(0,0),(100,0),(100,100),(200,100)])]


class YangAdapterTests(unittest.TestCase):
    def test_constant_orientation_nonzero_join_tangent(self):
        output,audit,raw = smooth_and_audit(records(), samples_per_piece=17)
        self.assertTrue(audit['sampled_tolerances_pass'])
        self.assertNotIn('continuity',raw)
        self.assertLess(max(j['tangent_angle_deg'] for j in audit['joins']),1e-4)
        self.assertLess(max(j['position_gap_mm'] for j in audit['joins']),1e-9)
        for key in ['x','y','z','qw','qx','qy','qz']:
            self.assertAlmostEqual(output[0][key],records()[0][key])
            self.assertAlmostEqual(output[-1][key],records()[-1][key])

    def test_shortest_rotation_across_sign(self):
        a=quaternion_to_rotation([np.cos(np.radians(179)/2),0,0,np.sin(np.radians(179)/2)])
        b=quaternion_to_rotation([np.cos(np.radians(-179)/2),0,0,np.sin(np.radians(-179)/2)])
        middle=_slerp(a,b,.5)
        self.assertAlmostEqual(_rotation_error(a,middle),1,places=6)

    def test_duplicate_position_rejected(self):
        data=records(); data[1].update(x=0,y=0)
        with self.assertRaises(ValueError):
            smooth_and_audit(data)


if __name__=='__main__':
    unittest.main()
