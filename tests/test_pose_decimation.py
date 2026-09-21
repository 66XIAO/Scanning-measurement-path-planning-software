import unittest
import numpy as np
from smoothing.pose_decimation import decimate_poses


class PoseDecimationTests(unittest.TestCase):
    def test_constant_orientation_line_keeps_endpoints(self):
        rows=[dict(index=i+1,x=float(i),y=0,z=0,qw=1,qx=0,qy=0,qz=0) for i in range(100)]
        output,audit=decimate_poses(rows)
        self.assertEqual(len(output),2)
        self.assertEqual(output[0]['x'],0); self.assertEqual(output[-1]['x'],99)

    def test_orientation_peak_is_not_discarded(self):
        rows=[]
        for i in range(11):
            theta=.3*np.sin(np.pi*i/10)
            rows.append(dict(index=i+1,x=float(i),y=0,z=0,qw=np.cos(theta/2),qx=0,qy=0,qz=np.sin(theta/2)))
        output,audit=decimate_poses(rows)
        self.assertGreater(len(output),2)
        self.assertLessEqual(audit['max_sampled_orientation_deviation_deg'],.03+1e-8)
