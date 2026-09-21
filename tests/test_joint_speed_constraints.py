import unittest
import numpy as np
from joint_speed_constraints import retime_joint_path


class JointRetimingTests(unittest.TestCase):
    def test_linear_joint_mapping(self):
        s=np.linspace(0,100,101); q=np.zeros((101,6)); q[:,0]=2*s
        v,d=retime_joint_path(s,q,np.full(101,200.),500,60,180)
        self.assertEqual(v[0],0); self.assertEqual(v[-1],0)
        self.assertLessEqual(max(v),24+1e-8)
        self.assertTrue(d['sampled_joint_constraints_pass'])

    def test_high_curvature_slows_locally(self):
        s=np.linspace(0,100,501); q=np.zeros((501,6)); q[:,0]=20*np.tanh((s-50)/3)
        v,d=retime_joint_path(s,q,np.full(501,100.),500)
        self.assertLess(v[250],v[100])
        self.assertTrue(d['sampled_joint_constraints_pass'])

    def test_duplicate_parameter_rejected(self):
        with self.assertRaises(ValueError):
            retime_joint_path([0,1,1],np.zeros((3,6)),[10,10,10],500)
