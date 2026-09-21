import unittest
from types import SimpleNamespace
import math
from trajectory_kinematics import angular_vector_diagnostics


class AngularVectorTests(unittest.TestCase):
    def test_rotation_reversal_is_acceleration(self):
        points=[SimpleNamespace(qw=math.cos(math.radians(a)/2),qx=0,qy=0,
                                qz=math.sin(math.radians(a)/2),dt_to_next=1) for a in [0,10,0]]
        result=angular_vector_diagnostics(points)
        self.assertAlmostEqual(result['max_tcp_angular_vector_speed_deg_s'],10)
        self.assertAlmostEqual(result['max_tcp_angular_vector_accel_deg_s2'],20)

    def test_quaternion_sign_does_not_change_rotation(self):
        points=[SimpleNamespace(qw=s,qx=0,qy=0,qz=0,dt_to_next=1) for s in [1,-1,1]]
        result=angular_vector_diagnostics(points)
        self.assertEqual(result['max_tcp_angular_vector_accel_deg_s2'],0)
