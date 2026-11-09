import math
import unittest
from scripts.show_targets_rviz import target_axes, rotate_vector_xyzw

class TestTCPOrientation(unittest.TestCase):
    def test_identity(self):
        actual=target_axes([0,0,0,1],0.02)
        expected=((.02,0,0),(0,.02,0),(0,0,.02))
        for a,b in zip(actual,expected):
            for x,y in zip(a,b):self.assertAlmostEqual(x,y,places=8)

    def test_downward_tcp(self):
        axes=target_axes([1,0,0,0],0.02)
        self.assertAlmostEqual(axes[0][0],.02,places=8)
        self.assertAlmostEqual(axes[1][1],-.02,places=8)
        self.assertAlmostEqual(axes[2][2],-.02,places=8)

    def test_world_paper_yaw(self):
        theta=math.pi/4
        q=(math.cos(theta/2),math.sin(theta/2),0.,0.)
        v=rotate_vector_xyzw(q,(0.,0.,.02))
        self.assertAlmostEqual(v[2],-.02,places=8)
        self.assertAlmostEqual(v[0],0.,places=8)
        self.assertAlmostEqual(v[1],0.,places=8)

    def test_nonunit_quaternion_normalised(self):
        self.assertAlmostEqual(rotate_vector_xyzw((2,0,0,0),(0,0,1))[2],-1,places=8)

if __name__=='__main__':unittest.main()
