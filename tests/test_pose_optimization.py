import unittest
import urbg2o
import numpy as np

class MainTest(unittest.TestCase):
  def test_pose_optimization(self):
    pose = urbg2o.poseOptimization(np.ones((100, 6), dtype=np.float64, order='f'), np.ones((100, 6), dtype=np.float64, order='f'))
    self.assertEqual(pose, 100)

if __name__ == '__main__':
    unittest.main()