import urbg2o
import numpy as np

pose = urbg2o.poseOptimization(np.ones((100, 6), dtype=np.float64, order='f'), np.ones((100, 6), dtype=np.float64, order='f'))
print(pose)
print(type(pose))
