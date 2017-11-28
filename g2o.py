import urbg2o

path = '/data/urbinn/notebooks/urb/sequence1911/dump/'
pose = urbg2o.poseOptimization(path + '1.txt')
print(pose)
print(type(pose))
print(g2o.bundle_adjustment())
