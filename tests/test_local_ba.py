import numpy as np
import urbg2o
keyframepoints = np.load('keyframepoints.npy')
keyframes = np.load('keyframes.npy')
links = np.load('links.npy')
urbg2o.LocalBundleAdjustment(keyframes, keyframepoints, links)

