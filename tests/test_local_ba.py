import numpy as np
import urbg2o
cv_keyframes = np.load('cv_keyframes.npy')
f_keyframes = np.load('f_keyframes.npy')
mappoints = np.load('mappoints.npy')
links = np.load('links.npy')

urbg2o.LocalBundleAdjustment(cv_keyframes, f_keyframes, mappoints, links)
