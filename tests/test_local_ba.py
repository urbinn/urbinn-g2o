import numpy as np
import urbg2o
cv_keyframes = np.load('tests/cv_keyframes.npy')
f_keyframes = np.load('tests/f_keyframes.npy')
mappoints = np.load('tests/mappoints.npy')
links = np.load('tests/links.npy')
cv_keyframes = np.asfortranarray(cv_keyframes)
mappoints = np.asfortranarray(mappoints)
links = np.array(links, order='f')

for a in cv_keyframes:
    print(a)


urbg2o.LocalBundleAdjustment(cv_keyframes, f_keyframes, mappoints, links)

for a in cv_keyframes:
    print(a)
