import g2o

def read_keypoints(f):
    keypoints = []
    with open(f, 'r') as keypoints_frame:
        for line in keypoints_frame:
            columns = line.strip().split()
            id, x, y, z = columns[:4]
            keypoints.append((int(id), int(x), int(y), int(z)))

    return keypoints

keypoints_0 = read_keypoints('./frames/0.txt')
keypoints_1 = read_keypoints('./frames/1.txt')

print(keypoints_0, keypoints_1)
print(g2o.test(keypoints_0, keypoints_1, len(keypoints_0), len(keypoints_1))
