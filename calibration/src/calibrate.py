import rosbag
import numpy as np
import os
import matplotlib.pylab as plt
import cv2
WORKDIR_PATH = '/home/koji/git/dvs_localziation/'

pattern_size = (6, 6) # corners of checkerboard
square_size = 2.2 #[cm], size of checkerboard

def generate_integrated_img(bag, n_events=50000, n_fill=4, img_size=[720, 1280]):
    xs = []
    ys = []
    cnt = 0
    for topic, msg, t in bag:
        if topic=='/prophesee/camera/cd_events_buffer':
            for event in msg.events:
                xs.append(event.x)
                ys.append(event.y)
                cnt += 1
            if cnt >= n_events:
                break
        if cnt >= n_events:
            break

    integrated_img = np.ones(img_size + [3]) * 255

    for x, y in zip(xs, ys):
        integrated_img[y-n_fill : y+n_fill, x-n_fill : x+n_fill] = np.array([1, 1, 1])
    integrated_img = integrated_img.astype(np.uint8)
    return integrated_img

pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

imgs = []
imgpoints = []
objpoints = []

print('Start calibration.')

# Extract corners for each images
i = 0
while True:
    bagpath = WORKDIR_PATH+'calibration/data/prophesee_rosbag/rec{}.bag'.format(i)
    if not os.path.exists(bagpath):
        break
    bag = rosbag.Bag(bagpath)
    
    # Integrate events to get one image from one bag
    integrated_img = generate_integrated_img(bag)
    imgs.append(integrated_img)
    ret, corner = cv2.findChessboardCorners(integrated_img, pattern_size)
    if ret:
        print('#{}: Corner detected.'.format(i))
        imgpoints.append(corner.reshape(-1, 2))
        objpoints.append(pattern_points)
    else:
        print('#{}: Not detected.'.format(i))
    i += 1

# Calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (1280, 720), None, None)
if ret:
    print('Calibration finished.')
    print('matrix:', mtx)
    print('distortion parameters:', dist)
np.savetxt('../data/matrix.txt', mtx)
np.savetxt('../data/dist.txt', dist)

# Undistort image
h, w = imgs[2].shape[:2]
newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
dst = cv2.undistort(imgs[2], mtx, dist, None, newcameramatrix)
fig, (axL, axR) = plt.subplots(ncols=2, figsize=(10,4))
axL.imshow(imgs[2])
axR.imshow(dst)
plt.show()