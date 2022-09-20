import numpy as np
import cv2
import glob
import os
import time
import matplotlib.pyplot as plt


def feature_matcher(orb, matcher, img1, img2):
    _, des1 = orb.detectAndCompute(img1, None)
    _, des2 = orb.detectAndCompute(img2, None)
    matches = matcher.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    return matches  #matches is a list which contains img2Idx, img1Idx, distance, imgIdx


def triangulation(kp1, kp2, T_1w, T_2w):
    kp1_3D = np.ones((3, kp1.shape[0]))
    kp2_3D = np.ones((3, kp2.shape[0]))
    kp1_3D[0], kp1_3D[1] = kp1[:, 0].copy(), kp1[:, 1].copy()
    kp2_3D[0], kp2_3D[1] = kp2[:, 0].copy(), kp2[:, 1].copy()
    X = cv2.triangulatePoints(T_1w[:3], T_2w[:3], kp1_3D[:2], kp2_3D[:2])
    X /= X[3]
    X1 = T_1w[:3] @ X
    X2 = T_2w[:3] @ X
    return X[:3], X1, X2 


if __name__ == '__main__':
    orb, matcher = cv2.ORB_create(), cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    img1 = cv2.resize(cv2.imread(os.path.join(os.getenv("HOME"), 'Downloads', '20220914_103425.jpg')), (800, 600))
    img2 = cv2.resize(cv2.imread(os.path.join(os.getenv("HOME"), 'Downloads', '20220914_103428.jpg')), (800, 600))
    t0 = time.time()
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    t1 = time.time()
    print('orb time:', t1-t0)
    matches = matcher.match(des1, des2)
    t2 = time.time()
    print('match time:', t2-t1)
    img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    #plt.imshow(img3),plt.show()
    print(len(matches))

