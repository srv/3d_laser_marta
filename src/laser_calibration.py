# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 15:45:21 2018

@author: Propietario
"""

from laser_detector import detect_laser_subpixel
import copy
import cv2
import numpy as np
from math import isnan
import glob
from ransac import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def augment(xyzs):
	axyz = np.ones((len(xyzs), 4))
	axyz[:, :3] = xyzs
	return axyz

def estimate(xyzs):
	axyz = augment(xyzs[:3])
	return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
	return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

def plot_plane(a, b, c, d):
    xx, yy = np.mgrid[-3:3, -1:1]*0.1
    return xx, yy, (-d - a * xx - b * yy) / c

def intersection(plane,point,mtx):
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    rt = [(point[0]-cx)/fx,(point[1]-cy)/fy,1]
    t = - plane[3]/((rt[0]*plane[0])+(rt[1]*plane[1])+(plane[2]))
    point3D = np.array([[rt[0]*t, rt[1]*t,rt[2]*t]])
    return point3D

def fitPlane(norm, point):
    A = norm[0]
    B = norm[1]
    C = norm[2]
    D = -np.dot(norm,point)[0]
    return (A, B, C, D)


# import the necessary packages
import argparse

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dir", required=True,
    help="Directory of calibration images without trailing slash")
ap.add_argument("-f", "--format", required=True,
    help="Format of the images. Example: 'jpg'")
ap.add_argument("-c", "--calibration", required=True,
    help="Camera calibration file")
ap.add_argument("-D", "--distortion", required=True,
    help="Camera distortion coefficients file")
ap.add_argument("-o", "--output", required=True,
    help="Output prefix of the calibration result")
args = vars(ap.parse_args())

camera_matrix = np.load(args["calibration"])
dist_coeffs = np.load(args["distortion"])
images = glob.glob(args["dir"] + '/*.' + args["format"])

# termination criteria- plane[3]/np.sum([plane[0:2]*point,plane[2]])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32);
threshold = 0.1;
window_size = 7;

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)*0.04

#laserp = np.zeros((6*8,3), np.float32)
laser_points = np.array([[],[],[]]).T

borders3d = np.array([[- 0.15, - 0.1, 0],
                      [- 0.15, + 0.3, 0],
                      [+ 0, + 0.3, 0],
                      [+ 0, - 0.1, 0]])

for fname in images:
    img = cv2.imread(fname)
    img_undist = copy.deepcopy(img)

    newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, img.shape[:2], 0)
    cv2.undistort(img, camera_matrix, dist_coeffs, img_undist, newcamera)
    gray_undist = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)

    # Arrays to store object points and image points from all the images.
    objpoints = np.array([]) # 3d point in real world space
    imgpoints = np.array([]) # 2d points in image plane.

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray_undist, (8,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
#        objpoints=np.concatenate((objpoints,objp))

        corners2 = cv2.cornerSubPix(gray_undist, corners, (11,11), (-1, -1), criteria)
#        imgpoints=np.concatenate((imgpoints,corners2))

        # Draw and display the corners
        img2 = copy.deepcopy(img)
        img2 = cv2.drawChessboardCorners(img2, (8,6), corners2,ret)
        # cv2.imshow('img2',img2)
        # cv2.waitKey(10)


        retval, rvec, tvec = cv2.solvePnP(objp, corners2, newcamera, None)

        # Create mask
        borders, jacobian = cv2.projectPoints(borders3d, rvec, tvec, newcamera, None)
        borders = borders.reshape(4,2)

        mask = np.zeros(img.shape)
        mask = cv2.fillPoly(mask, pts = np.array([borders], dtype=np.int32), color=(255,255,255))
        # cv2.imshow('Laser mask', mask)


        R,_=cv2.Rodrigues(rvec)
        norm = R[0:3,2]
        checkerboard_plane = fitPlane(norm, tvec)

        #detect Laser pixels
        subpixel_peaks = detect_laser_subpixel(img_undist, kernel, threshold, window_size);
        cv2.namedWindow('Laser Image', cv2.WINDOW_KEEPRATIO)
        for p in subpixel_peaks:
            row = p[0]
            col = p[1]
            in_mask = (mask[int(row),int(col),0] > 0)
            if not isnan(row) and not isnan(col) and in_mask:
                img2 = cv2.circle(img2,(int(col),int(row)),15,(0,0,255),1)
                #Interseccion between checkerboard_plane and point of laser
                new_point = intersection(checkerboard_plane, (col,row), newcamera)
                # print 'Point ' +  str(p) + ' projected to ' + str(new_point)
                laser_points = np.concatenate((laser_points, new_point))
        cx = camera_matrix[0,2]
        cy = camera_matrix[1,2]
        img2 = cv2.circle(img2,(int(cy),int(cx)),15,(0,255,0),1)
        cv2.imshow('Laser Image', img2)
        cv2.waitKey(10)

#    cv2.imshow('Laser Image', img)

#planeLaser = main_plane_fitting(laser_points)
#%% Find laser plane with RANSAC

max_iterations = 1000
goal_inliers = laser_points.shape[0] * 0.85
print goal_inliers
#np.savetxt('laser_points.csv', laser_points, delimiter=' ')   # X is an array

# RANSAC
# print np.mean(laser_points, axis=0)

laser_plane, b = run_ransac(laser_points, estimate, lambda x, y: is_inlier(x, y, 0.01), 3, goal_inliers, max_iterations)
a, b, c, d = laser_plane
xx, yy, zz = plot_plane(a, b, c, d)

fit_points_h = augment(laser_points[:3])

data = np.linalg.svd(fit_points_h)[-1][-1, :]
# print 'data: ' + str(data)

U, s, V = np.linalg.svd(fit_points_h, full_matrices=True)
# print 'U: ' +  str(U)
# print 's: ' +  str(s)
# print 'V: ' +  str(V)

xx2, yy2, zz2 = plot_plane(-1.757035634445412e-01, 9.613484363790542e-01,  2.119845316631321e-01, -6.561783487838777e-02)

#laser_plane = [-1.757035634445412e-01, 9.613484363790542e-01,  2.119845316631321e-01, -6.561783487838777e-02]

ax = plt.figure().gca(projection='3d')
ax.plot_surface(xx, yy, zz, color=(1, 0, 0, 0.2))
ax.plot_surface(xx2, yy2, zz2, color=(0, 0, 1, 0.2))
ax.scatter3D(laser_points[:,0], laser_points[:,1], laser_points[:,2])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
plt.show()

np.save(args['output'] + "laserCalibration.npy", laser_plane)

cv2.waitKey(0)
cv2.destroyAllWindows()