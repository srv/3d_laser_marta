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
from matplotlib import pylab
from mpl_toolkits import mplot3d

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
    xx, yy = np.mgrid[:10, :10]
    return xx, yy, (-d - a * xx - b * yy) / c

def intersection(plane,point,mtx):
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    rt = [(point[0]-cx)/fx,(point[1]-cy)/fy,1]
    t = - plane[3]/((rt[0]*plane[0])+(rt[1]*plane[1])+(plane[2]))
    point3D = np.array([rt[0]*t, rt[1]*t,rt[2]*t])
    return(point3D)

def fitPlane(norm,point):
    A=norm[0]
    B=norm[1]
    C=norm[2]
    D=-np.sum(norm*point)
    return(A,B,C,D)


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
laser_points = np.array([])

borders3d = np.array([[- 0.15, - 0.1, 0],
                      [- 0.15, + 0.3, 0],
                      [+ 0, + 0.3, 0],
                      [+ 0, - 0.1, 0]])

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Arrays to store object points and image points from all the images.
    objpoints = np.array([]) # 3d point in real world space
    imgpoints = np.array([]) # 2d points in image plane.

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
#        objpoints=np.concatenate((objpoints,objp))

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#        imgpoints=np.concatenate((imgpoints,corners2))

        # Draw and display the cornersç
        img2 = copy.deepcopy(img)
        img2 = cv2.drawChessboardCorners(img2, (8,6), corners2,ret)
        # cv2.imshow('img2',img2)
        # cv2.waitKey(10)


        retval, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)

        # Create mask
        borders, jacobian = cv2.projectPoints(borders3d, rvec, tvec, camera_matrix, dist_coeffs)
        borders = borders.reshape(4,2)

        mask = np.zeros(img.shape)
        mask = cv2.fillPoly(mask, pts = np.array([borders], dtype=np.int32), color=(255,255,255))
        # cv2.imshow('Laser mask', mask)


        R,_=cv2.Rodrigues(rvec)
        norm = R[0:3,2]
        planeCam=fitPlane(norm,tvec)

        #detect Laser pixels
        frame = copy.deepcopy(img)
        newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, img.shape[:2], 0)
        cv2.undistort(img, camera_matrix, dist_coeffs, frame, newcamera)
        subpixel_peaks = detect_laser_subpixel(frame, kernel, threshold, window_size);
        cv2.namedWindow('Laser Image', cv2.WINDOW_KEEPRATIO)
        for p in subpixel_peaks:
            x = p[0]
            y = p[1]
            in_mask = (mask[int(x),int(y),0] > 0)
            if not isnan(x) and not isnan(y) and in_mask:
                img2 = cv2.circle(img2,(int(y),int(x)),15,(0,0,255),1)
                #Interseccion between planeCalib and point of laser
                new_point = intersection(planeCam, p, newcamera)
                laser_points=np.concatenate((laser_points, new_point))

        cv2.imshow('Laser Image', img2)
        cv2.waitKey(10)

#    cv2.imshow('Laser Image', img)

#planeLaser = main_plane_fitting(laser_points)
#%% Find laser plane with RANSAC

fig = pylab.figure()
ax = mplot3d.Axes3D(fig)

n = 100
max_iterations = 100
goal_inliers = n * 0.3

xyzs = laser_points

ax.scatter3D(xyzs.T[0], xyzs.T[1], xyzs.T[2])

	# RANSAC
laser_plane, b = run_ransac(xyzs, estimate, lambda x, y: is_inlier(x, y, 0.01), 3, goal_inliers, max_iterations)
a, b, c, d = laser_plane
xx, yy, zz = plot_plane(a, b, c, d)
ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))

np.save(args['output'] + "laserCalibration.npy", laser_plane)

cv2.waitKey(10000)
#cv2.destroyAllWindows()