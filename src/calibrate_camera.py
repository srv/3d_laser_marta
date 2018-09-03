import numpy as np
import cv2
import glob
import yaml

# import the necessary packages
import argparse

print "I'm in"
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dir", required=True,
    help="Directory of calibration images without trailing slash")
ap.add_argument("-f", "--format", required=True,
    help="Format of the images. Example: 'jpg'")
ap.add_argument("-o", "--output", required=True,
    help="Output prefix of the calibration result")
args = vars(ap.parse_args())

images = glob.glob(args["dir"] + '/*.' + args["format"])

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)*0.04

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (8,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(10)

cv2.destroyAllWindows()

print('Computing calibration...')
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# It's very important to transform the matrix to list.

print('Saving results in ' + args['output'] + "calibration.yaml")
print mtx
print dist
data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
np.save(args['output'] + "cameraIntrinsics.npy",mtx)
np.save(args['output'] + "cameraDistCoeff.npy",dist)
print("DONE!")