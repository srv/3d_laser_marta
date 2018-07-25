# -*- coding: utf-8 -*-
"""
Created on Tue Jul 17 19:40:21 2018

@author: Propietario
"""

from laser_detector import detect_laser_subpixel
import copy
import cv2
import numpy as np
from math import isnan, sqrt
import glob
import serial
import time
from matplotlib import pylab
from mpl_toolkits import mplot3d
from pointcloud_viewer import ReconstructionViewer
import open3d

def outOfBounds(point,tvec):
    #Si la x esta fuera de los limites de la mesa
    if point[0]>tvec[0]+0.15:
        return True
    elif point[0]<tvec[0]-0.15:
        return True
    #O la coordenada y esta fuera de los limites
    elif point[1]>tvec[1]+0.15:
        return True
    elif point[1]<tvec[1]+0.15:
        return True
    else:
        return False
    #Sino se considerara como un punto acceptable


def intersection(plane,point,mtx):
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    rt = [(point[0]-cx)/fx,(point[1]-cy)/fy,1]
    t = - plane[3]/((rt[0]*plane[0])+(rt[1]*plane[1])+(plane[2]))
    point3D = np.array([[rt[0]*t, rt[1]*t,rt[2]*t]])
    return(point3D)

def calculate_rotation(frame, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
    marker_length=0.03 #in meters

    tvecs5=np.array([[[0,0,0]]])
    tvecs42=np.array([[[0,0,0]]])
    tvecs27=np.array([[[0,0,0]]])
    tvecs18=np.array([[[0,0,0]]])

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners,ids)
        i=0
        while(i<len(ids)):
            rvecs,tvecs,_objPoints  = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, dist_coeffs)
            cv2.aruco.drawAxis(frame,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
            if (ids[i]==5):
                R,_=cv2.Rodrigues(rvecs)
                t = np.array([-0.09,0,0])
                tvecs5=np.matmul(R,t.T)+tvecs
            if (ids[i]==42):
                R,_=cv2.Rodrigues(rvecs)
                t = np.array([+0.09,0,0])
                tvecs42=np.matmul(R,t.T)+tvecs
            if (ids[i]==27):
                R,_=cv2.Rodrigues(rvecs)
                t = np.array([0,-0.09,0])
                tvecs27=np.matmul(R,t.T)+tvecs
            if (ids[i]==18):
                R,_=cv2.Rodrigues(rvecs)
                t = np.array([0,+0.09,0])
                tvecs18=np.matmul(R,t.T)+tvecs
            i=i+1

        tvecs = (tvecs5+tvecs42+tvecs27+tvecs18)/len(ids) #si té tvecs més petit té més prioritat
        cv2.aruco.drawAxis(frame,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
        cv2.imshow('Tabe Calib',frame)

    return (R,tvecs)

# import the necessary packages
import argparse

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dir", required=True,
    help="Directory of images without trailing slash")
ap.add_argument("-f", "--format", required=True,
    help="Format of the images. Example: 'jpg'")
ap.add_argument("-c", "--calibration", required=True,
    help="Camera calibration file")
ap.add_argument("-D", "--distortion", required=True,
    help="Camera distortion coefficients file")
ap.add_argument("-l", "--laser", required=True,
    help="Laser calibration file")
args = vars(ap.parse_args())

#def rotate_points(frame,points):
#    calculate_rotation(frame)

#------------------------------MAIN--------------------------------------------
camera_matrix = np.load(args['calibration'])
dist_coeffs = np.load(args['distortion'])
laser_plane = np.load(args['laser'])

print('Camera calibration:')
print camera_matrix
print('Camera distortion:')
print dist_coeffs
print('Laser calibration:')
print laser_plane

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32)
threshold = 0.1
window_size = 7

detected_points = np.array([[],[],[]]).T
# Inicialment la taula té rotació zero i translació zero


viewer  = ReconstructionViewer()


# La matriu de rotació d'un pas de motor (4x4) angle=1.125 Rot z
table_increment = np.array([[0.999807,-0.01963,0,0],
                            [0.01963,0.999807,0,0],
                            [0,0,1,0],
                            [0,0,0,1]])

# La transformació del sistema de coordenades càmera a la taula (calibració, 4x4)
table_1to2 = np.identity(4)
table_2to1 = np.identity(4)
camera_to_table_1 = np.identity(4)
camera_to_table_2 = np.identity(4)
table_1_to_camera = np.identity(4)
table_2_to_camera = np.identity(4)

camera_to_table_1[0:3,0:3]=np.array([[-0.20223907, -0.91075492,  0.36003449],
                                     [-0.63157544, -0.15967696, -0.75869344],
                                     [ 0.748473  , -0.3808264 , -0.54291752]])
camera_to_table_1[0:3,3]= np.array([[-0.38777067, -0.13432474,  0.92294953]])

table2camera = np.array([[-0.2022,   -0.6316,    0.7485, -0.8541 ],
                         [-0.9108, -0.1597, -0.3808, -0.0231],
                         [ 0.3600, -0.7587, -0.5429,  0.5388],
                         [ 0, 0, 0, 1]])
tvec = np.array([-0.38777067, -0.13432474,  0.92294953])

mask = cv2.imread('calibrations/mask2.png')

step=1
while step<321:
    # Detect table
    fname = args['dir'] + '/step_num_table' + str(step) + '.' + args['format']
    frameTable = cv2.imread(fname)
    R,tvec = calculate_rotation(frameTable, camera_matrix, dist_coeffs)
    camera_to_table_2[0:3,0:3]=R
    camera_to_table_2[0:3,3]=tvec

    # camera_to_table_2 = np.matmul(camera_to_table_1, table_1to2)
    # table_1_to_camera = np.linalg.inv(camera_to_table_1)
    # table_2_to_camera = np.matmul(table_2to1, table_1_to_camera)
    table_2_to_camera = np.linalg.inv(camera_to_table_2)

    fname = args['dir'] + '/step_num' + str(step) + '.' + args['format']
    frame = cv2.imread(fname)

    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, frame.shape[:2], 0)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcamera)

    show_img = copy.deepcopy(frame)

    subpixel_peaks = detect_laser_subpixel(frame, kernel, threshold, window_size)
    subpixel_peaks= np.expand_dims(subpixel_peaks, 1)
    rectified_points = subpixel_peaks

    laser_points = np.array([[],[],[]]).T
    for p in rectified_points:
        x = p[0,0]
        y = p[0,1]



        in_mask = (mask[int(x),int(y),0] > 0)

        if not isnan(x) and not isnan(y) and in_mask:
            new_point = intersection(laser_plane,(x,y), newcamera)
            xp, yp, zp = new_point[0]
            dist = sqrt(xp*xp+yp*yp+zp*zp)
            if zp < 5.0 and dist < 10.0:
                new_point = np.matmul(table_2_to_camera[:3,:3],  new_point.T) + table_2_to_camera[:3,3:]
                laser_points=np.concatenate((laser_points,new_point.T))
                show_img = cv2.circle(show_img,(int(y),int(x)),15,(0,0,255),1)

    cv2.imshow('Laser Image', show_img)
    cv2.waitKey(3)

    viewer.append(laser_points, np.identity(4))
    viewer.drawnow()

    table_1to2 = np.matmul(table_1to2, table_increment)
    table_2to1 = np.linalg.inv(table_1to2)

    step=step+1

print "Proceso acabado"
viewer.save()
viewer.run()
#else:
#    cv2.destroyAllWindows()





