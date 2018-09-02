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

def invHom(m):
    R = m[0:3,0:3]
    t = m[0:3,3]

    minv = np.zeros((4,4))
    minv[0:3,0:3] = R.T
    minv[0:3,3] = np.matmul(R.T,-t)

    return minv

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

def calculate_rotation_priority(frame, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
    marker_length = 0.03 #in meters
            
    if ids is not None:

        max_y = 0
        max_y_idx = 0
        for idx, marker in enumerate(corners):
            for corner in marker[0]:
                if corner[1] > max_y:
                    max_y = corner[1]
                    max_y_idx = idx

        cv2.aruco.drawDetectedMarkers(frame, corners,ids)
        
        i = max_y_idx
        
        rvecs,tvecs,_objPoints  = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, dist_coeffs)
        cv2.aruco.drawAxis(frame,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
        R, _ = cv2.Rodrigues(rvecs)
        if ids[i][0]==5:
            t = np.array([-0.09,0,0])
            tvecs=np.matmul(R,t.T)+tvecs
        elif ids[i][0]==42:
            t = np.array([+0.09,0,0])
            tvecs=np.matmul(R,t.T)+tvecs
        elif ids[i][0]==27:
            t = np.array([0,-0.09,0])
            tvecs=np.matmul(R,t.T)+tvecs
        elif ids[i][0]==18:
            t = np.array([0,+0.09,0])
            tvecs=np.matmul(R,t.T)+tvecs
            # i=i+1
        else:
            print "Marker "+str(ids[i])+" NOT found!!!"

        cv2.aruco.drawAxis(frame,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
        cv2.imshow('Tabe Calib',frame)
#        write_name = 'CalibracioArucoPriority.jpg'
#        cv2.imwrite(write_name, frame)
        cv2.waitKey(3)
    return (R,tvecs)

def calculate_rotation_all(frame, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
    marker_length = 0.03 #in meters

    tvecs5=np.array([[[0,0,0]]])
    tvecs42=np.array([[[0,0,0]]])
    tvecs27=np.array([[[0,0,0]]])
    tvecs18=np.array([[[0,0,0]]])
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame,corners,ids)
        i=0
        while(i<len(ids)):
            rvecs,tvecs,_objPoints  = cv2.aruco.estimatePoseSingleMarkers(corners[i],marker_length,camera_matrix,dist_coeffs)
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
              
        tvecs = (tvecs5+tvecs42+tvecs27+tvecs18)/len(ids)
            
        cv2.aruco.drawAxis(frame,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
        cv2.imshow('Tabe Calib',frame)
#        write_name = 'CalibracioArucoAll.jpg'
#        cv2.imwrite(write_name, frame)
        cv2.waitKey(3)

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
ap.add_argument("-o", "--output", required=True,
    help="Output prefix of the calibration result")
ap.add_argument("-t", "--table", required=True,
    help="Output prefix of the calibration result")
args = vars(ap.parse_args())


#------------------------------MAIN--------------------------------------------
camera_matrix = np.load(args['calibration'])
dist_coeffs = np.load(args['distortion'])
laser_plane = np.load(args['laser'])
option_table = args['table']

num_points = 0
num_raw_points = 0

print('Camera calibration:')
print camera_matrix
print('Camera distortion:')
print dist_coeffs
print('Laser calibration:')
print laser_plane

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32)
threshold = 0.02

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
table_to_rot = np.identity(4)
rot_to_table = np.identity(4)
camera_to_table_1 = np.identity(4)
camera_to_table_2 = np.identity(4)
table_1_to_camera = np.identity(4)
table_2_to_camera = np.identity(4)

mask = cv2.imread('calibrations/mask2.png')
points = np.array([[],[],[]]).T
step=1
while step<321:
    # Detect table
    if step < 2:
        fname = args['dir'] + '/step_num_table' + str(step) + '.' + args['format']
#        fname = args['dir'] + '/step_num' + str(step) + '.' + args['format']
        frameTable = cv2.imread(fname)
        newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, frameTable.shape[:2], 0)
        frameTable_rect = cv2.undistort(frameTable, camera_matrix, dist_coeffs, None, newcamera)
        if option_table == '1':
            print ('Table calibration with all markers')
            R, tvec = calculate_rotation_all(frameTable_rect, newcamera, 0)
        if option_table == '2':
            print ('Table calibration with priority')
            R, tvec = calculate_rotation_priority(frameTable_rect, newcamera, 0)
        camera_to_table_1[0:3,0:3] = R
        camera_to_table_1[0:3,3] = tvec
        print 'Table transform: \n' + str(camera_to_table_1)

    table_1_to_camera = invHom(camera_to_table_1)
    camera_to_table_2 = np.matmul(table_to_rot, camera_to_table_1)
    table_2_to_camera = invHom(camera_to_table_2)
    cam_to_cam2 = np.matmul((np.matmul(camera_to_table_1,table_to_rot)), table_1_to_camera)
    cam2_to_cam = invHom(cam_to_cam2)
    table_to_rot = np.matmul(table_to_rot, table_increment)
    rot_to_table = invHom(table_to_rot)


    # undistort
    fname = args['dir'] + '/step_num' + str(step) + '.' + args['format']
    frame = cv2.imread(fname)
    newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, frame.shape[:2], 0)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcamera)
    show_img = copy.deepcopy(frame)
    subpixel_peaks = detect_laser_subpixel(frame, kernel, threshold, window_size)
    laser_points = np.array([[],[],[]]).T
    raw_points = np.array([[],[],[]]).T

    for p in subpixel_peaks:
        row = p[0]
        col = p[1]
        if not isnan(row) and not isnan(col):
            in_mask = (mask[int(row),int(col),0] > 0)
            if not isnan(row) and not isnan(col) and in_mask:
                new_point = intersection(laser_plane, (col,row), newcamera)
                xp, yp, zp = new_point[0]
                new_point[0] = xp, yp, zp
                dist = sqrt(xp*xp+yp*yp+zp*zp)
                raw_points=np.concatenate((raw_points,new_point))
                num_raw_points = num_raw_points+1
                if zp < 5.0 and dist < 10.0:
                    new_point = np.matmul(cam_to_cam2[:3,:3],  new_point.T) + cam_to_cam2[:3,3:]
                    num_points = num_points+1
    #                file.write(str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2])+'\n')
                    laser_points=np.concatenate((laser_points,new_point.T))
                    points=np.concatenate((points,new_point.T))
                    show_img = cv2.circle(show_img,(int(col),int(row)),15,(0,0,255),1)
    cv2.imshow('Laser Image', show_img)
    cv2.waitKey(3)
    viewer.append(laser_points, np.identity(4))
    viewer.drawnow()
    step = step + 1

file = open("model3.ply","w")
file.write('ply\n')
file.write('format ascii 1.0\n')
file.write('element vertex ' + str(num_points) + '\n')
file.write('property float x\n')
file.write('property float y\n')
file.write('property float z\n')
file.write('end_header\n')
for p in points:
    file.write(str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2])+'\n')
file.close()
print "Proceso acabado"
np.savetxt('reconstruct_points.csv', laser_points, delimiter=' ')   # X is an array
viewer.save()
viewer.run()
#else:
#    cv2.destroyAllWindows()





