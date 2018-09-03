#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import QtGui, QtCore
from subprocess import Popen
import GetData
import test_center_Aruco
import numpy as np
import cv2

import subprocess


class Window(QtGui.QMainWindow):

    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(50, 50, 500, 300)
        self.setWindowTitle("Laser Scanner")
        self.setStyleSheet("background-color: white")
        self.Buttons()

       

    def Buttons(self):
        btn = QtGui.QPushButton("Exit", self)
        btn.clicked.connect(self.close_application)
        btn.setStyleSheet("background-color: red")
        btn.resize(btn.minimumSizeHint())
        btn.move(425,0)
        
        btn1 = QtGui.QPushButton("Camera Calibration",self)
        btn1.resize(100,50)
        btn1.move(50,50)
        btn1.setStyleSheet("background-color: #80d4ff")
        btn1.clicked.connect(self.CameraCalib)
        
        btn2 = QtGui.QPushButton("Laser Calibration",self)
        btn2.resize(100,50)
        btn2.move(50,120)
        btn2.setStyleSheet("background-color: #3399ff")
        btn2.clicked.connect(self.LaserCalib)
        
        btn4 = QtGui.QPushButton("Table detection",self)
        btn4.resize(100,50)
        btn4.move(50,190)
        btn4.setStyleSheet("background-color: #0066ff")
        btn4.clicked.connect(self.Aruco)
        
        btn4 = QtGui.QPushButton("Get Data",self)
        btn4.resize(100,50)
        btn4.move(200,120)
        btn4.setStyleSheet("background-color: #ff6600")
        btn4.clicked.connect(self.GetData)
        
        btn5 = QtGui.QPushButton("Preview Cam",self)
        btn5.resize(100,50)
        btn5.move(200,50)
        btn5.setStyleSheet("background-color: orange")
        btn5.clicked.connect(self.Preview)
        
        btn5 = QtGui.QPushButton("Save Image",self)
        btn5.resize(100,50)
        btn5.move(200,190)
        btn5.setStyleSheet("background-color: orange")
        btn5.clicked.connect(self.Preview)
        
        btn3 = QtGui.QPushButton("Offline Process",self)
        btn3.resize(100,50)
        btn3.move(350,50)
        btn3.setStyleSheet("background-color: #bbff99")
        btn3.clicked.connect(self.Start)
        
        btn6 = QtGui.QPushButton("Online Process",self)
        btn6.resize(100,50)
        btn6.move(350,120)
        btn6.setStyleSheet("background-color: #66ff33")
        btn6.clicked.connect(self.Start)
        
        self.show()
    
    def close_application(self):
        choice = QtGui.QMessageBox.question(self, 'Exit',
                                            "Are you sure?",
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        if choice == QtGui.QMessageBox.Yes:
            sys.exit()
        else:
            pass

        
    def CameraCalib(self):
        print 'CameraCalibration'
        
        params={"-d", "images\camera_calibration\240818", "-f", "jpg","-o", "calibrations\240818"}
        p = subprocess.Popen("python calibrate_camera.py")
        stdout, stderr = p.communicate()
        print p.returncode # is 0 if success
        
#        p = Popen("run_calibrate_camera.bat", cwd=r"C:\Users\Propietario\Documents\GitHub\3d_laser_marta")
#        stdout, stderr = p.communicate()
    def LaserCalib(self):
        print 'LaserCalibration'
    def Start(self):
        print 'Scanning'
    def GetData(self):
        GetData.run()        
    def Aruco(self):
        test_center_Aruco.run()
        
    def Preview(self):
        cap = cv2.VideoCapture(0)

        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()
            # Display the resulting frame
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('e'):
                break
            
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()
        
    def Capture(self):
        cap = cv2.VideoCapture(0)

        # Capture frame-by-frame
        ret, frame = cap.read()
        # Display the resulting frame
        cv2.imshow('frame',frame)
        write_name = './images/Image.jpg'
        cv2.imwrite(write_name, frame)    
            
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()
        
        

        
def run():
    app = QtGui.QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())

run()