#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import QtGui, QtCore
from subprocess import Popen


class Window(QtGui.QMainWindow):

    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(50, 50, 500, 300)
        self.setWindowTitle("Laser Scanner")
        self.setWindowIcon(QtGui.QIcon('pythonlogo.png'))
        self.Buttons()

    def Buttons(self):
        btn = QtGui.QPushButton("Exit", self)
        btn.clicked.connect(self.close_application)
        btn.setStyleSheet("background-color: red")
        btn.resize(btn.minimumSizeHint())
        btn.move(0,0)
        
        self.btn1 = QtGui.QPushButton("Camera Calibration",self)
        self.btn1.resize(100,50)
        self.btn1.move(200,50)
        self.btn1.clicked.connect(self.CameraCalib)
        
        self.btn = QtGui.QPushButton("Laser Calibration",self)
        self.btn.resize(100,50)
        self.btn.move(200,120)
        self.btn.clicked.connect(self.LaserCalib)
        
        self.btn = QtGui.QPushButton("Start Process",self)
        self.btn.resize(100,50)
        self.btn.move(200,190)
        self.btn.clicked.connect(self.Start)
        
        
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
        p = Popen("run_calibrate_camera.bat", cwd=r"C:\Users\Propietario\Documents\GitHub\3d_laser_marta\run_calibrate_camera.bat")
        stdout, stderr = p.communicate()
    def LaserCalib(self):
        print 'LaserCalibration'
    def Start(self):
        print 'Scanning'
#        btn.clicked.connect(QtCore.QCoreApplication.instance().quit)
        

        
def run():
    app = QtGui.QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())

run()