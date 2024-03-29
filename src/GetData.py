# -*- coding: utf-8 -*-
"""
Created on Tue Jul 17 12:22:55 2018

@author: Propietario
"""
import time
import serial
import cv2


ser = serial.Serial()
ser.port = "COM12"
ser.baudrate = 9600
ser.timeout = 1 

cap = cv2.VideoCapture(0)


try: 
    ser.open()
    print ser.portstr       # check which port was really used
except Exception, e:
    print "error open serial port: " + str(e)
    exit()

if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                             #and discard all that is in buffer
        response = ser.readline()
        print("read data: " + response) #Check information from Arduino
        time.sleep(0.5)
        step=1
        
        while step<4:
            #Perform a step    
            ser.write("1") 
            time.sleep(1)
            
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            if ret==True:
                step=step+1
                write_name = './images/dataset/040918/step_num'+str(step)+'.jpg'
#                write_name = '.C:\Users\Propietario\Documents\GitHub\3d_laser_marta\images\dataset\040918\step_num_table'+str(step)+'.jpg'
                print step
                cv2.imwrite(write_name, frame)
    except Exception, e1:
        print "error communicating...: " + str(e1)
else:
    print "cannot open serial port "
ser.close()
cap.release()
cv2.destroyAllWindows()