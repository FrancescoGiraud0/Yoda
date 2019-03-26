#YODA (Yoda Object Detector and Avoider)
#Python3 source code
#Authors (@github):
# -Francesco Giraudo (@FrancescoGiraud0)
# -Matteo Allemandi (@MatteoAllemandi)
# -Nicolò Guerra (@NikoWar)
# -Gabriele Parola (@GoshuSan)
#GitHub: https://github.com/FrancescoGiraud0/Yoda

#Remember to:
#- upload motorControllerYoda.ino to Arduino
#- check the name of Arduino and RPLidar USB ports in config.py

import time
import sys
import serial
import numpy as np
import cv2
from rplidar import RPLidar
import config

"""
Functions to communicate to Arduino.
"""

#Send "go forward" signal to Arduino
def goForward():
    arduino.write(b'w')

#Send "stop" signal to Arduino
def stops():
    arduino.write(b's')

#Send "go backwards" signal to Arduino
def goBackwards():
    arduino.write(b'x')

#Send "turn right" signal to Arduino
def turnRight():
    arduino.write(b'd')

#Send "turn left" signal to Arduino
def turnLeft():
    arduino.write(b'a')

def sendCommand(cmd):
    if(cmd == 'w' ):
        goForward()
    elif(cmd == 'a'):
        turnLeft()
    elif(cmd == 's' ):
        stops()
    elif(cmd=='d'):
        turnRight()
    elif(cmd=='x'):
        goBackwards()
    else:
        stops()
        return 's'
    return cmd

def trackedObjectPosition(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_0 = cv2.inRange(hsv, lower_red_0 , upper_red_0)
    mask_1 = cv2.inRange(hsv, lower_red_1 , upper_red_1 )

    mask = cv2.bitwise_or(mask_0, mask_1)

    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose

    #This function returns the image, borders, and the height's array
    im,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) 

    #Draw border of the tracked object on frame
    cv2.drawContours(frame,conts,-1,(255,0,0),3)

    xMiddleRect, yMiddleRect = None, None
   
    if len(conts) > 0:
        #This method return the x,y, weight and height
        x,y,w,h=cv2.boundingRect(conts[0])
        #Draw a rectangle around the tracked object
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255), 2)

        #Coordinates of the center of the tracked object
        xMiddleRect=x+w/2
        yMiddleRect=y+h/2

    return xMiddleRect,yMiddleRect

def isFull(measurments_list):
    for measure in measurments_list:
        if measure <= -1:
            return False
    return True

def appendNewMeasure(measurment, measurments_list):
    measure_bool, measure_power, measure_angle, measure_distance = measurment

    if(measure_power>=config.QUALITY):
        if measurments_list[int(measure_angle//config.STEP_ANGLE)] == -1:
            measurments_list[int(measure_angle//config.STEP_ANGLE)] = measure_distance
    
    return measurments_list

def main():
    #create a list of -1
    measurments_list = [-1 for i in range(0,360//config.STEP_ANGLE)]
    commandSent = 's'

    for measurment in lidar.iter_measurments(max_buf_meas = config.MAX_BUF_MEAS):
        #append the measure in the measurment_list if it is in a angle range not yet measured
        measurments_list = appendNewMeasure(measurment, measurments_list)
        commandSent = 's'
        print(measurments_list)
        if isFull(measurments_list):
            print("sono arrivato qui 1")
            _, frame = cap.read()
            #Image
            frame = cv2.resize(frame,(config.WIDTH_SCREEN,config.HEIGHT_SCREEN))
            #Center of the tracked object in the frame
            trackedObjectPosX, trackedObjectPosY = trackedObjectPosition(frame)
            #Show the camera frame
            cv2.imshow("YodaCamera",frame)
            #Non blocking input from keyboard
            key = cv2.waitKey(5) & 0xFF
            #if pressed ESC
            if key==27:
                break
            elif key>=ord('a') and key<=ord('z'):
                commandSent = sendCommand(chr(key))
            else:
                sendCommand(commandSent)
            #Funzione per salvare in file csv

            #Reset the list for new measurerment
            measurments_list = [-1 for i in range(0,360//config.STEP_ANGLE)]

    #cap.release()
    #Close output window
    cv2.destroyAllWindows()
    #Stop RPLidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    #Send signal to stop motor to Arduino
    stops()

#For serial comunication with Arduino
arduino = serial.Serial(config.ARDUINO_PORT_NAME ,9600)

#Lidar usb comunication port
lidar = RPLidar(config.LIDAR_PORT_NAME)

#Define the camera
cap = cv2.VideoCapture(0)

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
font=cv2.FONT_HERSHEY_SIMPLEX

#HSV color range defining(RED)
lower_red_0 = np.array([0, 100, 100])
upper_red_0 = np.array([config.SENSITIVITY, 255, 255])
lower_red_1 = np.array([180 - config.SENSITIVITY, 100, 100])
upper_red_1 = np.array([180, 255, 255])

main()
