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
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'w')
        timeCommand = time.clock()

#Send "stop" signal to Arduino
def stops():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b's')
        timeCommand = time.clock()

#Send "go backwards" signal to Arduino
def goBackwards():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'x')
        timeCommand = time.clock()

#Send "turn right" signal to Arduino
def turnRight():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'd')
        timeCommand = time.clock()

#Send "turn left" signal to Arduino
def turnLeft():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'a')
        timeCommand = time.clock()

"""
Functions to define where the obstacles are with RPLidar.
"""

#This function return a dictionary of boolean (0 = no obstacles, 1 = obstacles)
#of where obstacle are
def find_obstacles(measurements_list):
    #Initialize return dictionary
    ret_dict= {"left": False, "center": False, "right": False}

    for measure_tuple in measurements_list:
        #Define a temporary dictionary to save obstacle in measure_angle
        ob_dict = {"left": False, "center": False, "right": False}

        measure_bool, measure_power, measure_angle, measure_distance = measure_tuple

        #If there is an object in distance range and the signal is clean
        if measure_distance <= config.MAX_DISTANCE and measure_distance >= config.MIN_DISTANCE and measure_power >= config.QUALITY:
            #if the object is in right range
            if measure_angle > config.RIGHT / 2 and measure_angle <= config.RIGHT:
                ob_dict["right"] = True
            #if the object is in left range
            elif measure_angle >= config.LEFT and measure_angle < (360 -(config.RIGHT/2)):
                ob_dict["left"] = True
            #if the object is in center range
            elif measure_angle >= config.CENTER and measure_angle <= config.RIGHT/2:
                ob_dict["center"] = True
            #if the object is in center range
            elif measure_angle >= (360 -(config.RIGHT/2)) and measure_angle <= 360:
                ob_dict["center"] = True

            #update the new obstacle in ret_dict
            ret_dict = update_obstacles(ret_dict, ob_dict)

    return ret_dict

#Function for update any new obstacle detected in angle in range
def update_obstacles(obstacles, ob):
    for key in ob.keys():
        if ob[key] == True:
            obstacles[key] = True
    return obstacles


"""
Function to define where the tracked object is (right, center or left),
draw contour of the tracked object and draw where the obstacles are in
the frame.
"""

#This function return a dictionary of boolean (0 = no object, 1 = object)
#of where the trcked object is.
def trackedObjectDirection(frame, obstacles):
    #Initialize return dictionary
    attributes = {"left": False, "right": False, "center": False}

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

    #Draw range line on frame
    cv2.line(frame,(int(widthScreen * (1-central_zone)*0.5),0),(int(widthScreen * (1-central_zone)*0.5),220),(255,0,0),2)
    cv2.line(frame,(int(widthScreen * (1 + central_zone)*0.5),0),(int(widthScreen * (1 + central_zone)*0.5),220),(255,0,0),2)

    #Draw "X" on ranges where are obstacles
    for direction, boolean in obstacles.items():
        if direction == "left" and boolean:
            cv2.line(frame, (0, heightScreen), (int(widthScreen/3),0), (0,0,255), 2)
            cv2.line(frame, (0,0), (int(widthScreen/3), heightScreen), (0,0,255), 2)
        elif direction == "center" and boolean:
            cv2.line(frame, (int(widthScreen/3),heightScreen), (int(widthScreen/3)*2, 0), (0,0,255), 2)
            cv2.line(frame, (int(widthScreen/3),0), (int(widthScreen/3)*2, heightScreen), (0,0,255), 2)
        elif direction == "right" and boolean:
            cv2.line(frame, (int(widthScreen/3)*2,heightScreen),(widthScreen, 0), (0,0,255), 2)
            cv2.line(frame, (int(widthScreen/3)*2,0), (widthScreen, heightScreen), (0,0,255), 2)

    #If there is one or more object 
    if len(conts) > 0:
        #This method return the x,y, weight and height
        x,y,w,h=cv2.boundingRect(conts[0])
        #Draw a rectangle around the tracked object
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255), 2)
        #This method gives a number to the tracked objects
        cv2.putText(frame, str(0+1),(x,y+h),font,2.,(0,255,255))

        #Coordinates of the center of the tracked object
        xMiddleRect=x+w/2
        yMiddleRect=y+h/2

        #Define where the conter of the object is
        if xMiddleRect <= int(widthScreen * (1-central_zone)*0.5) :
            attributes = {"left": True, "center": False, "right": False}
        elif xMiddleRect >= int(widthScreen * (1 + central_zone)*0.5):
            attributes = {"left": False, "center": False, "right": True}
        elif xMiddleRect > int(widthScreen * (1-central_zone)*0.5) and xMiddleRect < int(widthScreen * (1 + central_zone)*0.5):
            attributes = {"left": False, "center": True,"right": False}

    return attributes


"""
This function generate a string of 0 and 1 from a dictionary ("bitLeft+bitCenter+bitRight").
"""

def convert(dictionary):
    retString = ""

    if dictionary["left"] == True :
        retString+="1"
    else :
        retString+="0"

    if dictionary["center"] == True :
        retString+="1"
    else :
        retString+="0"

    if dictionary["right"] == True :
        retString+="1"
    else :
        retString+="0"

    return retString

def main():
    lidar = RPLidar(config.LIDAR_PORT_NAME)
    time.sleep(5)
    measurments_list = []
    obstacles = {"left": False, "center": False, "right": False}

    for measurment in lidar.iter_measurments(max_buf_meas = config.MAX_BUF_MEAS):
        measurments_list.append(measurment)
        
        if len(measurments_list) >= config.NUMBER_MEASURE:
            _, frame = cap.read()
            frame = cv2.resize(frame,(widthScreen,heightScreen))   #image
            obstacles = find_obstacles(measurments_list)
            results = trackedObjectDirection(frame, obstacles)
            commandsTable[convert(results)][convert(obstacles)]()
            measurments_list.clear()
            cv2.imshow("camera",frame)
            k = cv2.waitKey(5) & 0xFF
            if k==27:
                break

    ###release the capture###
    #cap.release()
    #Close output window
    cv2.destroyAllWindows()
    #Stop RPLidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    #send signal to stop motor to Arduino
    ### time.sleep() di config.FREQ_COMMANDS in modo da evitare che non sia inviato###
    #time.sleep(config.FREQ_COMMANDS)
    stops()


#Using this dictionary of dictionary it is possible to define what the robot must do based
#on the processed data (where is object and where are obstacles) similar to a truth table
commandsTable = { "000":
                    { "000":stops, "001":stops,"010":stops, "011":stops,
                      "100":stops, "101":stops,"110":stops, "111":stops },
                  "001":
                    { "000":turnRight, "001":stops,"010":turnRight, "011":stops,
                      "100":turnRight, "101":stops,"110":turnRight, "111":stops },
                  "010":
                    { "000":goForward, "001":goForward,"010":stops, "011":stops,
                      "100":goForward, "101":goForward,"110":stops, "111":stops },
                  "100":
                    { "000":turnLeft, "001":turnLeft,"010":turnLeft, "011":turnLeft,
                      "100":stops, "101":stops,"110":stops, "111":stops }
                }

#For serial communication with Arduino
arduino = serial.Serial(config.ARDUINO_PORT_NAME ,9600)

#Define the cam
cap = cv2.VideoCapture(0)

#Initialize timer used to prevent the bombardment in serial communication with Arduino
timeCommand = 0.0

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
font=cv2.FONT_HERSHEY_SIMPLEX

sensitivity = config.sensitivity    #color track settings

#HSV color range defining(RED)
lower_red_0 = np.array([0, 100, 100])
upper_red_0 = np.array([sensitivity, 255, 255])
lower_red_1 = np.array([180 - sensitivity, 100, 100])
upper_red_1 = np.array([180, 255, 255])

#Output window size
widthScreen= config.widthScreen
heightScreen= config.heightScreen

#Central zone defining used in trackedObjectDirection() function
central_zone = config.central_zone

main()
