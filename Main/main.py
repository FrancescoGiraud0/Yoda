import time
import sys
import serial
import random
import numpy as np
import cv2
from rplidar import RPLidar
import config

arduino = serial.Serial(config.ARDUINO_PORT_NAME ,9600)

def update_obstacles(obstacles, ob):
    for key in ob.keys():
        if ob[key] == True:
            obstacles[key] = True
    return obstacles

def find_obstacles(measurements_list):

    ret_dict= {"left": False, "center": False, "right": False}

    for measure_tuple in measurements_list:

        ob_dict = {"left": False, "center": False, "right": False}

        measure_bool, measure_power, measure_angle, measure_distance = measure_tuple

        if measure_distance <= config.MAX_DISTANCE and measure_distance >= config.MIN_DISTANCE and measure_power >= config.QUALITY:
            #right
            if measure_angle > config.RIGHT / 2 and measure_angle <= config.RIGHT:
                ob_dict["right"] = True
            #left
            elif measure_angle >= config.LEFT and measure_angle < (360 -(config.RIGHT/2)):
                ob_dict["left"] = True
            #center
            elif measure_angle >= config.CENTER and measure_angle <= config.RIGHT/2:
                ob_dict["center"] = True
            #center
            elif measure_angle >= (360 -(config.RIGHT/2)) and measure_angle <= 360:
                ob_dict["center"] = True

            ret_dict = update_obstacles(ret_dict, ob_dict)

    return ret_dict

def avanti():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'w')
        timeCommand = time.clock()

def stops():

    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b's')
        timeCommand = time.clock()

def indietro():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'x')
        timeCommand = time.clock()

def destra():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'd')
        timeCommand = time.clock()

def sinistra():
    global timeCommand

    if time.clock() - timeCommand > config.FREQ_COMMANDS:
        arduino.write(b'a')
        timeCommand = time.clock()

def trackedObjectDirection(frame, obstacles):
    attributes = {"left": False, "right": False, "center": False}  #dictionary that contain the value to return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_0 = cv2.inRange(hsv, lower_red_0 , upper_red_0)
    mask_1 = cv2.inRange(hsv, lower_red_1 , upper_red_1 )

    mask = cv2.bitwise_or(mask_0, mask_1)

    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose

    im,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #this function returns the image, boarders, and the height's array

    cv2.drawContours(frame,conts,-1,(255,0,0),3)

    cv2.line(frame,(int(widthScreen * (1-central_zone)*0.5),0),(int(widthScreen * (1-central_zone)*0.5),220),(255,0,0),2)
    cv2.line(frame,(int(widthScreen * (1 + central_zone)*0.5),0),(int(widthScreen * (1 + central_zone)*0.5),220),(255,0,0),2)

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

    if len(conts) > 0:
        x,y,w,h=cv2.boundingRect(conts[0]) #this method return the x,y, weight and height
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255), 2) #this method draws the border of the tracked object
        cv2.putText(frame, str(0+1),(x,y+h),font,2.,(0,255,255)) #this method gives a number to the tracked objects
        xMiddleRect=x+w/2
        yMiddleRect=y+h/2

        if xMiddleRect <= int(widthScreen * (1-central_zone)*0.5) :    #conditions to moves the robot
            attributes = {"left": True, "center": False, "right": False}
        elif xMiddleRect >= int(widthScreen * (1 + central_zone)*0.5):
            attributes = {"left": False, "center": False, "right": True}
        elif xMiddleRect > int(widthScreen * (1-central_zone)*0.5) and xMiddleRect < int(widthScreen * (1 + central_zone)*0.5):
            attributes = {"left": False, "center": True,"right": False}

    return attributes

#Generate a string from a dictionary bitLeft+bitCenter+bitRight
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

def motors_controller(obstacles, results):
        commandsTable[results][obstacles]

def main():
    lidar = RPLidar(config.LIDAR_PORT_NAME)
    time.sleep(5)
    measurments_list = []
    obstacles = {"left": False, "center": False, "right": False}

    commandSent = " "

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

    cv2.destroyAllWindows()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    stops()

commandsTable = { "000":
                    { "000":stops, "001":stops,"010":stops, "011":stops,
                      "100":stops, "101":stops,"110":stops, "111":stops },
                  "001":
                    { "000":destra, "001":stops,"010":destra, "011":stops,
                      "100":destra, "101":stops,"110":destra, "111":stops },
                  "010":
                    { "000":avanti, "001":avanti,"010":stops, "011":stops,
                      "100":avanti, "101":avanti,"110":stops, "111":stops },
                  "100":
                    { "000":sinistra, "001":sinistra,"010":sinistra, "011":sinistra,
                      "100":stops, "101":stops,"110":stops, "111":stops }
                }

timeCommand = 0.0

cap = cv2.VideoCapture(0)

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
font=cv2.FONT_HERSHEY_SIMPLEX

sensitivity = config.sensitivity    #color track settings

lower_red_0 = np.array([0, 100, 100])
upper_red_0 = np.array([sensitivity, 255, 255])
lower_red_1 = np.array([180 - sensitivity, 100, 100])
upper_red_1 = np.array([180, 255, 255])
widthScreen= config.widthScreen      #screen size
heightScreen= config.heightScreen
central_zone = config.central_zone

main()