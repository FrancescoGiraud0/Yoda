
import sys
import serial
import numpy as np
import cv2
import time

isConnected = True

try:
    arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=50)
    time.sleep(1)
except:
    isConnected = False
    print("arduino non connesso")

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


while True:
    
    cmd = input("comando: ")
    
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
    