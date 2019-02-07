import cv2
import numpy as np
import serial
import config
import time

def avanti():
    global commandSent

    if commandSent != "avanti":
        arduino.write(b'w')
        commandSent="avanti"
        

def stops():
    global commandSent
    
    if commandSent != "stops":
        arduino.write(b's')
        commandSent="stops"

def indietro():
    global commandSent
    
    if commandSent != "indietro":
        arduino.write(b'x')
        commandSent="indietro"
    
def destra():
    global commandSent

    if commandSent != "destra":
        arduino.write(b'd')
        commandSent="destra"
    
def sinistra():
    global commandSent

    if commandSent != "sinistra":
        arduino.write(b'a')
        commandSent="sinstra"
    
def trackedObjectDirection(cap):

    attributes = {"left": False, "right": False, "middle": False}  #dictionary that contain the value to return

    _, frame = cap.read()
    frame=cv2.resize(frame,(widthScreen,heightScreen))   #image resize
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

    if len(conts) > 0:
        x,y,w,h=cv2.boundingRect(conts[0]) #this method return the x,y, weight and height
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255), 2) #this method draws the border of the tracked object
        cv2.putText(frame, str(0+1),(x,y+h),font,2.,(0,255,255)) #this method gives a number to the tracked objects
        xMiddleRect=x+w/2
        yMiddleRect=y+h/2

        if xMiddleRect <= int(widthScreen * (1-central_zone)*0.5) :    #conditions to moves the robot
           
            attributes = {"left": True, "right": False, "middle": False}
        
        elif xMiddleRect >= int(widthScreen * (1 + central_zone)*0.5): 
            
            attributes = {"left": False, "right": True, "middle": False}
        
        elif xMiddleRect > int(widthScreen * (1-central_zone)*0.5) and xMiddleRect < int(widthScreen * (1 + central_zone)*0.5):   
            
            attributes = {"left": False, "right": False, "middle": True}
       

    return attributes,frame

cap = cv2.VideoCapture(0)           #camera initialization 

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
font=cv2.FONT_HERSHEY_SIMPLEX

widthScreen= config.widthScreen      #screen size
heightScreen= config.heightScreen
central_zone = config.central_zone 

sensitivity = config.sensitivity    #color track settings
lower_red_0 = np.array([0, 100, 100]) 
upper_red_0 = np.array([sensitivity, 255, 255])
lower_red_1 = np.array([180 - sensitivity, 100, 100]) 
upper_red_1 = np.array([180, 255, 255])



while(1):
    result,frame = trackedObjectDirection(cap)

    commandSent = None
    
    print(result)
    #time.sleep(0.5)

    cv2.imshow('frame',frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break