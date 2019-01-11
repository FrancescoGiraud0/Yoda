import cv2
import numpy as np
import serial

def destra():
    arduino.write(b'd')

def sinistra():
    arduino.write(b's')

cap = cv2.VideoCapture(2)           #impostazione della webcam da usare
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
font=cv2.FONT_HERSHEY_SIMPLEX
widthScreen=340                     #impostazione dimesione schermo
heightScreen=220

while(1):

    _, frame = cap.read()                               #ricezione immagine dalla webcam
    frame=cv2.resize(frame,(widthScreen,heightScreen))   #ridimensionamento immagine
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
    
    sensitivity = 15                                            #impostazione del colore da seguire
    lower_red_0 = np.array([0, 100, 100]) 
    upper_red_0 = np.array([sensitivity, 255, 255])
    lower_red_1 = np.array([180 - sensitivity, 100, 100]) 
    upper_red_1 = np.array([180, 255, 255])

    mask_0 = cv2.inRange(hsv, lower_red_0 , upper_red_0)
    mask_1 = cv2.inRange(hsv, lower_red_1 , upper_red_1 )

    mask = cv2.bitwise_or(mask_0, mask_1)

    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose

    im,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #restituisce l'immagine, i contorni e le liste delle altezze
    
    cv2.drawContours(frame,conts,-1,(255,0,0),3)

    for i in range(len(conts)):
        x,y,w,h=cv2.boundingRect(conts[0])                           #funzione che calcola il rettangolo per il contorno, restituise x,y,base e altezza
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255), 2)            #comando che disegna un rettangolo dando i due vertici opposti
        cv2.putText(frame, str(0+1),(x,y+h),font,2.,(0,255,255))     #funzione che numera il rettangolo 
        
        xCentroRett=x+w/2   #calcolo della x del centro e la y del centro del rettangolo appena disegnato
        yCentroRett=y+h/2
        
        print(xCentroRett,yCentroRett)

        if xCentroRett <= widthScreen//3 :      #scelta della direzione da seguire in base alla posizione dell'oggetto 
            print('sinistra()')
        elif xCentroRett >= (widthScreen//3)*2:
            print('destra()')
        elif xCentroRett > widthScreen//3 and xCentroRett < (widthScreen//3)*2 :   
            print('centro()')
        else:
            print('fermo()')

    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)       #visualizzazione delle finestre
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
