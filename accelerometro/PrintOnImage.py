import cv2 as cv

'''
FONT_HERSHEY_SIMPLEX = 0,
FONT_HERSHEY_PLAIN = 1,
FONT_HERSHEY_DUPLEX = 2,
FONT_HERSHEY_COMPLEX = 3,
FONT_HERSHEY_TRIPLEX = 4,
FONT_HERSHEY_COMPLEX_SMALL = 5,
FONT_HERSHEY_SCRIPT_SIMPLEX = 6,
FONT_HERSHEY_SCRIPT_COMPLEX = 7

lineType = cv.LINE_AA
'''

def PrintAcceleration(frame,accel,x,y,font,dimension,fontWidth,colors,lineType):
    R = color[0]
    G = color[1]
    B = color[2]

    cv.putText(frame,str(accel),(x,y),font,dimension,(R,G,B),fontWidth,lineType)
    #cv.putText(img, 'This one!", (230, 50), font, 0.8, (0, 255, 0), 2, cv.LINE_AA)