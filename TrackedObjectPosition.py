def TrackedObjectPosition():

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

    return xMiddleRect,yMiddleRect
