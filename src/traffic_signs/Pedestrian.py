import cv2
import numpy as np

def pedestrian(img):
    pedestrian = False
    dimensions = img.shape
    h = int(dimensions[0])
    w = int(dimensions[1])
<<<<<<< HEAD
    img = img[int(h/2.5): int(h/1.4) , int(w/4):int(w/1.5)]
=======
    img = img[int(h/5): int(h/1.3) , int(w/4):int(w/1.5)]
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    #------PINK MASK--------
    #lower = np.array([157,73,156]) 
    #upper = np.array([179,255,255])
    lower = np.array([133,80,91]) 
    upper = np.array([179,255,187])

    pink_mask = cv2.inRange(imgHSV,lower,upper)
    pink_imgResult = cv2.bitwise_and(img,img,mask=pink_mask)
<<<<<<< HEAD
=======
    cv2.imwrite("ped.jpg",pink_imgResult)
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
    #print(imgResult.size)
    #print(pink_imgResult.size,"pink")

    #detect color
    color ="None"
    number_of_black_pix = np.sum(pink_imgResult == 0)
<<<<<<< HEAD
    if number_of_black_pix<pink_imgResult.size-20000:
=======
    if number_of_black_pix<pink_imgResult.size-14000:
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
        color ="pink"

    if(color=="pink"):
        pedestrian = True
    
    return pedestrian
