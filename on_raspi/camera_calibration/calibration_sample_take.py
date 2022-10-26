import cv2
import os
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS,60)


file_name = 0

if "simple" not in os.listdir():
    os.mkdir("simple")

while(1):
    ret,frame = cap.read()
    
    cv2.imshow("frame",frame)
    
    K = cv2.waitKey(1)
    if(K == 32):
        cv2.imwrite('simple/simple_' + str(file_name) + ".jpg",frame)
        file_name += 1
    elif(K == 27):
        break

cv2.destroyAllWindows()
