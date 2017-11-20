import cv2
import numpy as np
cam = cv2.VideoCapture(0)
minH=255
minS=255
minV=255
maxH=0
maxS=0
maxV=0

while True:
    _, frame=cam.read()
    cv2.imshow("frame",frame)
    key= cv2.waitKey(1) & 0xFF
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if key == ord('q'):
        for row in hsv:
            for pixel in row:
                if pixel[0] < minH:
                    minH=pixel[0]
                if pixel[1] < minS:
                    minS=pixel[1]
                if pixel[2] < minV:
                    minV=pixel[2]
                if pixel[0] > maxH:
                    maxH=pixel[0]
                if pixel[1] > maxS:
                    maxS=pixel[1]
                if pixel[2] > maxV:
                    maxV=pixel[2]
        file=open("Ace.acpf",'w')
        file.write( "self.lower_range,self.upper_range = ({},{},{}),({},{},{})".format(minH,minS,minV,maxH,maxS,maxV))
        file.close()
        break
