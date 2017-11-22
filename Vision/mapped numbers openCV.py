import cv2
import numpy as np
grid=np.zeros((1000,1000,3),dtype=np.uint8) # an empty grid to test on
number=0 #base number for later calculations
base=36 #doron and i got bored so we decided on a multi-base functionality
def toStr(n,base):
    """
    :param n: the number in decimal base
    :param base: the wanted base
    :return: a string representing the desired number in the chosen base

    """
    convertString = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    if n < base:
      return convertString[n]
    else:
        return toStr(n//base,base) + convertString[n%base]
while True:
    grid=np.zeros((1000,1000,3),dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(grid,toStr(number,base),(10,500), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow('grid',grid)
    key=cv2.waitKey(1)
    if key in range(48,48+base) and key<58:#48-58 are the possible numbers
        number *= base
        number += key-48
    if key in range(97,97+base-10) and base > 9: #every character from A-Z if the base allows
        number*=base
        number+=key-87
    if key is 13: #13 is Enter
        print(number)
        number=0
    if key is 8: #8 is backspace
        number/=base
        number=int(number)
    if key is ord('.'):
        break
