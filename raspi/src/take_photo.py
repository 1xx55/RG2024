import cv2
import sys
#引入库

cap = cv2.VideoCapture("/dev/r_video") #right
cap2 = cv2.VideoCapture("/dev/l_video")#left


i = 0
while True:
    i=i+1
    ret, frame = cap.read()
    ret2, frame2 = cap2.read()
    
    if (ret2):
        print(1)
        break
    else :
        print(0)
        if i==100:
            break
#随时准备按q退出
cap.release()
#关掉所有窗口
cv2.destroyAllWindows()
