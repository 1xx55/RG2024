import cv2
import sys
#引入库
import serial
list_i=[2,5]

def open_cameras(n):
    caps = [cv2.VideoCapture(i) for i in range(0,n)]
    # caps = [cv2.VideoCapture(i) for i in list_i]
    
    while True:
        for i, cap in enumerate(caps):
            # if i==0:
            #     cap.set(cv2.CAP_PROP_EXPOSURE,-4)
            # if i==2:
            #     cap.set(cv2.CAP_PROP_EXPOSURE,-5)
            ret, frame = cap.read()
            if ret:
                cv2.namedWindow(str(i), 0)
                cv2.imshow(str(i), frame)
                
        c = cv2.waitKey(1) & 0xff
        if c == ord("q"):
            break
    
    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()
 
open_cameras(9)  # 打开并显示3个摄像头的画面
# serial_index = 'COM11'
# ser = serial.Serial(serial_index, 115200, timeout=0.5)
# print("serial_init_success")
# cap = cv2.VideoCapture(0) #right
# # cap2 = cv2.VideoCapture(3)#left
# cap3 = cv2.VideoCapture(1)#back
# # cap4 = cv2.VideoCapture(4)#back
# # cap5 = cv2.VideoCapture(5)#4)#back
# # cap6 = cv2.VideoCapture(6)#4)#back
# # cap5 = cv2.VideoCapture(5)#4)#back

# #f1 br0 bl2 b4
# i = 0
# while True:
#     ret, frame = cap.read()\
#     # ret2, frame2 = cap2.read()
#     ret3, frame3 = cap3.read()
#     # ret4, frame4 = cap4.read()
#     # ret5, frame5 = cap5.read()
#     # ret6, frame6 = cap6.read()
#     if (ret3):
#         cv2.namedWindow("1",0)   #创建一个名为Video01的窗口，0表示窗口大小可调
#         #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         cv2.imshow("1", frame)
#         # cv2.namedWindow("2",0)   #创建一个名为Video01的窗口 ，0表示窗口大小可调
#         # #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         # cv2.imshow("2", frame2)
#         cv2.namedWindow("3",0)   #创建一个名为Video01的窗口 ，0表示窗口大小可调
#         #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         cv2.imshow("3", frame3)
#         # cv2.namedWindow("4",0)   #创建一个名为Video01的窗口 ，0表示窗口大小可调
#         # #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         # cv2.imshow("4", frame4)
#         # cv2.namedWindow("5",0)   #创建一个名为Video01的窗口 ，0表示窗口大小可调
#         # #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         # cv2.imshow("5", frame5)
#         # cv2.namedWindow("6",0)   #创建一个名为Video01的窗口 ，0表示窗口大小可调
#         # #cv2.resizeWindow("Video01",1280,720) ##创建一个名为Video01的窗口，设置窗口大小为 1920 * 1080 与上一个设置的 0 有冲突
#         # cv2.imshow("6", frame6)
#         c = cv2.waitKey(1) & 0xff

#         if c == ord("q"):
#             break
    
# #随时准备按q退出
# cap.release()
# #关掉所有窗口
# cv2.destroyAllWindows()
