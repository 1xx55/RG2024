import cv2
import numpy as np
import pupil_apriltags as pl
import os
import csv
from datetime import datetime
import time
import math
# Create a folder to save images

# Initialize CSV file and write the header row

# Camera intrinsic parameters (these values should be obtained by calibrating your camera)
right_camera_matrix =np.array([[4.412598730441295e+02,0,3.001939990083209e+02],
[0,4.411583903956606e+02,2.442232787044877e+02],
                               [0,0,1]])
right_distortion = np.array([[-0.068257502583129,0.155030717002371,0,0,0]])
# right_camera_matrix =np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
#     [0,6.662070938786943e+02,3.525276587772178e+02],
#     [0,0,1]])
# right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

# The actual size of the AprilTag (in meters)
tag_size = 0.4  # Modify this to the actual size of your tag
options = pl.Detector(families='tag36h11')
def cal_turn_angle(x1,y1,id1,x2,y2,id2):
    k=((y2-y1)/(x2-x1))
    b=40+64*id1-64*(id2-id1)*x1/(x2-x1)
    angle=np.arctan(k)*180/math.pi
    pos_x=300-(y1-k*x1)*(x2-x1)/math.sqrt((x2-x1)**2+(y2-y1)**2)
    pos_y=b-(y1-k*x1)*(y2-y1)/math.sqrt((x2-x1)**2+(y2-y1)**2)
    return angle,pos_x,pos_y
def self_locate(image):
    self_position = [] 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = options.detect(gray)
    for r in results:
        id = r.tag_id
        corners = r.corners.astype(int)
        object_points = np.array([[-tag_size / 2, -tag_size / 2, 0],
                                [tag_size / 2, -tag_size / 2, 0],
                                [tag_size / 2, tag_size / 2, 0],
                                [-tag_size / 2, tag_size / 2, 0]])
        retval, rvec, tvec = cv2.solvePnP(object_points, corners.astype(np.float64), right_camera_matrix, right_distortion)
        tag_position = tvec.flatten()
        # print("tag position: ", tag_position)
        # print("tag id: ", id)
        tag_position = np.array([tag_position[0]*100, tag_position[1]*100, tag_position[2]*100])
        self_position.append([tag_position[0], tag_position[2], id])
        # self_position.append([40+64*id-tag_position[0], 300-tag_position[2]])
    # if self_position != []:
    #     x=0
    #     y=0
    #     for i in range(len(self_position)):
    #         x+=self_position[i][1]
    #         y+=self_position[i][0]
    #     acutual_location = np.array([int(x/len(self_position)), int(y/len(self_position))])
    #     print("agent position: ", acutual_location)
    #     return self_position
    # else:
    #     return None
    if self_position != []:
        if len(self_position) >1:
            angle,pos_x,pos_y=cal_turn_angle(self_position[0][0],self_position[0][1],self_position[0][2],self_position[1][0],self_position[1][1],self_position[1][2])
            print("ngle: ", angle)
            print("position: ", pos_x, pos_y)
            return int(angle),int(pos_x-20),int(pos_y)
        else:
            # return 0,int(300-self_position[0][1]),int(40+64*self_position[0][2]-self_position[0][0])
            return 180,False,False
    else:
        return False,False,False

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_EXPOSURE,-5)
    
    while True:
        ret,image = cap.read()
        # cv2.windowName('image',0)
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self_locate(image)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break
    cap.release()
    cv2.destroyAllWindows()
