import cv2
import numpy as np
import apriltag
import os
import csv
from datetime import datetime
import time
# Create a folder to save images

# Initialize CSV file and write the header row

# Camera intrinsic parameters (these values should be obtained by calibrating your camera)
right_camera_matrix =np.array([[6.666340134907508e+02,0,6.450309322183250e+02],
    [0,6.662070938786943e+02,3.525276587772178e+02],
    [0,0,1]])
right_distortion = np.array([[-0.060693317140140,0.148720908337062,-4.696798593761847e-04,-3.517967377954150e-05,-0.109112699405528]])

# The actual size of the AprilTag (in meters)
tag_size = 0.16  # Modify this to the actual size of your tag
options = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
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
        tag_position = np.array([tag_position[0]*100, tag_position[1]*100, tag_position[2]*100])
        self_position.append([int(40+64*id+tag_position[0]), 303-tag_position[2]])
        print("agent position: ", self_position)
    if self_position != []:
        acutual_location = np.array([np.sum(self_position[1])/len(self_position), np.sum(self_position[0])/len(self_position)])
        return acutual_location
    else:
        return None

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while True:
        ret, image = cap.read()
        if ret:
            tag_position = self_locate(image)
            break
    cap.release()
    cv2.destroyAllWindows()
