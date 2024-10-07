import numpy as np
import cv2
import glob

# Define the dimensions of the checkerboard
CHECKERBOARD = (12, 16)  # number of corners (width, height)
square_size = 0.035    # size of a square in your defined unit (meter, inch...)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (5,8,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Path to the folder containing the images
images_path = '/home/yao/Documents/mmwave/pic/*.jpg'  # Update this path

# Get a list of all the image file names
images = glob.glob(images_path)

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Img', img)
        cv2.waitKey(500)  # Wait a bit for each image

cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Output the camera matrix and distortion coefficients
print("Camera matrix:")
print(mtx)
print("Distortion coefficients:")
print(dist)

# Camera matrix:
# [[930.79652155   0.         661.20967027]
#  [  0.         932.27891919 408.65175784]
#  [  0.           0.           1.        ]]
# Distortion coefficients:
# [[ 4.81284473e-02 -8.86667343e-02  6.04081268e-04 -6.77186841e-05
#    2.81485370e-02]]
