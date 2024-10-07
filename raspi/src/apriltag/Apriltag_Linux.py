import cv2
import numpy as np
import apriltag
import os
import csv
import time

def create_folder_for_test():
    """Create a new folder to save images and CSV files for the current test, named with the current timestamp."""
    folder_name = time.strftime("test_%Y%m%d_%H%M%S")
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    return folder_name

# Create a new folder for each test
test_folder = create_folder_for_test()

# Initialize the CSV file and write the header row
csv_filename = os.path.join(test_folder, "detection_results.csv")
with open(csv_filename, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Tag ID", "Tag Position"])

# Camera intrinsic parameters (these values should be obtained by calibrating your camera)
camera_matrix = np.array([[930.79652155, 0, 661.20967027],
                          [0, 932.27891919, 408.65175784],
                          [0, 0, 1]], dtype=np.float64)
dist_coeffs = np.array([[0.0481284473, -0.0886667343, 0.000604081268, -0.0000677186841, 0.0281485370]])

# The actual size of the AprilTag (in meters)
tag_size = 0.16  # Modify this to the actual size of your tag

cap = cv2.VideoCapture(0)  # Capture from the camera
options = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

while True:
    ret, image1 = cap.read()
    if not ret:
        break  # Exit the loop if the image cannot be captured

    current_time = time.time()  # Get the current time

    gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    results = options.detect(gray)
    for r in results:
        # Get the ID
        id = r.tag_id
        print(f"Detected AprilTag ID: {id}, Time: {current_time}")

        # Get the coordinates of the four corners
        corners = r.corners.astype(int)
        # Draw lines between each corner point
        for i in range(4):
            cv2.line(image1, tuple(corners[i - 1]), tuple(corners[i]), (255, 0, 255), 2, cv2.LINE_AA)

        # Calculate 3D coordinates
        object_points = np.array([[-tag_size / 2, -tag_size / 2, 0],
                                  [tag_size / 2, -tag_size / 2, 0],
                                  [tag_size / 2, tag_size / 2, 0],
                                  [-tag_size / 2, tag_size / 2, 0]])
        retval, rvec, tvec = cv2.solvePnP(object_points, corners.astype(np.float64), camera_matrix, dist_coeffs)

        tag_position = tvec.flatten()
        print("Tag position: ", tag_position)
        # Uncomment to print the distance: print('distance:', tag_position[2])

        # Draw the center coordinate of the AprilTag
        center = np.mean(corners, axis=0).astype(int)
        cv2.circle(image1, tuple(center), 5, (0, 0, 255), -1)

        # Overlay text information on the image
        cv2.putText(image1, f"ID: {id}", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Save the current detected information to the CSV file
        with open(csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, id, tag_position])

        # Save the image with the detected tag
        image_filename = os.path.join(test_folder, f"tag_{id}_str{current_time}.jpg")
        cv2.imwrite(image_filename, image1)

    # Display the processed image
    cv2.imshow("Apriltags", image1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
