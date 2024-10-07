import cv2

# Create a video capture object, 0 means the system's default camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open the camera")
else:
    # Get the current resolution
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print(f"The current camera resolution is: {int(width)}x{int(height)}")

# Release the camera
cap.release()

# The current camera resolution is: 1280x720
