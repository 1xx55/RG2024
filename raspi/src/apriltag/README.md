# AprilTag Detection System

This Python script detects AprilTags in real-time using a connected camera, records detection results, and saves both processed and original images for each detection. It utilizes OpenCV, AprilTag, and other dependencies to perform detection and image processing.

## Features

- **Real-Time Detection**: Detects AprilTags in real-time from the camera feed.
- **Image Saving**: Saves both processed images (with detected tag overlays) and original images.
- **CSV Logging**: Records detection results, including timestamp, tag ID, and tag position, to a CSV file.
- **Automatic Folder Creation**: Creates a new folder for each test session based on the current timestamp to organize saved images and CSV files.

## Requirements

- Python 3.x
- OpenCV (`cv2`)
- NumPy
- AprilTag Python library
- A camera connected to your computer

## Installation

Ensure you have Python installed on your system. Then, install the required Python packages using pip:


## Usage

1. **Start the Script**: Run the script using Python.

    ```
    python Apriltag_Linux_optimized.py
    ```

2. **Real-Time Detection**: Point your camera at AprilTags. The script automatically detects tags, processes images, and logs detection data.

3. **Stop the Script**: Press 'q' to quit the detection loop and close the program.

## Output

- **Processed Images**: Images with detected tags highlighted and ID information overlaid. Saved in a test-specific folder.
- **Original Images**: Unaltered camera frames, saved only if tags are detected during the frame.
- **CSV Log**: A file named `detection_results.csv` within the test-specific folder, logging detection timestamps, tag IDs, and positions.

## Camera Calibration

The script uses a predefined camera matrix and distortion coefficients for tag detection. For optimal performance, calibrate your camera and replace the placeholder values with your calibration results.

## Customization

Modify the script parameters such as `tag_size`, camera index (`cap = cv2.VideoCapture(0)`), or CSV headers as needed to suit your requirements.
