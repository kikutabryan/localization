import cv2
import numpy as np
import time
import yaml
import os

# Constants for camera source
SOURCE_DEFAULT_CAMERA = 0
SOURCE_CSI_CAMERA = 1
SOURCE_VIDEO_MP4 = 2

CAMERA = SOURCE_DEFAULT_CAMERA
# CAMERA = SOURCE_CSI_CAMERA
# CAMERA = SOURCE_VIDEO_MP4

# Constants for display output
DISPLAY_NONE = 0
DISPLAY_WINDOW = 1
DISPLAY = DISPLAY_WINDOW

def main():
    # Select the camera source
    source = CAMERA

    # Display video
    display = DISPLAY

    # Set the source address based on the selected source
    if source == SOURCE_DEFAULT_CAMERA:
        address = 0  # Default camera
    elif source == SOURCE_CSI_CAMERA:
        address = (
            # CSI camera
            "nvarguscamerasrc sensor-id={sensor_id} ! "
            "video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
            "nvvidconv flip-method={flip_method} ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
        ).format(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            framerate=60,
            flip_method=3
        )
    elif source == SOURCE_VIDEO_MP4:
        address = 'video.mp4'

    pattern_size = (6, 7)  # Number of inner corners of the calibration pattern
    square_size = 0.0254  # Size of each square in meters (assuming the calibration pattern is printed on a square grid)

    # Define termination criteria for calibration
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (7,5,0)
    object_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    object_points[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    object_points *= square_size

    # Arrays to store object points and image points from all images
    obj_points = []  # 3D points in real-world space
    img_points = []  # 2D points in image plane

    # Video capture object
    cap = None

    try:
        while True:
            # Open the video capture from the specified address
            if cap is None or not cap.isOpened():
                try:
                    cap = cv2.VideoCapture(address)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    if cap.isOpened():
                        print('Video capture was opened successfully.')
                    else:
                        print('Failed to open video capture. Retrying in 1 second...')
                        time.sleep(1)  # Wait for 1 second before retrying

                except cv2.error as e:
                    print('Error:', str(e))
                    print('Retrying in 1 second...')
                    time.sleep(1)  # Wait for 1 second before retrying

            else:
                # Read frame from video capture
                ret, frame = cap.read()
                if not ret:
                    pass

                # Convert the frame from the video capture to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Find chessboard corners
                ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

                if ret:
                    obj_points.append(object_points)
                    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    img_points.append(corners)

                    # Draw and display the corners on the original frame
                    cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
                    cv2.imshow('Chessboard', frame)

                # Display the video capture frame
                if display == DISPLAY_WINDOW:
                    cv2.imshow('Video Capture', frame)
                    cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Program interrupted by user")

    finally:
        print("Cleaning up...")

        # Release the video capture
        cap.release()
        if display == DISPLAY_WINDOW:
            cv2.destroyAllWindows()

        # Perform camera calibration using the last captured frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the last captured frame to grayscale
        ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

        # Convert the arrays to strings
        camera_matrix = str(camera_matrix.tolist())
        dist_coeffs = str(dist_coeffs.tolist())

        # Get the path to the YAML file
        current_directory = os.getcwd()
        parent_directory = os.path.dirname(current_directory)
        target_folder = 'config'
        yaml_file = 'params.yaml'
        yaml_path = os.path.join(parent_directory, target_folder, yaml_file)

        # Load the YAML file
        with open(yaml_path, 'r') as file:
            yaml_content = yaml.safe_load(file)

        # Update the values in the YAML content
        yaml_content['camera_matrix'] = camera_matrix
        yaml_content['dist_coeffs'] = dist_coeffs
        
        # Save the dictionary to a YAML file
        with open(yaml_path, 'w') as file:
            yaml.dump(yaml_content, file)

if __name__ == '__main__':
    main()