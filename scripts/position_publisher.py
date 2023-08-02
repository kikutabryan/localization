import cv2
import numpy as np
import ast
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import csv
import os

class PositionPublisher:
    def __init__(self):
        """
        Initialize the PositionPublisher class.

        This class is responsible for detecting ArUco markers in the camera image,
        estimating the pose of the ArUco board relative to the camera, and publishing
        the position and orientation of the board as a ROS PoseStamped message.

        It also saves the position data to a CSV file.

        The class subscribes to the camera image topic and publishes the position on
        the specified topic. It uses OpenCV for ArUco marker detection and pose estimation.

        ROS parameters:
        - marker_size: Size of the ArUco marker side length in meters.
        - marker_spacing: Spacing between adjacent ArUco markers on the board in meters.
        - markers_x: Number of markers along the X axis of the board.
        - markers_y: Number of markers along the Y axis of the board.
        - camera_matrix: Camera intrinsic matrix as a string representation of a 3x3 list.
        - dist_coeffs: Camera distortion coefficients as a string representation of a 1x5 list.
        """
        rospy.init_node('position_publisher')

        self.frame = None

        self.position_ros = np.empty((3, 1))
        self.position_ros[0] = 0
        self.position_ros[1] = 0
        self.position_ros[2] = 0

        self.quaternion_ros = np.empty((4, 1))
        self.quaternion_ros[0] = 0
        self.quaternion_ros[1] = 0
        self.quaternion_ros[2] = 0
        self.quaternion_ros[3] = 1

        # Declare parameters
        self.marker_size = rospy.get_param('marker_size', 0.075)
        self.marker_spacing = rospy.get_param('marker_spacing', 0.522)
        self.markers_x = rospy.get_param('markers_x', 7)
        self.markers_y = rospy.get_param('markers_y', 7)

        # Parse the camera matrix from the parameter server and convert it to a NumPy array
        camera_matrix_str = rospy.get_param('camera_matrix', '[[1204.4697584542726, 0.0, 354.6182298916754], [0.0, 1201.8571173383637, 652.7026112972777], [0.0, 0.0, 1.0]]')
        self.camera_matrix = np.array(ast.literal_eval(camera_matrix_str))

        # Parse the distortion coefficients from the parameter server and convert them to a NumPy array
        dist_coeffs_str = rospy.get_param('dist_coeffs', '[[0.008252427738841408, 0.17962144633096283, -0.0010671121861827175, -0.0017195153485555525, -0.7083074964169391]]')
        self.dist_coeffs = np.array(ast.literal_eval(dist_coeffs_str))

        # Create a publisher to publish positions on the specified topic
        self.publisher1_ = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # Create a subscriber to listen to the camera image topic
        self.subscription = rospy.Subscriber('camera_image_topic', Image, self.update_position, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / 50), self.publish_position)

        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Define the aruco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Create the aruco board object with 4x4 grid and 16 markers
        self.board = cv2.aruco.GridBoard_create(self.markers_y, self.markers_x, self.marker_size, self.marker_spacing, self.aruco_dict)

    def update_position(self, msg):
        """
        Callback function for the camera image subscriber.

        This function detects ArUco markers in the camera image and estimates the pose
        of the ArUco board relative to the camera. It then converts the pose to ROS
        coordinate convention (ENU) and stores the position and orientation.

        Parameters:
        - msg: The ROS Image message from the camera.

        Note: This function is called automatically when a new camera image is received.
        """
        # Convert the ROS Image message to an OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        try:
            # Detect the aruco markers in the frame
            corners, ids, rejected = cv2.aruco.detectMarkers(self.frame, self.aruco_dict, parameters=self.aruco_params)

            # If at least one marker is detected
            if ids is not None:
                # Estimate the pose of the board relative to the camera
                ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, None, None, False)

                # If the pose estimation is successful
                if ret > 0:
                    # Determine the rotation matrix from the board to the camera
                    camera_to_board_matrix, _ = cv2.Rodrigues(rvec)
                    board_to_camera_matrix = np.matrix(camera_to_board_matrix).T

                    # Determine the translation vector from the board to the camera
                    camera_to_board_vector = np.empty((3, 1))
                    camera_to_board_vector[0] = tvec[0, 0]
                    camera_to_board_vector[1] = tvec[1, 0]
                    camera_to_board_vector[2] = tvec[2, 0]
                    board_to_camera_vector = -np.matmul(board_to_camera_matrix, camera_to_board_vector)
                    position_cv2 = np.empty((3, 1))
                    position_cv2[0] = board_to_camera_vector[0]
                    position_cv2[1] = board_to_camera_vector[1]
                    position_cv2[2] = board_to_camera_vector[2]

                    # Convert CV2 positions to ROS (ENU)
                    self.position_ros[0] = position_cv2[0]
                    self.position_ros[1] = -position_cv2[1]
                    self.position_ros[2] = -position_cv2[2]

                    # Generate a quaternion from the rotation matrix
                    quaternion_cv2 = self.rotation_matrix_to_quaternion(np.matmul(board_to_camera_matrix, self.rotation_matrix_xyz(0, 0, -np.pi / 2)))

                    # Convert CV2 quaternion to ROS
                    self.quaternion_ros[0] = quaternion_cv2[0]
                    self.quaternion_ros[1] = -quaternion_cv2[1]
                    self.quaternion_ros[2] = -quaternion_cv2[2]
                    self.quaternion_ros[3] = quaternion_cv2[3]

        except Exception as e:
            rospy.logerr('Error: ' + str(e))

    def publish_position(self, event):
        """
        Publish the position and orientation as a ROS PoseStamped message.

        This function is called periodically based on the timer to publish the
        position and orientation of the ArUco board as a ROS PoseStamped message.

        Parameters:
        - event: The ROS Timer event triggering the function.
        """
        try:
            # Publish the position as a ROS message
            vision_msg = PoseStamped()
            vision_msg.header.stamp = rospy.Time.now()
            vision_msg.header.frame_id = 'map'
            vision_msg.pose.position.x = float(self.position_ros[0])
            vision_msg.pose.position.y = float(self.position_ros[1])
            vision_msg.pose.position.z = float(self.position_ros[2])
            vision_msg.pose.orientation.x = float(self.quaternion_ros[0])
            vision_msg.pose.orientation.y = float(self.quaternion_ros[1])
            vision_msg.pose.orientation.z = float(self.quaternion_ros[2])
            vision_msg.pose.orientation.w = float(self.quaternion_ros[3])
            self.publisher1_.publish(vision_msg)

            # Save the data to the CSV file
            self.save_to_csv(vision_msg.header.stamp, vision_msg.pose.position.x, vision_msg.pose.position.y, vision_msg.pose.position.z, vision_msg.pose.orientation.x, vision_msg.pose.orientation.y, vision_msg.pose.orientation.z, vision_msg.pose.orientation.w)

        except Exception as e:
            rospy.logerr('Error: ' + str(e))

    def rotation_matrix_xyz(self, theta_x, theta_y, theta_z):
        """
        Create a 3x3 rotation matrix for rotations around the X, Y, and Z axes.

        Parameters:
        - theta_x: Rotation angle around the X axis in radians.
        - theta_y: Rotation angle around the Y axis in radians.
        - theta_z: Rotation angle around the Z axis in radians.

        Returns:
        - R: 3x3 NumPy array representing the rotation matrix.
        """
        cos_x = np.cos(theta_x)
        sin_x = np.sin(theta_x)
        cos_y = np.cos(theta_y)
        sin_y = np.sin(theta_y)
        cos_z = np.cos(theta_z)
        sin_z = np.sin(theta_z)

        R_x = np.array([[1, 0, 0],
                        [0, cos_x, -sin_x],
                        [0, sin_x, cos_x]])

        R_y = np.array([[cos_y, 0, sin_y],
                        [0, 1, 0],
                        [-sin_y, 0, cos_y]])

        R_z = np.array([[cos_z, -sin_z, 0],
                        [sin_z, cos_z, 0],
                        [0, 0, 1]])

        R = np.dot(np.dot(R_z, R_y), R_x)

        return R

    def rotation_matrix_to_quaternion(self, m):
        """
        Convert a 3x3 rotation matrix to a quaternion.

        Parameters:
        - m: 3x3 NumPy array representing the rotation matrix.

        Returns:
        - q: 1x4 NumPy array representing the quaternion (x, y, z, w).
        """
        q = np.empty((4,), dtype=np.float64)
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (m[2, 1] - m[1, 2]) * s
            q[1] = (m[0, 2] - m[2, 0]) * s
            q[2] = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                q[3] = (m[2, 1] - m[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (m[0, 1] + m[1, 0]) / s
                q[2] = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                q[3] = (m[0, 2] - m[2, 0]) / s
                q[0] = (m[0, 1] + m[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                q[3] = (m[1, 0] - m[0, 1]) / s
                q[0] = (m[0, 2] + m[2, 0]) / s
                q[1] = (m[1, 2] + m[2, 1]) / s
                q[2] = 0.25 * s
        return q

    def save_to_csv(self, timestamp, x, y, z, qx, qy, qz, qw):
        """
        Save the position data to a CSV file.

        Parameters:
        - timestamp: The timestamp of the position data.
        - x: X-coordinate of the position.
        - y: Y-coordinate of the position.
        - z: Z-coordinate of the position.

        Note: The data is appended to the CSV file with the following format:
              Timestamp, X, Y, Z
        """
        try:
            file_path = os.path.join(os.path.expanduser('~'), 'position_data_aruco.csv')
            rospy.loginfo(f"Saving file to: {file_path}")

            file_exists = os.path.isfile(file_path)

            # If the file does not exist, create a new one and write the header row
            if not file_exists:
                with open(file_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'qX', 'qY', 'qZ', 'qW'])

            # Open the CSV file in append mode and write the data to a row
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, x, y, z, qx, qy, qz, qw])
        except Exception as e:
            rospy.logerr('Error: ' + str(e))

def main():
    position_publisher = PositionPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
