import cv2
import numpy as np
import ast
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PositionPublisher:
    def __init__(self):
        rospy.init_node('position_publisher')

        # Declare parameters
        self.marker_size = rospy.get_param('marker_size', 0.1)
        self.marker_spacing = rospy.get_param('marker_spacing', 0.1)
        self.markers_x = rospy.get_param('markers_x', 2)
        self.markers_y = rospy.get_param('markers_y', 2)

        # Parse the camera matrix from the parameter server and convert it to a NumPy array
        camera_matrix_str = rospy.get_param('camera_matrix', None)
        self.camera_matrix = np.array(ast.literal_eval(camera_matrix_str))

        # Parse the distortion coefficients from the parameter server and convert them to a NumPy array
        dist_coeffs_str = rospy.get_param('dist_coeffs', None)
        self.dist_coeffs = np.array(ast.literal_eval(dist_coeffs_str))

        # Create a publisher to publish positions on the specified topic
        self.publisher1_ = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # Create a subscriber to listen to the camera image topic
        self.subscription = rospy.Subscriber('camera_image_topic', Image, self.frame_updater, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / 30), self.publish_position)

        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Define the aruco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Create the aruco board object with 4x4 grid and 16 markers
        self.board = cv2.aruco.GridBoard_create(self.markers_y, self.markers_x, self.marker_size, self.marker_spacing,
                                                self.aruco_dict)

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

    def frame_updater(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def publish_position(self, event):
        if self.frame is not None:
            rospy.loginfo("Hi")
            try:
                # Convert the frame to grayscale
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

                # Detect the aruco markers in the frame
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                # If at least one marker is detected
                if ids is not None:
                    # Estimate the pose of the board relative to the camera
                    ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix,
                                                                self.dist_coeffs, None, None, False)

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
                        quaternion_cv2 = self.rotation_matrix_to_quaternion(board_to_camera_matrix)

                        # Convert CV2 quaternion to ROS
                        self.quaternion_ros[0] = -quaternion_cv2[1]
                        self.quaternion_ros[1] = -quaternion_cv2[0]
                        self.quaternion_ros[2] = -quaternion_cv2[2]
                        self.quaternion_ros[3] = quaternion_cv2[3]

                # Publish the position as a ROS message
                vision_msg = PoseStamped()
                vision_msg.header = Header()
                vision_msg.header.stamp = rospy.Time.now()
                vision_msg.pose.position.x = float(self.position_ros[0])
                vision_msg.pose.position.y = float(self.position_ros[1])
                vision_msg.pose.position.z = float(self.position_ros[2])
                vision_msg.pose.orientation.x = float(self.quaternion_ros[0])
                vision_msg.pose.orientation.y = float(self.quaternion_ros[1])
                vision_msg.pose.orientation.z = float(self.quaternion_ros[2])
                vision_msg.pose.orientation.w = float(self.quaternion_ros[3])
                self.publisher1_.publish(vision_msg)

            except Exception as e:
                rospy.logerr('Error: ' + str(e))

    def rotation_matrix_to_quaternion(self, m):
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


def main():
    position_publisher = PositionPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
