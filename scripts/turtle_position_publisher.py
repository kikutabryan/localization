import cv2
import numpy as np
import ast
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PositionPublisher:
    def __init__(self):
        rospy.init_node('position_publisher')

        self.frame = None

        self.turtle_position = np.empty((3, 1))
        self.turtle_position[0] = 0
        self.turtle_position[1] = 0
        self.turtle_position[2] = 0

        self.local_pose = PoseStamped()

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
        self.publisher1_ = rospy.Publisher('position_turtle_bot', PoseStamped, queue_size=10)

        # Create a subscriber to listen to the camera image topic
        self.subscription1 = rospy.Subscriber('camera_image_topic', Image, self.update_position, queue_size=10)

        # Create a subscriber to listen to the drone local position
        self.subscription2 = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.update_local_pose, queue_size=10)

        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Define the aruco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Create the aruco board object with 4x4 grid and 16 markers
        self.board = cv2.aruco.GridBoard_create(self.markers_y, self.markers_x, self.marker_size, self.marker_spacing, self.aruco_dict)

    def update_local_pose(self, msg):
        self.local_pose = msg

    def update_position(self, msg):
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
                    # Position of the turtlebot with respect to the drone (ENU)
                    position_t_d = np.empty((3, 1))
                    position_t_d[0] = tvec[0, 0]
                    position_t_d[1] = -tvec[0, 0]
                    position_t_d[2] = -tvec[0, 0]

                    # Rotation matrix of the drone with respect to the world (FLU)
                    quaternion_d_w = np.matrix((4, 1))
                    quaternion_d_w[0] = self.local_pose.pose.orientation.x
                    quaternion_d_w[1] = self.local_pose.pose.orientation.y
                    quaternion_d_w[2] = self.local_pose.pose.orientation.z
                    quaternion_d_w[3] = self.local_pose.pose.orientation.w
                    rotation_d_w = self.quaternion_to_rotation_matrix(quaternion_d_w)

                    # Rotate matrix to go from FLU to RFU
                    rotation_d_w = np.matmul(rotation_d_w, self.rotation_matrix_xyz(0, 0, np.pi / 2))

                    # Rotation matrix of the world with respect to the drone
                    rotation_w_d = np.matrix(rotation_d_w).T

                    # Position of the drone with respect to the world (ENU)
                    position_d_w = np.matrix((3, 1))
                    position_d_w[0] = self.local_pose.pose.position.x
                    position_d_w[1] = self.local_pose.pose.position.y
                    position_d_w[2] = self.local_pose.pose.position.z

                    # Position of the turtlebot with respect to the world (ENU)
                    position_t_w = np.matmul(rotation_w_d, position_t_d)
                    position_t_w = position_t_w + position_d_w

                    # Set the turtlebot position
                    self.turtle_position = position_t_w
                    self.publish_position()

        except Exception as e:
            rospy.logerr('Error: ' + str(e))

    def publish_position(self):
        try:
            # Publish the position as a ROS message
            vision_msg = PoseStamped()
            vision_msg.header.stamp = rospy.Time.now()
            vision_msg.header.frame_id = 'map'
            vision_msg.pose.position.x = float(self.turtle_position[0])
            vision_msg.pose.position.y = float(self.turtle_position[1])
            vision_msg.pose.position.z = float(self.turtle_position[2])
            self.publisher1_.publish(vision_msg)

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
    
    def quaternion_to_rotation_matrix(self, q):
        # Ensure the quaternion is normalized
        q = q / np.linalg.norm(q)

        # Extract quaternion components
        x, y, z, w = q

        # Compute the rotation matrix elements
        xx = x * x
        xy = x * y
        xz = x * z
        xw = x * w

        yy = y * y
        yz = y * z
        yw = y * w

        zz = z * z
        zw = z * w

        # Build the rotation matrix
        rotation_matrix = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
            [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
            [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)]
        ])

        return rotation_matrix

def main():
    position_publisher = PositionPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
