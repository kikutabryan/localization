import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')

        # Create a publisher to publish images on the specified topic
        self.publisher_ = rospy.Publisher('camera_image_topic', Image, queue_size=10)

        # Create a CvBridge object to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Get the video source path from the parameter server
        self.video_source = rospy.get_param('video_source', 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! nvvidconv flip-method=3 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink')

        # sensor_id = 1
        # capture_width = 1280
        # capture_height = 720
        # display_width = 1280
        # display_height = 720
        # framerate = 60
        # flip_method = 3
        # self.video_source = "nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink" % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)

        # Video capture object
        self.cap = None

        # ROS image to publish
        self.ros_image = None

        # Open the camera feed
        self.connect_camera()

    def connect_camera(self):
        # Check if the video capture is already open or not
        if self.cap is None or not self.cap.isOpened():
            try:
                # Open the camera feed
                source = self.video_source
                self.cap = cv2.VideoCapture(source)
                # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                # self.cap.set(cv2.CAP_PROP_FPS, 60)
                if self.cap.isOpened():
                    rospy.loginfo('Camera was opened successfully')
                else:
                    rospy.logwarn('Failed to open camera')
            except cv2.error as e:
                rospy.logerr('Error: ' + str(e))

    def publish_image(self):
        while not rospy.is_shutdown():
            # Read a frame from the camera feed
            ret, frame = self.cap.read()

            if ret:
                # Resize frame to smaller size
                # frame = cv2.resize(frame, (960, 540))

                # Convert the image to a ROS Image message
                self.ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            else:
                # Attempt to connect the camera again
                self.connect_camera()

            if self.ros_image is not None:
                # Publish the image on the specified topic
                self.publisher_.publish(self.ros_image)

    def __del__(self):
        # Release the video capture when the object is deleted
        if self.cap is not None:
            self.cap.release()


def main():
    camera_publisher = CameraPublisher()
    camera_publisher.publish_image()
    del camera_publisher


if __name__ == '__main__':
    main()
