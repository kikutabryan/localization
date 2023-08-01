import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy


class CameraPublisher:
    def __init__(self):
        """
        Initialize the CameraPublisher class.

        This class is responsible for capturing images from the camera feed and publishing
        them as ROS Image messages on the specified topic.

        It uses OpenCV for video capture and the CvBridge class to convert OpenCV images
        to ROS Image messages.

        ROS parameters:
        - video_source: The video source for the camera feed in GStreamer pipeline format.
        """
        rospy.init_node('camera_publisher')

        # Create a publisher to publish images on the specified topic
        self.publisher_ = rospy.Publisher('camera_image_topic', Image, queue_size=10)

        # Create a CvBridge object to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        self.video_source = rospy.get_param('video_source', 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! nvvidconv flip-method=3 ! video/x-raw, width=720, height=1280, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink')

        # Video capture object
        self.cap = None

        # ROS image to publish
        self.ros_image = None

        # Open the camera feed
        self.connect_camera()

    def connect_camera(self):
        """
        Open the camera feed and connect to the video source.

        This function opens the camera feed using the GStreamer pipeline format
        specified in the 'video_source' parameter. It initializes the video capture
        object and logs the success or failure of the camera connection.

        Note: This function is automatically called when the CameraPublisher object is created.
        """
        # Check if the video capture is already open or not
        if self.cap is None or not self.cap.isOpened():
            try:
                # Open the camera feed
                source = self.video_source
                self.cap = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)

                # Uncomment the following lines to set video properties (optional)
                # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                # self.cap.set(cv2.CAP_PROP_FPS, 60)

                if self.cap.isOpened():
                    rospy.loginfo('Camera was opened successfully')
                    rospy.loginfo(self.cap.get(cv2.CAP_PROP_FPS))
                else:
                    rospy.logwarn('Failed to open camera')
            except cv2.error as e:
                rospy.logerr('Error: ' + str(e))

    def publish_image(self):
        """
        Continuously publish images from the camera feed.

        This function runs in a loop and continuously captures frames from the camera feed.
        It converts each frame to a ROS Image message and publishes it on the specified topic.

        Note: This function should be called after the CameraPublisher object is created.
        """
        while not rospy.is_shutdown():
            # Read a frame from the camera feed
            ret, frame = self.cap.read()

            if ret:
                # Convert the image to a ROS Image message
                self.ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            else:
                # Attempt to connect the camera again
                self.connect_camera()

            if self.ros_image is not None:
                # Publish the image on the specified topic
                self.publisher_.publish(self.ros_image)

    def __del__(self):
        """
        Destructor for the CameraPublisher class.

        This function is automatically called when the CameraPublisher object is deleted.
        It releases the video capture object to free up resources.
        """
        # Release the video capture when the object is deleted
        if self.cap is not None:
            self.cap.release()


def main():
    camera_publisher = CameraPublisher()
    camera_publisher.publish_image()
    del camera_publisher


if __name__ == '__main__':
    main()
