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
        self.video_source = rospy.get_param('video_source', '/dev/video0')

        # Video capture object
        self.cap = None

        # Open the camera feed
        self.connect_camera()

        # Create a timer event to periodically publish images
        # The timer period is set to 1/30th of a second (approximately 30 fps)
        self.timer = rospy.Timer(rospy.Duration(1.0 / 60), self.publish_image)

    def connect_camera(self):
        # Check if the video capture is already open or not
        if self.cap is None or not self.cap.isOpened():
            try:
                # Open the camera feed
                source = self.video_source
                self.cap = cv2.VideoCapture(source)
                if self.cap.isOpened():
                    rospy.loginfo('Camera was opened successfully')
                else:
                    rospy.logwarn('Failed to open camera')
            except cv2.error as e:
                rospy.logerr('Error: ' + str(e))

    def publish_image(self, event):
        # Read a frame from the camera feed
        ret, frame = self.cap.read()

        if ret:
            # Convert the image to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Publish the image on the specified topic
            self.publisher_.publish(ros_image)
        else:
            # Attempt to connect the camera again
            self.connect_camera()

    def __del__(self):
        # Release the video capture when the object is deleted
        if self.cap is not None:
            self.cap.release()


def main():
    camera_publisher = CameraPublisher()
    rospy.spin()
    del camera_publisher


if __name__ == '__main__':
    main()
