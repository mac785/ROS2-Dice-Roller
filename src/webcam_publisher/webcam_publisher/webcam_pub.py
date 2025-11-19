import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1 / 30, self.timer_callback)  # ~30 FPS

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("/dev/usb_cam")  # Use camera 0, may need to change on other devices

        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")
            exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        # Convert OpenCV BGR → ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
