import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class DiceDetector(Node):
    def __init__(self):
        super().__init__('dice_detector')

        # Load YOLO model (replace with your model path or Roboflow URL)
        weights_path = "/root/yolo_models/dice.v1i.yolov8/runs/detect/train/weights/best.pt"
        self.model = YOLO(weights_path)

        self.bridge = CvBridge()

        # Subscribe to raw camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info('DiceDetector started')

        # Publisher for dice sum
        self.dice_pub = self.create_publisher(Int32, '/dice/sum', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(frame)

        # Extract predictions
        dice_sum = 0
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # bounding boxes
            scores = result.boxes.conf.cpu().numpy()  # confidence
            classes = result.boxes.cls.cpu().numpy()  # predicted class indices

            # Assuming your YOLO model class 0..5 matches dice face 1..6
            for cls in classes:
                dice_sum += 7 - (int(cls) + 1)

        # Publish dice sum
        msg_out = Int32()
        msg_out.data = dice_sum
        self.dice_pub.publish(msg_out)

        # Optional: display for debugging
        # cv2.imshow("Dice Detection", frame)
        annotated_image = results[0].plot()
        cv2.imshow("Dice Detection", annotated_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DiceDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
