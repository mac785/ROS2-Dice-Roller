import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy

class DualSenseNode(Node):
    def __init__(self):
        super().__init__('dualsense_node')

        # Publisher: all button states
        self.publisher = self.create_publisher(Int32MultiArray, 'buttons_state', 10)

        # Subscriber: raw joystick messages
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg: Joy):
        """
        Publish which buttons are currently pressed.
        Encoded pressed buttons as 1, released as 0 in an Int32MultiArray.
        """
        button_array = Int32MultiArray()
        button_array.data = msg.buttons  # msg.buttons is already a list of 0/1
        self.publisher.publish(button_array)

        # log for debugging
        pressed_buttons = [i for i, b in enumerate(msg.buttons) if b]
        if pressed_buttons:
            self.get_logger().info(f"Buttons pressed: {pressed_buttons}")

def main(args=None):
    rclpy.init(args=args)
    node = DualSenseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
