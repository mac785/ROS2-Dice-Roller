import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class DualSenseNode(Node):
    def __init__(self):
        super().__init__('dualsense_node')
        self.publisher = self.create_publisher(Empty, 'move_command', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.button_index = 0  # Change to the button you want to use
        self.prev_pressed = False

    def joy_callback(self, msg: Joy):
        pressed = msg.buttons[self.button_index] == 1
        if pressed and not self.prev_pressed:
            # Button was just pressed
            self.publisher.publish(Empty())
            self.get_logger().info('Button pressed, move command sent')
        self.prev_pressed = pressed

def main(args=None):
    rclpy.init(args=args)
    node = DualSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()