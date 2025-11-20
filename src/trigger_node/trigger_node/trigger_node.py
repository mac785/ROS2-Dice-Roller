import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Twist

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')

        self.linear_scale = 0.1
        self.angular_scale = 1

        self.latest_sum = None
        self.dpad_up_pressed = False
        self.dpad_down_pressed = False

        # Subscribe to dice sum
        self.sum_sub = self.create_subscription(
            Int32,
            '/dice/sum',
            self.sum_callback,
            10
        )

        # Subscribe to buttons state
        self.buttons_sub = self.create_subscription(
            Int32MultiArray,
            '/buttons_state',
            self.buttons_callback,
            10
        )

        # Publisher for robot velocity
        self.velocity_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer to continuously publish velocity while D-Pad is held
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("TriggerNode running: waiting for dice sum and D-Pad input")

        # Configure which button indices correspond to D-Pad Up/Down
        # These may need to be adjusted for your controller
        self.dpad_up_index = 11
        self.dpad_down_index = 12
        self.dpad_left_index = 13
        self.dpad_right_index = 14

    # -----------------------------
    def sum_callback(self, msg: Int32):
        self.latest_sum = msg.data

    # -----------------------------
    def buttons_callback(self, msg: Int32MultiArray):
        buttons = msg.data
        self.dpad_up_pressed = buttons[self.dpad_up_index] == 1
        self.dpad_down_pressed = buttons[self.dpad_down_index] == 1
        self.dpad_left_pressed = buttons[self.dpad_left_index] == 1
        self.dpad_right_pressed = buttons[self.dpad_right_index] == 1

    # -----------------------------
    def timer_callback(self):
        """Publish Twist continuously based on D-Pad and dice sum."""
        if self.latest_sum is None:
            return

        twist_msg = Twist()
        if self.dpad_up_pressed:
            twist_msg.linear.x = -float(self.latest_sum) * self.linear_scale
        elif self.dpad_down_pressed:
            twist_msg.linear.x = float(self.latest_sum) * self.linear_scale
        else:
            twist_msg.linear.x = 0.0

        if self.dpad_left_pressed:
            twist_msg.angular.z = float(self.latest_sum) * self.angular_scale
        elif self.dpad_right_pressed:
            twist_msg.angular.z = -float(self.latest_sum) * self.angular_scale
        else:
            twist_msg.angular.z = 0.0

        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        self.velocity_pub.publish(twist_msg)

# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
