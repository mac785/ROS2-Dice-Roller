import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from std_msgs.msg import Float64MultiArray

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')

        # -----------------------------
        # USER SETTINGS
        # -----------------------------
        self.threshold = 10

        # Position if dice sum is <= threshold
        self.position_low = [0.2, 0.2]

        # Position if dice sum is > threshold
        self.position_high = [1.0, 1.0]
        # -----------------------------

        self.latest_sum = None

        # Subscribe to dice sum
        self.sum_sub = self.create_subscription(
            Int32,
            '/dice/sum',
            self.sum_callback,
            10
        )

        # Subscribe to joystick trigger / button press
        self.trigger_sub = self.create_subscription(
            Empty,
            '/move_command',
            self.trigger_callback,
            10
        )

        # Publisher to the robot controller
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        self.get_logger().info("TriggerNode running: waiting for /move_command")

    # --------------------------------------------------------

    def sum_callback(self, msg: Int32):
        """Store the most recent dice sum."""
        self.latest_sum = msg.data

    # --------------------------------------------------------

    def trigger_callback(self, _msg: Empty):
        """When the joystick button is pressed, choose a high or low position."""
        if self.latest_sum is None:
            self.get_logger().warn("Button pressed, but no dice sum received yet!")
            return

        # Choose which position based on threshold
        if self.latest_sum > self.threshold:
            target = self.position_high
            self.get_logger().info(
                f"Dice sum {self.latest_sum} > {self.threshold}: moving to HIGH position {target}"
            )
        else:
            target = self.position_low
            self.get_logger().info(
                f"Dice sum {self.latest_sum} <= {self.threshold}: moving to LOW position {target}"
            )

        # Publish Float64MultiArray
        msg_out = Float64MultiArray()
        msg_out.data = target

        self.position_pub.publish(msg_out)

    # --------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
