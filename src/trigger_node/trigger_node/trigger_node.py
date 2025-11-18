import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Float64

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')

        # ----- USER SETTINGS -----
        self.threshold = 10              # value above which we go to "high" position
        self.low_position = 0.2          # robot position for "low" case
        self.high_position = 1.0         # robot position for "high" case
        # --------------------------

        self.latest_sum = None

        # Subscribe to the dice sum
        self.sum_sub = self.create_subscription(
            Int32,
            '/dice/sum',
            self.sum_callback,
            10
        )

        # Subscribe to button press
        self.trigger_sub = self.create_subscription(
            Empty,
            '/move_command',
            self.trigger_callback,
            10
        )

        # Publish target position
        self.position_pub = self.create_publisher(
            Float64,
            '/target_position',
            10
        )

        self.get_logger().info("TriggerNode running: waiting for /move_command")

    def sum_callback(self, msg: Int32):
        """Stores the most recent dice sum."""
        self.latest_sum = msg.data

    def trigger_callback(self, _msg: Empty):
        """When the button is pressed, choose high or low position."""
        if self.latest_sum is None:
            self.get_logger().warn("Move command received but no dice sum yet!")
            return

        # Determine which position to send
        if self.latest_sum > self.threshold:
            pos = self.high_position
            self.get_logger().info(
                f"Dice sum {self.latest_sum} > {self.threshold}: moving to HIGH position ({pos})"
            )
        else:
            pos = self.low_position
            self.get_logger().info(
                f"Dice sum {self.latest_sum} <= {self.threshold}: moving to LOW position ({pos})"
            )

        # Publish the position
        msg = Float64()
        msg.data = pos
        self.position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
