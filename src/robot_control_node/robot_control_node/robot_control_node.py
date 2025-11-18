import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.subscription = self.create_subscription(Pose, 'desired_position', self.position_callback, 10)
        self.current_target = None

    def position_callback(self, msg: Pose):
        self.current_target = msg
        self.get_logger().info(f'Received target position: {msg}')
        self.move_to_target(msg)

    def move_to_target(self, pose: Pose):
        # Replace this with your Gazebo or ros2_control command logic
        self.get_logger().info(f'Moving robot to pose: {pose}')
        # Example: send joint commands or Pose commands to robot controller

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
