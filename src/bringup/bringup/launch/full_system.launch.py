from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to the rrbot Gazebo launch file
    gazebo_pkg = 'ros2_control_demo_example_9'
    gazebo_launch_file = 'rrbot_gazebo.launch.py'

    gazebo_launch_path = os.path.join(
        get_package_share_directory(gazebo_pkg),
        'launch',
        gazebo_launch_file
    )

    return LaunchDescription([

        # 1. webcam publisher
        Node(
            package='webcam_publisher',
            executable='webcam_pub',
            output='screen'
        ),

        # 2. dice detector node
        Node(
            package='dice_detector',
            executable='dice_node',
            output='screen'
        ),

        # 3. joy node
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),

        # 4. dualsense node
        Node(
            package='dualsense_node',
            executable='dualsense_node',
            output='screen'
        ),

        # 5. trigger node
        Node(
            package='trigger_node',
            executable='trigger_node',
            output='screen'
        ),

        # 6. include the rrbot Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
    ])
