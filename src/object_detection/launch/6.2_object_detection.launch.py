from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            output='screen',
            parameters=['/home/ubuntu/Relbot_ws/src/cam2image_vm2ros/config/cam2image.yaml']
        ),
        Node(
            package='image_tools',
            executable='showimage',
            output='screen'
        ),
        Node(
            package='object_detection',
            executable='green_object_detector',
            output='screen'
        ),
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator"
        ),
        Node(
            package="relbot2turtlesim",
            executable="relbot2turtlesim",
            name="relbot2turtlesim"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        ),
        Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        )
    ])