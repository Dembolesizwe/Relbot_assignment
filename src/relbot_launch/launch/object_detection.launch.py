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
            executable='ImageProcessing',
            output='screen')
    ])

