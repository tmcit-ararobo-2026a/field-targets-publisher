from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="field_target_pub",
            executable="field_target_node",
        ),
    ])