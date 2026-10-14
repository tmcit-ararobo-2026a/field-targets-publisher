"""Launch explicit fixed targets; the supplied template contains no approved coordinates."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return a target provider with a required approval profile."""
    default_file = os.path.join(get_package_share_directory('field_target_pub'),
                                'config', 'targets.json')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='field_target_pub', executable='field_target_node',
             parameters=[{'config_file': LaunchConfiguration('config_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'),
    ])
