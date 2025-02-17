#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('twist_mux_test'),
                                        'config', 'params.yaml')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')

    declare_joy_vel = DeclareLaunchArgument('joy_vel', default_value='/cmd_vel_drone')

    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('joy_vel'))},
            parameters=[config_file_path],
        )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_joy_vel)
    ld.add_action(twist_mux_node)

    return ld