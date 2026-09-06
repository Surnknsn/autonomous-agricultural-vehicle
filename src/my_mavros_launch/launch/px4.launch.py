#!/usr/bin/env python3
"""
Launch file for PX4 + MAVROS in ROS2 Foxy
Compatible with Pixhawk 1.15.4
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:57600',
        description='Serial port of the flight controller'
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Ground Control Station URL (optional)'
    )
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )
    disable_protocol_check_arg = DeclareLaunchArgument(
        'disable_protocol_check',
        default_value='true',
        description='Disable PX4 VER protocol check (useful for v1.15.4)'
    )

    # MAVROS Node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system': LaunchConfiguration('tgt_system'),
            'target_component': LaunchConfiguration('tgt_component'),
            'disable_protocol_check': LaunchConfiguration('disable_protocol_check'),
            'fcu_protocol': 'v2.0'
        }],
        arguments=['--ros-args']
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        disable_protocol_check_arg,
        mavros_node
    ])
