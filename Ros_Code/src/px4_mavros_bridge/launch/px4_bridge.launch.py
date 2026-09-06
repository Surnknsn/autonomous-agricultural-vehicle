from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyACM0:57600',
                'gcs_url': '',
                'system_id': 1,
                'component_id': 1,
                'target_system_id': 1,
                'target_component_id': 1,
                'use_native_quaternion': True,
                'plugin_blacklist': ['gps_rtk', 'hil', 'terrain', 'ftp'],
            }],
        ),
    ])
