from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'utlidar_lidar',
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.01,
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 10.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/utlidar/cloud'),
                ('scan', '/scan'),
            ]
        ),
    ])