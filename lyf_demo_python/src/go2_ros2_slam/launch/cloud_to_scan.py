from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                # 'target_frame': 'base_link',
                # 'target_frame': 'utlidar_lidar',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.1416,  # -M_PI/2
                'angle_max': 3.1416,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/utlidar/cloud'),
                # ('cloud_in', '/lio_sam_ros2/deskew/cloud_deskewed'),
                # ('cloud_in', '/rslidar_points'),                
                ('scan', '/scan'),
            ]
        ),


        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': 'false',  # 如果在仿真中使用，设置为True
        #         # 'scan_topic': '/scan',  # 设置激光雷达话题
        #         # 'scan_queue_size': 100,  # 设置消息队列大小
        #         # 'odom_frame': 'odom',  # 里程计的参考坐标系
        #         'base_frame': 'base_link',  # 里程计数据的子坐标系
        #         # 其他slam_toolbox参数
        #     }],

        # ),


    ])