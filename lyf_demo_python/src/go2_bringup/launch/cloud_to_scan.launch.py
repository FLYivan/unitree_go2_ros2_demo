import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # 点云转2d激光节点
    node_p2l = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.14,                     # -M_PI/2
                'angle_max': 3.14,                      # M_PI/2
                'angle_increment': 0.0087,              # M_PI/360.0
                'scan_time': 0.05,                      # 20Hz
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                # ('cloud_in', '/lidar_points'),                
                ('cloud_in', '/rslidar_points'), 
                ('scan', '/scan_old'),
            ]
        )

    # 激光frame_id修改节点
    node_frame =  Node(
            package='go2_bringup',                      
            executable='frame_id_modifier',             
            name='frame_id_modifier',
            output='screen',
            parameters=[{'new_frame_id': 'ridar'}]        # 修改后frameid
        )

    # 延迟启动 node_frame
    delayed_node = TimerAction(period=3.0,  actions=[node_frame]) # 延迟 3 秒启动


    return LaunchDescription([
        node_p2l,
        delayed_node
    ])



