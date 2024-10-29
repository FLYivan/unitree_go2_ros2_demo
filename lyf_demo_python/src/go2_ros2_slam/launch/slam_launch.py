#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# slam_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取YAML文件的绝对路径
    params_file = os.path.join(
        os.path.dirname(__file__),                  # 获取当前文件的目录
        '..',                                       # 返回上一级目录
        'config',
        'unitree_slam_params.yaml'                  # YAML文件的名称
    )

    return LaunchDescription([
        Node(
            package='go2_ros2_slam',
            executable='slam_node',                                # 指定要运行的节点名
            output='both',
            parameters=[params_file]
        ),
        Node(
            package='go2_ros2_slam',
            executable='unitree_slam_service',
            output='both',
            parameters=[params_file]
        ),
        # Add more nodes as needed
    ])


# 举例
# my_launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='my_package_1',
#             executable='node1',
#             name='node1',
#             output='screen'
#         ),
#         Node(
#             package='my_package_2',
#             executable='node2',
#             name='node2',
#             output='screen'
#         ),
#     ])