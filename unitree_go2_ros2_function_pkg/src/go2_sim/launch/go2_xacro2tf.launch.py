#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Debug mode for robot'
    )

    # 获取功能包路径
    pkg_path = get_package_share_directory('go2_sim')  # 替换为您的功能包名称

    # xacro 文件路径
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')  # 根据实际路径调整

    # 使用 xacro 命令处理 xacro 文件
    robot_description = Command(
        [
            FindExecutable(name='xacro'), ' ',
            xacro_file, ' ',
            'debug:=',
            LaunchConfiguration('debug')
        ]
    )

    # 配置 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 配置 joint_state_publisher 节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 添加 rviz2 节点用于可视化
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'go2_rviz.rviz')  # 根据实际路径调整
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        debug_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])