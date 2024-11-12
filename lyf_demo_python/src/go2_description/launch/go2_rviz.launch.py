from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 获取URDF文件的路径
    pkg_path = os.path.join(get_package_share_directory('go2_description'))
    xacro_file = os.path.join(pkg_path,'urdf','mbot_gazebo.xacro')          #放在gazebo的模型文件
    robot_description_config = xacro.process_file(xacro_file)         #保存机器人模型的完整路径

    # 获取rviz配置文件路径
    rviz_file = os.path.join(
        get_package_share_directory('go2_description'),
        'launch',
        'check_joint.rviz'
    )


    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(),  #保存机器人模型的完整路径
              'use_sim_time': use_sim_time,
            #   'publish_frequency': 1000.0
              }    # 设置发布频率
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]           #维护机器人的TF关系
    )


    # 启动关节状态发布器节点
    node_joint_state_publisher_gui =   Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[
                {"use_gui": "TRUE"}  # 启用图形界面
            ]
        )
    # 启动RViz2节点
    node_rviz =Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file]
        )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',                       #申明了launch文件的参数
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        # node_joint_state_publisher_gui,
        # node_rviz,
    ])