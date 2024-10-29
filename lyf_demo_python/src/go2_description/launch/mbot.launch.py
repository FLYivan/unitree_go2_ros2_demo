import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('learning_gazebo'))
    xacro_file = os.path.join(pkg_path,'urdf','mbot_gazebo.xacro')          #放在gazebo的模型文件
    robot_description_config = xacro.process_file(xacro_file)         #保存机器人模型的完整路径
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}  #保存机器人模型的完整路径
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]           #维护机器人的TF关系
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',                       #申明了launch文件的参数
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
