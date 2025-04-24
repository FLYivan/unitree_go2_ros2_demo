import launch
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time')


    # 获取rviz配置文件路径
    rviz_file = os.path.join(
        get_package_share_directory('go2_camera_processing'),
        'rviz',
        'showcamera.rviz'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')


    # 接收串行口数据并发送运控指令节点
    start_cmd_node = Node(
            package='go2_demo_show',
            executable='brain_interface_cmd',
            name='brain_interface_cmd',
        )

    # 摄像头拉流节点
    start_camera_node =  Node(
            package='go2_camera_processing',                      
            executable='video_stream',             
            name='video_stream',
            output='screen',
            parameters=[{
                # 'multicast_iface': 'wlp0s20f3',
                'multicast_iface': 'enp0s31f6',
     
                         }]       
        )
    
    # RViz2节点
    start_rviz_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    # slam建图launch文件
    start_traject_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_slam_algorithm'), '/launch', '/go2_slam_toolbox.launch.py']),
    )	

    return LaunchDescription([

        declare_use_sim_time_cmd,

        start_cmd_node,
        start_camera_node,
        # start_rviz_node,

        start_traject_launch_file,

    ])



