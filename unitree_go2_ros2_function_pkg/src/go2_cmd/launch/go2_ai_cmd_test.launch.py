import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml

def generate_launch_description():
    # 输入参数声明

    cmd_params_file = LaunchConfiguration('cmd_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 获取launch文件和定义地址
    bringup_dir = get_package_share_directory("go2_cmd")

    # 获取rviz配置文件路径
    rviz_file = os.path.join(
        get_package_share_directory('go2_slam_algorithm'),
        'rviz',
        'dog_slam_simp.rviz'
    )


    # 声明go2控制节点启动时，参数文件的默认地址
    declare_go2_cmd_params_file = DeclareLaunchArgument(
        'cmd_params_file',
        default_value=os.path.join(bringup_dir,'config', 'cmd.yaml'),                
        description='Full path to the cmd parameters file to use for the some cmd node')
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    
    
    # 模拟速度指令发布节点
    start_vel_pub_test_node =  Node(
            package='go2_cmd',                      
            executable='vel_pub_test',             
            name='vel_pub_test',
            output='screen',
        )
    
    # go2控制节点
    start_go2_ai_cmd_node = Node(
            parameters=[
                cmd_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='go2_cmd',
            executable='go2_ai_cmd',
            name='go2_ai_cmd',
            output='screen',
            # remappings=[
            #         ('/map', '/map_slamtoolbox_go2')  # 将 /map 重映射为 /map_slamtoolbox_go2
            #          ]
        )
 
    # XX的launch文件
    start_lidar_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_lidar_processing'), '/launch', '/cloud_to_scan.launch.py']),
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





    return launch.LaunchDescription([

        
        declare_go2_cmd_params_file,
        declare_use_sim_time_cmd,
  
        # start_lidar_launch_file,
        # start_vel_pub_test_node,        # 启动模拟速度指令发布节点
        start_go2_ai_cmd_node,

        # start_rviz_node,




    ])
