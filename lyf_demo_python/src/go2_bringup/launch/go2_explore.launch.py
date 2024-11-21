from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 获取与拼接默认路径
    go2_explore_dir = get_package_share_directory(
        'go2_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        go2_explore_dir, 'rviz', 'nav2_go2_view.rviz')

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='False')

    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(go2_explore_dir, 'config', 'nav2_params.yaml'))
    
    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                            description='Use simulation (Gazebo) clock if true')
    
    declare_nav2_param_path_cmd = launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load')
    
    
    # 自定义的slam的launch文件
    start_slam_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([go2_explore_dir, '/launch', '/go2_slam_noshow.launch.py']),
    )	

    # nav2的navigation_launch.py文件
    start_nav2_launch_file = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    
    # cmd_vel联动go2的运动控制API
    start_go2Move_node =  Node(
            package='go2_bringup',                      
            executable='go2_move',             
            name='go2_move',
            output='screen',
        )
    
    # rviz2节点
    start_rviz_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )


    # # 自探索功能节点
    # start_explore_node =  Node(
    #         package='go2_bringup',                      
    #         executable='wavefront_frontier',             
    #         name='wavefront_frontier',
    #         parameters=[
    #             {'map_frame_id': 'map_slamtoolbox'},
    #             {'odom_topic': 'lio_sam_ros2/dogOdomProcess/DogOdomGlobal'},
    #             {'map_topic': 'map_slamtoolbox_go2'},
         
    #         ],
    #         output='screen',
    #     )
    



    return LaunchDescription(
        [

            declare_use_sim_time_cmd,
            declare_nav2_param_path_cmd,
            
            start_slam_launch_file,
            start_nav2_launch_file,
            start_go2Move_node,
            # start_rviz_node,
            # start_explore_node,

        ]
    )