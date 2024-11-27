import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取与拼接默认路径
    go2_navigation2_dir = get_package_share_directory(
        'go2_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        go2_navigation2_dir, 'rviz', 'nav2_go2_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(go2_navigation2_dir, 'map', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(go2_navigation2_dir, 'config', 'nav2_params.yaml'))
    
    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                            description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_path_cmd = launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load')
    
    declare_nav2_param_path_cmd = launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load')
    # nav2启动文件
    nav2_bringup = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    
    # 激光frame_id修改launch文件
    launch_lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_bringup'), '/launch', '/cloud_to_scan.launch.py']),
    )	

    # tf关系发布节点
    node_tftree =  Node(
            package='go2_bringup',                      
            executable='odom2tf',             
            name='odom2tf',
            output='screen',
        )
    
    # 使用自定义odom，进行tf关系发布节点
    start_cus_tftree_node =  Node(
            package='go2_bringup',                      
            executable='motion_to_tf',             
            name='motion_to_tf',
            output='screen',
        )
    
    # 运动控制节点
    node_go2Move =  Node(
            package='go2_bringup',                      
            executable='go2_move',             
            name='go2_move',
            output='screen',
        )
    
    # RViz2节点
    node_rviz =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    
    # 初始化位置节点
    node_initialpose =  Node(
            package='go2_bringup',                      
            executable='initialpose_publisher',             
            name='initialpose_publisher',
            output='screen',
        )

    return launch.LaunchDescription([


        declare_use_sim_time_cmd,
        declare_map_yaml_path_cmd,
        declare_nav2_param_path_cmd,
        launch_lidar,
        # node_tftree,
        start_cus_tftree_node,
        node_go2Move,
        nav2_bringup,
        node_rviz,
        node_initialpose,

    ])