from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    go2_slam_dir = FindPackageShare('go2_slam_algorithm')
    go2_nav_dir = get_package_share_directory('go2_nav')
    go2_explore_dir = get_package_share_directory('go2_explore_algorithm')

    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    rviz_config_dir = os.path.join(
        go2_nav_dir, 'rviz', 'nav2_go2_view.rviz')

    
    use_sim_time = LaunchConfiguration('use_sim_time')


    explore_param_path = LaunchConfiguration(
        'params_file', default=os.path.join(go2_explore_dir, 'config', 'explore.yaml'))
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    declare_explore_param_path_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=explore_param_path,
        description='Full path to param file to load')


    # slam启动文件路径
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([go2_slam_dir, 'launch', 'go2_slamtoolbox_for_explore.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # nav2启动文件路径
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': explore_param_path,
        }.items(),
    )


    # 自探索功能节点
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    

    start_explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        arguments=['--ros-args', '--log-level', 'ExploreNode:=DEBUG'],
        parameters=[
            explore_param_path, 
            {"use_sim_time": use_sim_time}
            
            ],
        output="screen",
        remappings=remappings,
    )

 # 延迟启动 node_frame
    delayed_start_explore_node = TimerAction(period=10.0,  actions=[start_explore_node]) # 延迟 10 秒启动

    # 运动控制节点
    start_go2Move_node =  Node(
            package='go2_cmd',                      
            executable='go2_move',             
            name='go2_move',
            output='screen',
        )


    # RViz2节点
    start_rviz_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                # '--ros-args', '--log-level', 'WARN',  # 设置日志级别为 WARN
                '-d', rviz_config_dir  # 设置指定rviz文件路径

                ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',

        )

    return LaunchDescription(
        [
            declare_explore_param_path_cmd,
            declare_use_sim_time_cmd,
            slam_launch,                    # 启动slam功能
            nav2_bringup_launch,            # 启动nav2功能
            # start_explore_node,            # 启动探索节点
            delayed_start_explore_node,       # 延迟启动探索节点
            start_go2Move_node,             # 运动控制节点
            start_rviz_node,
        ]
    )