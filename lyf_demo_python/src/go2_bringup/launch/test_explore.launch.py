import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取与拼接默认路径
    param_dir = get_package_share_directory('go2_bringup')
    explore_dir = get_package_share_directory('explore_lite')
    
    # 创建 Launch 配置
    use_sim_time = LaunchConfiguration(
        'use_sim_time', default='false')

    explore_param_path = LaunchConfiguration(
        'params_file', default=os.path.join(param_dir, 'config', 'explore.yaml'))
    

    
    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                            description='Use simulation (Gazebo) clock if true')

    declare_explore_param_path_cmd = launch.actions.DeclareLaunchArgument(
        'params_file', 
        default_value=explore_param_path,
        description='Full path to param file to load')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[
            explore_param_path, 
            {"use_sim_time": use_sim_time}
            
            ],
        output="screen",
        remappings=remappings,
    )


    return launch.LaunchDescription([


        declare_use_sim_time_cmd,

        declare_explore_param_path_cmd,

        # explore_bringup,

        start_explore_node,
    ])