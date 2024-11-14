import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    slam_params_file = LaunchConfiguration('slam_params_file')


    # 声明slam-toolbox启动时，参数文件的默认地址
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("go2_bringup"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')


    
    # slam-toolbox节点
    start_async_slam_toolbox_node = Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': False}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[
                    ('/map', '/map_slamtoolbox_go2')  # 将 /map 重映射为 /map_slamtoolbox_go2
                     ],
        )
 




    
    
    return launch.LaunchDescription([

        declare_slam_params_file_cmd,
        start_async_slam_toolbox_node,  # slam-toolbox算法节点


    ])
