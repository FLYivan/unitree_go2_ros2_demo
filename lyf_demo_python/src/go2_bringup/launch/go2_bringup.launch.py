import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    slam_params_file = LaunchConfiguration('slam_params_file')

    # 获取rviz配置文件路径
    rviz_file = os.path.join(
        get_package_share_directory('go2_bringup'),
        'rviz',
        'dog_slam_simp.rviz'
    )

    # 声明slam-toolbox启动时，参数文件的默认地址
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("go2_bringup"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')


    # tf关系发布节点
    node_tftree =  Node(
            package='go2_bringup',                      
            executable='odom2tf',             
            name='odom2tf',
            output='screen',
        )
    
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
 

    # 激光frame_id修改launch文件
    launch_lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_bringup'), '/launch', '/cloud_to_scan.launch.py']),
    )	


    # # go2实体模型launch文件
    # launch_go2_model = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory(
    #         'go2_description'), '/launch', '/go2_real.launch.py']),
    # )

    # RViz2节点
    node_rviz =Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file]
        )


    # 启动rqt
    pass
    
    
    return launch.LaunchDescription([
        node_tftree,                    # 启动tf关系发布节点
        launch_lidar,                   # 启动激光frame_id修改launch文件
        # launch_go2_model,               # 启动go2实体模型launch文件
        node_rviz,                      # 启动RViz2节点
        declare_slam_params_file_cmd,
        start_async_slam_toolbox_node,  # slam-toolbox算法节点


    ])
