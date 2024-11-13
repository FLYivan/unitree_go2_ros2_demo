import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # 激光frame_id修改节点
    node_tftree =  Node(
            package='go2_bringup',                      
            executable='odom2tf',             
            name='odom2tf',
            output='screen',
        )
 

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_bringup'), '/launch', '/cloud_to_scan.launch.py']),
    )	

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_go2_model = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_description'), '/launch', '/go2_real.launch.py']),
    )

    
    
    return launch.LaunchDescription([
        node_tftree,
        launch_lidar,
        launch_go2_model,


    ])
