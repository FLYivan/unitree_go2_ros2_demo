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

    sensor_switch_parameters={
        # 选择需要融合的传感器类型
          'sync_rgb':                   False,                     # rgb图像数据
          'sync_depth':                 False,                     # 深度信息数据
          'sync_camera_info':           True,                     # 相机内参数据
          'sync_pointcloud':            False,                     # 点云数据
          'sync_imu':                   False,                    # IMU数据
          'sync_scan':                  True,                     # 激光扫描数据
          'sync_rgb_compressed':        True,                    # 压缩图像数据
          'sync_depth_compressed':      True,                    # 压缩深度信息数据
          
    }

    # 激光frame_id修改launch文件
    start_lidar_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_lidar_processing'), '/launch', '/cloud_to_scan.launch.py']),
    )	

    
    # 使用自定义odom，进行tf关系发布节点
    start_cus_tftree_node =  Node(
            package='go2_slam_algorithm',                      
            executable='motion_to_tf',             
            name='motion_to_tf',
            output='screen',
        )
    


    # go2的URDF发布launch文件
    start_urdf_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_sim'), '/launch', '/go2_urdf2tf.launch.py']),
    )	


    # 传感器时间戳同步节点
    start_sensor_sync_node =  Node(
            package='go2_lidar_processing',                      
            executable='sensor_sync_node',             
            name='sensor_sync_node',
            output='screen',
            parameters=[sensor_switch_parameters],
        )


    return launch.LaunchDescription([
 
  
        start_lidar_launch_file,        # 启动激光frame_id修改launch文件
        start_urdf_launch_file,


        start_cus_tftree_node,
        start_sensor_sync_node,



    ])
