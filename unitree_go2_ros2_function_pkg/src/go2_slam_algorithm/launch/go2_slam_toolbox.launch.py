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

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_params_file = LaunchConfiguration('rviz_params_file')

    # 获取launch文件和定义地址
    bringup_dir = get_package_share_directory("go2_slam_algorithm")

    

    sensor_switch_parameters={
        # 选择需要融合的传感器类型
          'sync_rgb':                   False,                     # rgb图像数据
          'sync_depth':                 False,                     # 深度信息数据
          'sync_camera_info':           False,                     # 相机内参数据
          'sync_pointcloud':            False,                     # 点云数据
          'sync_imu':                   False,                    # IMU数据
          'sync_scan':                  True,                     # 激光扫描数据
          'sync_rgb_compressed':        False,                    # 压缩图像数据
          'sync_depth_compressed':      False,                    # 压缩深度信息数据
          
    }

    # 声明slam-toolbox启动时，参数文件的默认地址
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_dir,'config', 'mapper_params_online_async.yaml'),                
        description='Full path to the slam parameters file to use for the slam_toolbox node')
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    # 获取rviz配置文件路径
    declare_rviz_params_file_cmd = DeclareLaunchArgument(
        'rviz_params_file',
        default_value=os.path.join(
        bringup_dir,
        'rviz',
        'dog_slam_simp.rviz'
        ),
        description='use slam rviz file')

    
    
    # 使用自定义odom，进行tf关系发布节点
    start_cus_tftree_node =  Node(
            package='go2_slam_algorithm',                      
            executable='motion_to_tf',             
            name='motion_to_tf',
            output='screen',
        )
    
    # slam-toolbox节点
    start_async_slam_toolbox_node = Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            # remappings=[
            #         ('/map', '/map_slamtoolbox_go2')  # 将 /map 重映射为 /map_slamtoolbox_go2
            #          ]
        )
 
    # 激光frame_id修改launch文件
    start_lidar_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_lidar_processing'), '/launch', '/cloud_to_scan.launch.py']),
    )	


    # L1点云转2d激光
    start_L1_lidar_launch_file = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'go2_lidar_processing'), '/launch', '/cloud_to_scan_L1.launch.py']),
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

    # RViz2节点
    start_rviz_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_params_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    # 运动轨迹显示节点
    start_traject_node =  Node(
        package='go2_slam_algorithm',                      
        executable='trajectory_visualizer',             
        name='trajectory_visualizer',
        output='screen',
    )



    return launch.LaunchDescription([

        
        declare_slam_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_rviz_params_file_cmd,
  
        # start_L1_lidar_launch_file,     # 启动L1点云转2d激光文件


        start_lidar_launch_file,        # 启动点云转扫描数据文件
        start_cus_tftree_node,
        start_async_slam_toolbox_node,  # slam-toolbox算法节点
        start_rviz_node,
        start_urdf_launch_file,         # urdf中各种静态tf关系
        start_traject_node,             # 运动轨迹显示节点


        # start_sensor_sync_node,


    ])
