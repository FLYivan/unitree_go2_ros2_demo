from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_point_lio_hesai_lidar = FindPackageShare('point_lio_unilidar')
    
    # 创建配置文件路径
    config_file = PathJoinSubstitution([
        pkg_point_lio_hesai_lidar,
        'config',
        'hesai_lidar.yaml'
    ])


    # 雷达驱动launch文件
    start_lidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('hesai_ros_driver'),
            '/launch/start.py'
        ])
    )



    # 传感器矫正节点
    transform_node = Node(
        package='transform_sensors',
        executable='transform_hesai_on_pc2',
        name='transform_hesai_on_pc2',
        output='screen'
    )


    # 点云切割节点
    pointclouds_cut_node = Node(
        package='transform_sensors',
        executable='transform_hesai_process',
        name='transform_hesai_process',
        output='screen'
    )

    # 带点云切割的传感器矫正节点
    sensor_process_node = Node(
        package='transform_sensors',
        executable='sensor_fusion_on_pc2',
        name='sensor_fusion_on_pc2',
        output='screen'
    )


    # 激光雷达建图节点
    mapping_node = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_imu_as_input': False,        # 是否使用IMU作为输入
                'prop_at_freq_of_imu': True,      # 是否以IMU频率进行传播
                'check_satu': True,               # 是否检查IMU饱和
                'init_map_size': 10,              # 初始地图大小
                'point_filter_num': 1,            # 点云滤波数量，可选4或3
                'space_down_sample': True,        # 是否进行空间降采样
                'filter_size_surf': 0.1,          # 表面特征点滤波大小，可选0.5,0.3,0.2,0.15,0.1
                'filter_size_map': 0.1,           # 地图点云滤波大小，可选0.5,0.3,0.15,0.1
                'cube_side_length': 1000.0,       # 体素边长，可选2000
                'runtime_pos_log_enable': False   # 是否启用运行时位置日志记录
            }
        ],
        remappings=[
            ('/cloud_registered', '/registered_scan'),
            ('/aft_mapped_to_init', '/state_estimation')
        ]
    )


    # frame_id重映射节点1
    transform_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubMap',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    )

    # frame_id重映射节点2
    transform_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubVehicle',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'aft_mapped', 'sensor']
    )



    # 返回启动描述
    return LaunchDescription([
        # start_lidar_launch_file,
        # transform_node,             # 二选一

        pointclouds_cut_node,       # 二选一
        sensor_process_node,

        mapping_node,

        transform_node_1,
        transform_node_2,
    ])