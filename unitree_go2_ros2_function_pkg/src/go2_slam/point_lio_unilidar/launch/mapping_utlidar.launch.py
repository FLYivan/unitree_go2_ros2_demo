from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_point_lio_unilidar = FindPackageShare('point_lio_unilidar')
    
    # 创建配置文件路径
    config_file = PathJoinSubstitution([
        pkg_point_lio_unilidar,
        'config',
        'utlidar.yaml'
    ])

    # 获取rviz配置文件路径
    rviz_file = PathJoinSubstitution([
        pkg_point_lio_unilidar,
        'rviz_cfg',
        'brain_interface_traj.rviz'
    ])


    # 传感器矫正节点
    transform_node = Node(
        package='transform_sensors',
        executable='transform_everything',
        name='transform_everything',
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
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 1,        # 4, 3
                'space_down_sample': True,
                'filter_size_surf': 0.1,      # 0.5, 0.3, 0.2, 0.15, 0.1
                'filter_size_map': 0.1,       # 0.5, 0.3, 0.15, 0.1
                'cube_side_length': 1000.0,   # 2000
                'runtime_pos_log_enable': False
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



    # RViz2节点
    start_rviz_node =Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            # parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    # 返回启动描述
    return LaunchDescription([
        transform_node,
        mapping_node,
        start_rviz_node,
        transform_node_1,
        transform_node_2,
    ])