#
#
# Example with gazebo:
#   1) Launch simulator (husky):
#     $ ros2 launch clearpath_gz simulation.launch.py
#     Click on "Play" button on bottom-left of gazebo as soon as you can see it to avoid controllers crashing after 5 sec.
#
#   2) Launch rviz:
#     $ ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
#
#   3) Launch SLAM:
#     $ ros2 launch rtabmap_demos husky_slam3d_assemble.launch.py use_sim_time:=true
#
#   4) Launch nav2"
#     $ ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true
#
#   4) Click on "Play" button on bottom-left of gazebo.
#
#   5) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/a200_0000/cmd_vel
#

# 导入launch相关的类和函数
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 定义launch配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_ns = LaunchConfiguration('robot_ns')

    # ICP里程计参数配置
    icp_odom_parameters={
          'odom_frame_id':'icp_odom',          # 里程计坐标系ID
          'guess_frame_id':'odom_go2',             # 初始位姿估计坐标系ID
          'OdomF2M/ScanSubtractRadius': '0.3', # match voxel size  # 扫描点云降采样半径
          'OdomF2M/ScanMaxSize': '10000'       # 扫描点云最大点数
    }

    # RTAB-Map SLAM参数配置
    rtabmap_parameters={
          'subscribe_rgbd':True,               # 订阅RGBD数据
          'subscribe_depth':False,             # 不订阅深度图
          'subscribe_rgb':False,               # 不订阅RGB图像
          'subscribe_scan_cloud':True,         # 订阅激光点云
          'use_action_for_goal':True,          # 使用action实现目标点导航
          'odom_sensor_sync': True,            # 传感器数据同步
          'topic_queue_size': 30,              # 话题队列大小
          'sync_queue_size': 30,               # 同步队列大小
          'approx_sync': True,                 # 使用近似同步
          'qos': 1,                            # 服务质量等级
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false',    # 不保留未连接的节点
          'Grid/RangeMin':'0.5',               # ignore laser scan points on the robot itself  # 最小激光范围
          'Grid/NormalsSegmentation':'false',  # Use passthrough filter to detect obstacles   # 不使用法向量分割
          'Grid/MaxGroundHeight':'0.05',       # All points above 5 cm are obstacles         # 地面最大高度
          'Grid/MaxObstacleHeight':'1',        # All points over 1 meter are ignored         # 障碍物最大高度
          'Grid/RayTracing':'true',            # Fill empty space                            # 启用射线追踪
          'Grid/3D':'false',                   # Use 2D occupancy                            # 使用2D占据栅格
          'RGBD/OptimizeMaxError':'0.3',       # There are a lot of repetitive patterns...   # 回环检测最大误差
          'Rtabmap/DetectionRate': '0'         # Rate is limited by the assembling time...   # 检测频率
    }

    # 共享参数配置
    shared_parameters={
          'frame_id':'base',              # 基准坐标系
          'use_sim_time':use_sim_time,         # 是否使用仿真时间
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',                  # 配准策略
          'Reg/Force3DoF':'true',             # we are moving on a 2D flat floor  # 强制3自由度运动
          'Mem/NotLinkedNodesKept':'false',    # 不保留未连接节点
          'Icp/VoxelSize': '0.3',              # ICP体素大小
          'Icp/MaxCorrespondenceDistance': '3', # roughly 10x voxel size  # 最大对应点距离
          'Icp/PointToPlaneGroundNormalsUp': '0.9',  # 地面法向量阈值
          'Icp/RangeMin': '0.5',               # ICP最小范围
          'Icp/MaxTranslation': '2'            # ICP最大平移距离
    }

    # 话题重映射配置
    remappings=[
          ('/tf', 'tf'),                       # TF坐标变换
          ('/tf_static', 'tf_static'),         # 静态TF变换
          ('odom', 'icp_odom'),                # 里程计话题
          ('rgb/image', 'sync/rgb/image'),           # RGB图像话题
          ('rgb/camera_info', 'sync/rgb/camera_info'), # 相机信息话题
          ('depth/image', 'sync/depth/image')]         # 深度图像话题

    return LaunchDescription([

        # Launch arguments  # 启动参数
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),  # 是否使用仿真时间
        
        DeclareLaunchArgument(
            'robot_ns', default_value='go2',
            description='Robot namespace.'),  # 机器人命名空间

        # Nodes to launch  # 启动节点
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',  # RGBD同步节点
            namespace=robot_ns,
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',  # ICP里程计节点
            namespace=robot_ns,
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'sync/points')],
            arguments=["--ros-args", "--log-level", 'warn']),
        
        #Assemble scans  # 点云组装节点
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            namespace=robot_ns,
            parameters=[{'assembling_time': 1.0, 'range_min': 0.5, 'fixed_frame_id': "", 'use_sim_time':use_sim_time, 'sync_queue_size': 30, 'topic_queue_size':30}],
            remappings=remappings + [('cloud', 'sync/points')]),

        # SLAM Mode:  # SLAM模式节点
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',  # RTAB-Map SLAM主节点
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'assembled_cloud')],
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',  # RTAB-Map可视化节点
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'sync/points')]),
    ])
