#
# Requirements: 
#   - Install: ros-$ROS_DISTRO-clearpath-simulator ros-$ROS_DISTRO-clearpath-nav2-demos ros-$ROS_DISTRO-clearpath-config ros-$ROS_DISTRO-moveit-setup-srdf-plugins
#   - Copy /opt/ros/humble/share/clearpath_config/sample/a200_sample.yaml to ~/clearpath/robot.yaml  
#   - Fix camera intrinsics by editing /opt/ros/humble/share/clearpath_sensors_description/urdf/intel_realsense.urdf.xacro:
#        <horizontal_fov>1.047</horizontal_fov>
#        <image>
#           <width>320</width>
#           <height>240</height>
#        </image>
#
# Example with gazebo:
#   1) Launch simulator (husky, nav2 and rtabmap):
#     $ ros2 launch rtabmap_demos husky_sim_scan3d_assemble_demo.launch.py robot_ns:=a200_0000
#
#   2) Click on "Play" button on bottom-left of gazebo as soon as you can see it to avoid controllers crashing after 5 sec.
#
#   3) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/a200_0000/cmd_vel
#

# 导入获取包共享目录的函数
from ament_index_python.packages import get_package_share_directory

# 导入launch相关的类和函数
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import os

# 定义启动参数
ARGUMENTS = [
    DeclareLaunchArgument('rtabmap_viz', default_value='true',
                          choices=['true', 'false'], description='Start rtabmap_viz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('robot_ns', default_value='go2',
                          description='Robot namespace'),
]

def generate_launch_description():
    # Directories
    # 获取各个包的共享目录路径
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')
    pkg_clearpath_viz = get_package_share_directory(
        'clearpath_viz')
    
    pkg_rtabmap_demos = get_package_share_directory(
        'go2_slam_algorithm')
    

    pkg_clearpath_nav2_demos = get_package_share_directory(
        'clearpath_nav2_demos')
    
    # Paths
    # 定义各个launch文件的路径
    sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
    viz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])
    
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap_demos, 'launch', 'slam3d_assemble_go2.launch.py'])
    

    nav2_launch = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])

    # 配置仿真启动
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
        ]
    )

    # 配置可视化启动
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([viz_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('robot_ns')),
        ]
    )

    # 配置RTAB-Map启动
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('rtabmap_viz', LaunchConfiguration('rtabmap_viz')),
            ('use_sim_time', 'false'),
            ('robot_ns', LaunchConfiguration('robot_ns'))
        ]
    )

    # 配置导航启动
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            # ('setup_path', os.path.expanduser('~')+'/clearpath/'),
            ('setup_path', pkg_rtabmap_demos + '/data/clearpath/'),
            ('use_sim_time', 'true'),
        ]
    )

    # Create launch description and add actions
    # 创建启动描述并添加动作
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rtabmap)
    # ld.add_action(sim)
    # ld.add_action(viz)
    # ld.add_action(nav2)
    return ld
