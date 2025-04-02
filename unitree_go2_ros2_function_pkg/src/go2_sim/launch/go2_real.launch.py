
import os  # 导入os模块
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory                 # 导入get_package_share_directory函数


from launch import LaunchDescription  # 导入LaunchDescription类
from launch.actions import IncludeLaunchDescription,ExecuteProcess # 导入IncludeLaunchDescription类
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入PythonLaunchDescriptionSource类
from launch_ros.actions import Node  # 导入Node类


def generate_launch_description():  # 定义generate_launch_description函数


    # 获取默认路径

    go2_urdf_path = get_package_share_directory('go2_description')
    go2_model_path = go2_urdf_path + '/urdf/go2_description.urdf'

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(go2_model_path),
        description='URDF 的绝对路径')
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path, 
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])

    