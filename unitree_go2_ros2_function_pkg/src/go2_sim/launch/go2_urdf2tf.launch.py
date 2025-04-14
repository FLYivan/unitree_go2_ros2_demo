import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('go2_sim')
    go2_model_path = urdf_tutorial_path + '/urdf/go2_description.urdf'
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(go2_model_path),
        description='URDF 的绝对路径')
    # rviz文件路径
    rviz_config_dir = os.path.join(
        urdf_tutorial_path, 'rviz', 'go2_rviz.rviz')
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
    # RViz2节点
    rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', rviz_config_dir  # 设置指定rviz文件路径
                ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )


    return launch.LaunchDescription([
        action_declare_arg_mode_path, 
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node,
    ])