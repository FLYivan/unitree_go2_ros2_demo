import launch  # 导入launch模块
import launch_ros  # 导入launch_ros模块
from ament_index_python.packages import get_package_share_directory  # 导入get_package_share_directory函数


def generate_launch_description():  # 定义generate_launch_description函数
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('fishbot_description')  # 获取URDF教程路径
    default_model_path = urdf_tutorial_path + '/urdf/first_robot.urdf'  # 定义默认模型路径
    default_rviz_config_path = urdf_tutorial_path + '/config/rviz/display_model.rviz'  # 定义默认RViz配置路径
    # 为 Launch 壿明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(  # 壿明模型参数
        name='cat', default_value=str(default_model_path),  # 参数名和默认值
        description='URDF 的绝对路径')  # 参数描述
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(  # 生成机器人描述参数
        launch.substitutions.Command(  # 执行命令
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),  # 命令参数
        value_type=str)  # 参数类型
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(  # 发布机器人状态节点
        package='robot_state_publisher',  # 包名
        executable='robot_state_publisher',  # 可执行文件名
        parameters=[{'robot_description': robot_description}]  # 节点参数
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(  # 发布关节状态节点
        package='joint_state_publisher',  # 包名
        executable='joint_state_publisher',  # 可执行文件名
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(  # RViz节点
        package='rviz2',  # 包名
        executable='rviz2',  # 可执行文件名
        arguments=['-d', default_rviz_config_path]  # 节点参数
    )
    return launch.LaunchDescription([  # 返回启动描述
        action_declare_arg_mode_path,  # 模型参数
        joint_state_publisher_node,  # 关节状态发布节点
        robot_state_publisher_node,  # 机器人状态发布节点
        rviz_node  # RViz节点
    ])
