
import os  # 导入os模块

from ament_index_python.packages import get_package_share_directory                 # 导入get_package_share_directory函数


from launch import LaunchDescription  # 导入LaunchDescription类
from launch.actions import IncludeLaunchDescription,ExecuteProcess # 导入IncludeLaunchDescription类
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入PythonLaunchDescriptionSource类
from launch_ros.actions import Node  # 导入Node类


def generate_launch_description():  # 定义generate_launch_description函数


    package_name='go2_description'                                         # 设置包名 #<--- CHANGE ME
    world_file_path = 'world/custom_room.world'                    # 设置世界文件路径
    
    pkg_path = os.path.join(get_package_share_directory(package_name))  # 获取包路径
    world_path = os.path.join(pkg_path, world_file_path)                # 获取世界文件路径  
    
    # Pose where we want to spawn the robot  # 设置机器人初始姿态
    spawn_x_val = '0.0'  # 设置x坐标
    spawn_y_val = '0.0'  # 设置y坐标
    spawn_z_val = '0.0'  # 设置z坐标
    spawn_yaw_val = '0.0'  # 设置yaw角度
  
    """
    以下部分对机器人模型进行配置
    """

    go2_robot = IncludeLaunchDescription(                                                       # 包含go2启动文件
                PythonLaunchDescriptionSource([os.path.join(                                    # 获取go2启动文件路径
                    get_package_share_directory('go2_description'),'launch','go2_rviz.launch.py'     # 获取包路径和启动文件名
                )]), 
                launch_arguments={'use_sim_time': 'true', 'world':world_path}.items()      # 设置启动参数
                # launch_arguments={'use_sim_time': 'true'}.items()      # 设置启动参数
    )
    
 

    """
    以下部分开始加载机器人模型文件到gazebo
    """

    # 包含Gazebo启动文件
    gazebo = IncludeLaunchDescription(  
                PythonLaunchDescriptionSource([os.path.join(                                        # 获取Gazebo启动文件路径
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),     #启动一个空环境，把gazebo模型先跑起来
             )

    # 运行spawn_entity节点
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',   #ros2提供的一个spawn_entity节点，作用是加载模型
                        arguments=['-topic', '/robot_description',    #模型文件的完整路径
                                   '-entity', 'go2_description',   #机器人名
                                   '-x', spawn_x_val,  #设置x坐标
                                   '-y', spawn_y_val,  #设置y坐标
                                   '-z', spawn_z_val,  #设置z坐标
                                   '-Y', spawn_yaw_val],    #设置yaw角度
                        output='screen')  #设置输出
    

    node_static_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0',
                '--qx', '0', 
                '--qy', '0', 
                '--qz', '0', 
                '--qw', '1',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link'
                ],
            output='screen'
        )


    # 启动所有节点
    return LaunchDescription([  # 返回启动描述
        go2_robot,                          # 启动go2_robot节点
        gazebo,                             # 启动Gazebo节点
        spawn_entity,                       # 启动spawn_entity节点
        node_static_transform,              # 静态变换发布器节点
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/Bool', 'data: true'],
        #     output='screen'
        # ),
    ])
