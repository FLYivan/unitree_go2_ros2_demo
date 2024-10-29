
import os  # 导入os模块

from ament_index_python.packages import get_package_share_directory  # 导入get_package_share_directory函数


from launch import LaunchDescription  # 导入LaunchDescription类
from launch.actions import IncludeLaunchDescription  # 导入IncludeLaunchDescription类
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入PythonLaunchDescriptionSource类

from launch_ros.actions import Node  # 导入Node类


def generate_launch_description():  # 定义generate_launch_description函数


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled  # 包含robot_state_publisher启动文件
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!  # 确保设置正确的包名

    package_name='learning_gazebo'  # 设置包名 #<--- CHANGE ME
    world_file_path = 'worlds/custom_room.world'  # 设置世界文件路径
    
    pkg_path = os.path.join(get_package_share_directory(package_name))  # 获取包路径
    world_path = os.path.join(pkg_path, world_file_path)  # 获取世界文件路径  
    
    # Pose where we want to spawn the robot  # 设置机器人初始姿态
    spawn_x_val = '0.0'  # 设置x坐标
    spawn_y_val = '0.0'  # 设置y坐标
    spawn_z_val = '0.0'  # 设置z坐标
    spawn_yaw_val = '0.0'  # 设置yaw角度
  
    """
    以下部分对机器人模型进行配置
    """

    mbot = IncludeLaunchDescription(  # 包含mbot启动文件
                PythonLaunchDescriptionSource([os.path.join(  # 获取mbot启动文件路径
                    get_package_share_directory(package_name),'launch','mbot.launch.py'  # 获取包路径和启动文件名
                )]), launch_arguments={'use_sim_time': 'true', 'world':world_path}.items()  # 设置启动参数
    )

    """
    以下部分开始加载机器人模型文件到gazebo
    """

    # Include the Gazebo launch file, provided by the gazebo_ros package  # 包含Gazebo启动文件
    gazebo = IncludeLaunchDescription(  # 包含Gazebo启动文件
                PythonLaunchDescriptionSource([os.path.join(  # 获取Gazebo启动文件路径
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),   #启动一个空环境，把gazebo模型先跑起来
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.  # 运行spawn_entity节点
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',   #ros2提供的一个spawn_entity节点，作用是加载模型
                        arguments=['-topic', 'robot_description',    #模型文件的完整路径
                                   '-entity', 'mbot',   #机器人名
                                   '-x', spawn_x_val,  #设置x坐标
                                   '-y', spawn_y_val,  #设置y坐标
                                   '-z', spawn_z_val,  #设置z坐标
                                   '-Y', spawn_yaw_val],    #设置yaw角度
                        output='screen')  #设置输出


    # Launch them all!  # 启动所有节点
    return LaunchDescription([  # 返回启动描述
        mbot,  # 启动mbot节点
        gazebo,  # 启动Gazebo节点
        spawn_entity,  # 启动spawn_entity节点
    ])
