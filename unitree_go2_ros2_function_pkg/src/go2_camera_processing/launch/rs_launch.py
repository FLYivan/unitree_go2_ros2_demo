# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""启动 realsense2_camera 节点"""
import os
from launch import LaunchDescription  # 导入launch描述相关类
from ament_index_python.packages import get_package_share_directory  # 导入获取包共享目录的函数
import launch_ros.actions  # 导入launch ROS动作
from launch.actions import DeclareLaunchArgument  # 导入声明launch参数的动作
from launch.substitutions import LaunchConfiguration, PythonExpression  # 导入launch配置和Python表达式
from launch.conditions import IfCondition  # 导入条件判断


# 可配置参数列表,每个参数包含名称、默认值和描述
configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': '相机唯一名称'},
                           {'name': 'serial_no',                    'default': "''", 'description': '通过序列号选择设备'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': '通过USB端口ID选择设备'},
                           {'name': 'device_type',                  'default': "''", 'description': '通过类型选择设备'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml配置文件'},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-无, 1-复制, 2-线性插值]'},
                           {'name': 'json_file_path',               'default': "''", 'description': '允许高级配置'},
                           {'name': 'log_level',                    'default': 'info', 'description': '调试日志级别[DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': '节点输出[screen|log]'},
                           {'name': 'depth_module.profile',         'default': '0,0,0', 'description': '深度模块配置'},                           
                           {'name': 'enable_depth',                 'default': 'true', 'description': '启用深度流'},
                           {'name': 'rgb_camera.profile',           'default': '0,0,0', 'description': '彩色图像宽度'},
                           {'name': 'enable_color',                 'default': 'true', 'description': '启用彩色流'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': '启用红外1流'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': '启用红外2流'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': '启用红外RGB流'},
                           {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': '鱼眼宽度'},
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': '启用鱼眼1流'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': '启用鱼眼2流'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': '启用深度流'},
                           {'name': 'gyro_fps',                     'default': '0', 'description': '陀螺仪帧率'},                           
                           {'name': 'accel_fps',                    'default': '0', 'description': '加速度计帧率'},                           
                           {'name': 'enable_gyro',                  'default': 'false', 'description': '启用陀螺仪'},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': '启用加速度计'},                           
                           {'name': 'enable_pose',                  'default': 'true', 'description': '启用位姿'},                           
                           {'name': 'pose_fps',                     'default': '200', 'description': '位姿帧率'},                           
                           {'name': 'pointcloud.enable',            'default': 'false', 'description': '启用点云'}, 
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': '点云的纹理流'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': '点云的纹理流索引'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': '启用同步'},                           
                           {'name': 'align_depth.enable',           'default': 'false', 'description': '启用深度对齐'},                           
                           {'name': 'colorizer.enable',             'default': 'false', 'description': '启用着色器'},
                           {'name': 'clip_distance',                'default': '-2.', 'description': '裁剪距离'},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': '线性加速度协方差'},                           
                           {'name': 'initial_reset',                'default': 'false', 'description': '初始重置'},                           
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': '允许无纹理点'},                           
                           {'name': 'ordered_pc',                   'default': 'false', 'description': '有序点云'},
                           {'name': 'calib_odom_file',              'default': "''", 'description': '里程计标定文件'},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'T265轮式里程计话题'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '发布static_tf的频率'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': '发布诊断的频率. 0=禁用'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': '启用抽取滤波器'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': '用作设备的realsense包文件'},
                           {'name': 'depth_module.exposure.1',     'default': '7500', 'description': 'HDR合并滤波器的初始值'},
                           {'name': 'depth_module.gain.1',         'default': '16', 'description': 'HDR合并滤波器的初始值'},
                           {'name': 'depth_module.exposure.2',     'default': '1', 'description': 'HDR合并滤波器的初始值'},
                           {'name': 'depth_module.gain.2',         'default': '16', 'description': 'HDR合并滤波器的初始值'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': '等待设备连接的超时时间(秒)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': '连续重连尝试之间的超时时间(秒)'},
                          ]

def declare_configurable_parameters(parameters):
    # 为每个参数创建launch参数声明
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    # 创建参数字典,将每个参数名映射到其launch配置
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    log_level = 'info'
    # 根据ROS发行版选择不同的launch配置
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense节点 - 无配置文件情况
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ),
            # Realsense节点 - 有配置文件情况
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters=[set_configurable_parameters(configurable_parameters)
                            , PythonExpression([LaunchConfiguration("config_file")])
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                ),
            ])
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense节点 - 无配置文件情况
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                ),
            # Realsense节点 - 有配置文件情况
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            , PythonExpression([LaunchConfiguration("config_file")])
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                ),
        ])