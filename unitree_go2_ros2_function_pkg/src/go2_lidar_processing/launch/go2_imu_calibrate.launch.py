import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # 宇树状态接口转自带IMU类型话题
    node_imu_pub_node = Node(
            package='go2_lidar_processing',
            executable='go2_imu_node',
            name='go2_imu_node',
            output='screen',
        )

    # 激光frame_id修改节点
    node_imu_calibrate_node =  Node(
            package='calibrate_imu',                      
            executable='calibrate_go2_imu',             
            name='calibrate_go2_imu',
            output='screen',
        )

    # 延迟启动 node_frame
    delayed_node = TimerAction(period=2.0,  actions=[node_imu_calibrate_node]) # 延迟 2 秒启动


    return LaunchDescription([
        node_imu_pub_node,
        delayed_node,
    ])



