import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # 接收串行口数据并发送运控指令节点
    start_cmd_node = Node(
            package='go2_demo_show',
            executable='brain_interface_cmd',
            name='brain_interface_cmd',
        )

    # 摄像头拉流节点
    start_camera_node =  Node(
            package='go2_camera_processing',                      
            executable='video_stream',             
            name='video_stream',
            output='screen',
            parameters=[{
                'multicast_iface': 'wlp0s20f3',
                # 'multicast_iface': 'enp0s31f6',
     
                         }]       
        )

    # 延迟启动 start_camera_node
    delayed_node = TimerAction(period=3.0,  actions=[start_camera_node]) # 延迟 3 秒启动


    return LaunchDescription([
        start_cmd_node,
        delayed_node,
    ])



