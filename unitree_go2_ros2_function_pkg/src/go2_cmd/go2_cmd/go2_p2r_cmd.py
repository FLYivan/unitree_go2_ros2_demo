"""
将宇数的python api 作为模块导入到ros2节点中
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)

# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色

class Python2RosCmd(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sport_client = SportClient()  
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # 初始化网络通道
        ChannelFactoryInitialize(0, "wlp0s20f3")
        
        
        # 创建nav2控制指令订阅者
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 订阅 cmd_vel 话题
            self.cmd_vel_callback,
            10
        )


    def cmd_vel_callback(self, msg):
        # 收到移动命令时调用API

        self.sport_client.Move(msg.vx, msg.vy, msg.vyaw)
        self.get_logger().info(f'{YELLOW}Sending to go2: vx ={msg.vx}, vy={msg.vy}, vyaw={msg.vyaw}{RESET}')

def main():
    rclpy.init()
    node = Python2RosCmd('go2_p2r_cmd')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()