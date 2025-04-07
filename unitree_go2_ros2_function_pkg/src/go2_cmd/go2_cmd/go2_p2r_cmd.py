"""
将宇数的python api 作为模块导入到ros2节点中
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色

class Python2RosCmd(Node):
    def __init__(self,name):
        super().__init__(name)
        

        # 定义网口名称参数
        self.declare_parameter('channel_name', "test")  # 参数名为'my_param'，默认值为10
        # 获取参数值
        channel_name_value = self.get_parameter('channel_name').get_parameter_value().string_value

        # 初始化网络通道
        ChannelFactoryInitialize(0, channel_name_value)

        self.get_logger().info(f'{YELLOW}通道初始化成功{RESET}')

        # 初始化客户端
        self.sport_client = SportClient()  
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.get_logger().info(f'{RED}客户端初始化成功{RESET}')
        
        # 创建nav2控制指令订阅者
        self.move_subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 订阅 cmd_vel 话题
            self.cmd_vel_callback,
            10
        )


        # 创建订阅者，监听其他控制指令
        self.cmd_subscription = self.create_subscription(
            String,
            'robot_commands',  # 订阅的其他指令话题
            self.cmd_callback,
            10
        )

        # 初始化nav2获取的速度变量
        self.nav2_vx = 0.0
        self.nav2_vy = 0.0
        self.nav2_vyaw = 0.0

    def cmd_vel_callback(self, msg):
        # 收到移动命令时调用API

        self.nav2_vx = msg.linear.x
        self.nav2_vy = msg.linear.y
        self.nav2_vyaw = msg.angular.z

        # 移动指令
        self.move_lower_body(self.nav2_vx, self.nav2_vy, self.nav2_vyaw)
        
        # self.sport_client.Move(self.nav2_vx, self.nav2_vy, self.nav2_vyaw)

        

    def cmd_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # 根据接收到的指令执行不同的动作
        if command == "stand_up":
            self.stand_up()
        elif command == "sit_down":
            self.sit_down()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def move_lower_body(self, vx, vy, vyaw):

        self.sport_client.Move(vx, vy, vyaw)

        # 打印速度日志
        self.get_logger().info(f'{YELLOW}Sending to go2: vx ={vx}, vy={vy}, vyaw={vyaw}{RESET}')

    def stand_up(self):
        # 实现站起的逻辑
        self.get_logger().info('Standing up...')
        # 这里可以调用具体的站起控制函数

    def stand_down(self):
        
        # 先停止移动
        self.sport_client.StopMove()

        # 实现趴下的逻辑
        self.sport_client.StandDown()

        self.get_logger().info('趴下...')



def main():
    rclpy.init()
    node = Python2RosCmd('go2_p2r_cmd')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()