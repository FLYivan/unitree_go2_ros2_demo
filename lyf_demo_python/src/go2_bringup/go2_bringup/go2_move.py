import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from unitree_go.msg import SportModeState                           # 导入SportModeState消息类型，订阅状态信息
from unitree_api.msg import Request                                 # 导入Request消息类型，发送运动指令
from go2_sport.ros2_sport_client import (SportClient,PathPoint)     # 高层运动控制接口API调用 


SPORTTOPIC = "api/sport/request"                                     # 添加运动控制话题


# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('go2_move')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 订阅 cmd_vel 话题
            self.cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Request, SPORTTOPIC, 10)       # 创建运动控制的发布者 
        self.req = Request()                                                    # 初始化Request消息
        self.sport_req = SportClient()                                          # 实例化一个SportClient对象(这里是json封装的client)

    def cmd_vel_callback(self, msg):
        # 获取线速度和角速度
        vx = msg.linear.x
        vy = msg.linear.y
        vyaw = msg.angular.z


        # 将速度发送给实体机器人
        self.send_to_go2(vx, vy, vyaw)

    def send_to_go2(self,vx :float,vy : float,vyaw : float):
        # 在这里调用go2机器人的 API，将速度发送给go2
 

        self.sport_req.Move(self.req,vx, vy, vyaw)                              # 获取与高级运动命令对应的请求消息
        self.cmd_vel_pub.publish(self.req)                                      # 发布速度命令

        self.get_logger().info(f'{YELLOW}Sending to go2: vx ={vx}, vy={vy}, vyaw={vyaw}{RESET}')




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()