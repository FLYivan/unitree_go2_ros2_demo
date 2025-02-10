import rclpy  # 导入ROS2 Python客户端库

from rclpy.node import Node  # 从rclpy中导入Node类
from unitree_go.msg import SportModeState  # 导入SportModeState消息类型
from unitree_api.msg import Request  # 导入Request消息类型
from go2_sport.ros2_sport_client import (SportClient,PathPoint)

class SportDemo(Node):
    def __init__(self):
        super().__init__('sport_demo')

        
        self.dt = 5.0                                                                 # 设置控制时间步长
        
        self.req = Request()                                                            # 初始化Request消息
        self.sport_req = SportClient()                                                  # 实例化一个SportClient对象(这里是json封装的client)
        self.t = -1                                                                     # 初始化运行时间计数
     
        self.vx = 0.0                                                                    # 初始化初始x位置
        self.vy = 0.0                                                                    # 初始化初始y位置
        self.vyaw = 0.0                                                                  # 初始化初始偏航角


        self.req_puber = self.create_publisher(Request, '/api/sport/request', 10)                               # 创建'/api/sport/request'的发布者

        


        self.timer = self.create_timer(self.dt, self.timer_callback)                    # 创建一个定时器，每0.05秒（create_time单位秒）调用一次timer_callback函数

    def timer_callback(self):
        self.t += self.dt                                                               # 等待1秒后启动
        if self.t >= 0:                                                                 # 检查运行时间计数是否为非负
 
            # self.VelocityMove()
            # self.Start()
            # self.Relax()
            self.Stop()
            # 后续在这里继续添加其他运动测试方法

        elif self.t >=30:
            self.Stop()

    def VelocityMove(self):
  
        self.vx = 0.0                                                                    # 初始化初始x位置
        self.vy = 0.0                                                                    # 初始化初始y位置
        self.vyaw = 0.0                                                                  # 初始化初始偏航角
        self.sport_req.Move(self.req, self.vx, self.vy, self.vyaw)                  # 获取与高级运动命令对应的请求消息
        self.req_puber.publish(self.req)                                            # 发布请求消息


    
    def Relax(self):

        # # 坐下
        # self.sport_req.Sit(self.req)
        # self.req_puber.publish(self.req) 

        # # 站起（相对于坐下）
        # self.sport_req.RiseSit(self.req)
        # self.req_puber.publish(self.req) 


        # # 伸懒腰Stretch()
        # self.sport_req.Stretch(self.req)
        # self.req_puber.publish(self.req) 

        # 切换步态 (只要一次，后面还要切回正常步态)
        self.sport_req.SwitchGait(self.req,3)
        self.req_puber.publish(self.req) 

        # 切换步态后要move


    def Start(self):

        # 打招呼
        self.sport_req.Hello(self.req)
        self.req_puber.publish(self.req)  



    def Stop(self):

        # self.sport_req.StopMove(self.req)                                           # 获取与高级运动命令对应的请求消息
        # self.req_puber.publish(self.req)                                            # 发布请求消息

        # 拜年(只能一次性)
        self.sport_req.Scrape(self.req)
        self.req_puber.publish(self.req) 

def main(args=None):
    rclpy.init(args=args)                                                                   # 初始化rclpy
    sport_demo_node = SportDemo()                                                           # 创建SportRequest节点的实例
    rclpy.spin(sport_demo_node)                                                             # 保持节点运行

    rclpy.shutdown()                                                                        # 关闭rclpy

if __name__ == '__main__':
    main()      