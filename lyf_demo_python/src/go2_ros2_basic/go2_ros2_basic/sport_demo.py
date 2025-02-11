import rclpy  # 导入ROS2 Python客户端库
import time
import threading

from rclpy.node import Node  # 从rclpy中导入Node类
from unitree_go.msg import SportModeState  # 导入SportModeState消息类型
from unitree_api.msg import Request  # 导入Request消息类型
from go2_sport.ros2_sport_client import (SportClient,PathPoint)

class SportDemo(Node):
    def __init__(self):
        super().__init__('sport_demo')

        self.req = Request()                                                            # 初始化Request消息
        self.sport_req = SportClient()                                                  # 实例化一个SportClient对象(这里是json封装的client)
        # 初始化发布者
        self.req_puber = self.create_publisher(Request, '/api/sport/request', 10)                               # 创建'/api/sport/request'的发布者


        self.vx = 0.0                                                                    # 初始化初始x位置
        self.vy = 0.0                                                                    # 初始化初始y位置
        self.vyaw = 0.0                                                                  # 初始化初始偏航角

        self.start_enabled = True
        self.relax_flag = True                                                        # 初始化休息标签
        self.sit_enabled = True
        self.resit_enabled = True
        self.change_enabled = True
        self.newyear = True 
        self.timer_enabled = True  # 定时器控制标志位

        self.current_time = None
        self.last_change_time = None        # 记录步态切换的上次执行时间
        self.change_interval_time = 5.0       # 步态切换的执行间隔，比如5秒
        

        self.dt = 0.05                                                                 # 设置控制时间步长
        self.t = -1                                                                     # 初始化运行时间计数
        # 创建一个定时器
        self.timer = self.create_timer(self.dt, self.timer_callback)                    

        self.callback_lock = threading.Lock()  # 添加锁


    def timer_callback(self):
        self.current_time = time.time()
        # 如果锁已被占用，说明上一次回调还在执行，直接返回
        if not self.callback_lock.acquire(blocking=False):
            self.get_logger().warning('之前的回调还在进行中，跳过此次回调')
            return

        try:
            if not self.timer_enabled:
                return
            
            self.t += self.dt                                                               # 等待1秒后启动
            self.get_logger().info(f"当前时刻为：{self.t}")

            if self.t >= 0 and self.t <3:                                                                 # 检查运行时间计数是否为非负
                if self.start_enabled:    
                    self.Start()
                self.VelocityMove()
                

            elif self.t >= 3 and self.t < 15:
                if self.relax_flag :
                    self.Relax()

                elif self.sit_enabled :
                    self.go2Sit()

                elif self.resit_enabled :
                    self.go2RiseSit()

                elif self.change_enabled :
                    self.ChangeGait(3)
                    

                else :
                    self.VelocityMove()

            elif self.t >=15 and self.t <20:
                if self.current_time - self.last_change_time >= self.change_interval_time :
                    self.change_enabled = True
                
                if self.change_enabled:
                    self.ChangeGait(0) 

                self.VelocityMove()

            elif self.t >=20:
                self.Stop()
                if self.newyear :
                    self.NewYear()
                else :
                    self.Stop()

        finally:
            self.callback_lock.release()  # 释放锁

    # 控制定时器起停
    def start_timer(self):
        self.timer_enabled = True
        self.timer.reset()  # 重置定时器
        self.get_logger().info('定时器重启')
    
    def stop_timer(self):
        self.timer_enabled = False
        self.get_logger().info('定时器暂停')

    def VelocityMove(self):
  
        self.vx = 0.0                                                                    # 初始化初始x位置
        self.vy = 0.0                                                                    # 初始化初始y位置
        self.vyaw = 0.0                                                                  # 初始化初始偏航角
        self.sport_req.Move(self.req, self.vx, self.vy, self.vyaw)                  # 获取与高级运动命令对应的请求消息
        self.req_puber.publish(self.req)                                            # 发布请求消息

        # self.get_logger().info(f'当前x方向速度{self.vx}， y方向速度{self.vy}， 偏航角速度{self.vyaw}')

    # 切换步态 (只要一次，后面还要切回正常步态)
    def ChangeGait(self,num: int):
        # 0  为  idle， 1  为  trot，2  为  trot running，3  正向爬楼模式，4：逆向爬楼模式。
        self.get_logger().info(f"切换步态为{num}")
        self.sport_req.SwitchGait(self.req,num)
        self.req_puber.publish(self.req) 

        self.last_change_time = self.current_time
        self.change_enabled = False

    def Relax(self):
        self.get_logger().info("休息中....")

        # 伸懒腰Stretch()
        self.sport_req.Stretch(self.req)
        self.req_puber.publish(self.req) 

        self.relax_flag = False



        self.get_logger().info(f"当前休息标志为{self.relax_flag}")

    # 拜年(只能一次性)
    def NewYear(self):
        self.get_logger().info("给大家拜年....")
        self.sport_req.Scrape(self.req)
        self.req_puber.publish(self.req) 

        self.newyear = False

    # 坐下
    def go2Sit(self):
        self.get_logger().info("坐下....")
        self.sport_req.Sit(self.req)
        self.req_puber.publish(self.req) 

        time.sleep(5)

        self.sit_enabled = False

    # 站起（相对于坐下）
    def go2RiseSit(self): 
        self.get_logger().info("起立....")
        self.sport_req.RiseSit(self.req)
        self.req_puber.publish(self.req) 

        time.sleep(2)
        self.resit_enabled = False

    def Start(self):

        self.get_logger().info("初始化中....")

        # 打招呼
        self.sport_req.Hello(self.req)
        self.req_puber.publish(self.req)  

        self.start_enabled = False

    def Stop(self):
        self.sport_req.StopMove(self.req)                                           # 获取与高级运动命令对应的请求消息
        self.req_puber.publish(self.req)                                            # 发布请求消息





def main(args=None):
    rclpy.init(args=args)                                                                   # 初始化rclpy
    sport_demo_node = SportDemo()                                                           # 创建SportRequest节点的实例
    rclpy.spin(sport_demo_node)                                                             # 保持节点运行

    rclpy.shutdown()                                                                        # 关闭rclpy

if __name__ == '__main__':
    main()      