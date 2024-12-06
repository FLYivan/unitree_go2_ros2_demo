import rclpy
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped

from unitree_go.msg import SportModeState                           # 导入SportModeState消息类型，订阅状态信息
from unitree_api.msg import Request                                 # 导入Request消息类型，发送运动指令
from go2_sport.ros2_sport_client import (SportClient,PathPoint)     # 高层运动控制接口API调用 

# 定义话题名称
UTLIDARRANGETOPIC = "utlidar/range_info"                             # 添加障碍物雷达测距值话题
SPORTTOPIC = "api/sport/request"                                     # 添加运动控制话题

# 定义避障安全距离 
SAFE_DISTANCE_HEAD = 0.7                    # 前侧安全距离，单位：米
SAFE_DISTANCE_FLANK = 0.35                  # 两侧安全距离，单位：米  
GO_DISTANCE = 2                             # 可前进距离 
CONFIRM_TIME = 2                            # 90度转弯传感器确认次数
BREAK_CONFIRM_TIME = 2                      # 转弯直行打断步数


# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('go2_move')

        # 初始化变量
        self.current_scan = None

        # 初始化nav2获取的速度变量
        self.nav2_vx = 0.0
        self.nav2_vy = 0.0
        self.nav2_vyaw = 0.0

        # 创建nav2控制指令订阅者
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 订阅 cmd_vel 话题
            self.cmd_vel_callback,
            10
        )

        # 创建自带激光雷达避障数据订阅者 
        self.scan_sub = self.create_subscription(PointStamped, UTLIDARRANGETOPIC, self.scan_callback, 10)


        # 创建go2控制指令发布者
        self.cmd_vel_pub = self.create_publisher(Request, SPORTTOPIC, 10)       # 创建运动控制的发布者 
        self.req = Request()                                                    # 初始化Request消息
        self.sport_req = SportClient()                                          # 实例化一个SportClient对象(这里是json封装的client)

        # 创建定时器
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

    
    # 激光雷达数据的回调函数
    def scan_callback(self, msg):
        self.current_scan = msg

    # nav2路径规划回调函数
    def cmd_vel_callback(self, msg):
        # 获取线速度和角速度
        self.nav2_vx = msg.linear.x
        self.nav2_vy = msg.linear.y
        self.nav2_vyaw = msg.angular.z


    # go2控制指令发布函数
    def send_to_go2(self,vx :float,vy : float,vyaw : float):
        # 在这里调用go2机器人的 API，将速度发送给go2
 

        self.sport_req.Move(self.req,vx, vy, vyaw)                              # 获取与高级运动命令对应的请求消息
        self.cmd_vel_pub.publish(self.req)                                      # 发布速度命令

        # self.get_logger().info(f'{YELLOW}Sending to go2: vx ={vx}, vy={vy}, vyaw={vyaw}{RESET}')

    # 定时器回调，启动主控
    def timer_callback(self):
        self.active_control()             # 避障行进
        # self.only_nav2()

    # 只用nav2的结果来控制的方法
    def only_nav2(self):
        self.send_to_go2(self.nav2_vx, self.nav2_vy, self.nav2_vyaw)

    # go2行进策略执行函数
    def active_control(self):

        # 调用避障函数，判断如何行进
        # action = self.avoid_obstacle()

        # 调用简化避障函数，判断如何行进
        action = self.avoid_obstacle_simply()


        # 左小转     
        if action == 'little Rotate left':
            vx = 0.05
            vy = 0
            vyaw = 0.15
            self.send_to_go2(vx,vy,vyaw)

        # 右小转
        elif action == 'little Rotate right':
            vx = 0.05
            vy = 0
            vyaw = -0.15 
            self.send_to_go2(vx,vy,vyaw)  

        # 左平移
        elif action == 'left move':
            vx = 0
            vy = 0.05
            vyaw = 0
            self.send_to_go2(vx,vy,vyaw) 

        # 右平移
        elif action == 'right move' :
            vx = 0
            vy = -0.05
            vyaw = 0
            self.send_to_go2(vx,vy,vyaw) 

        # 原地左小转
        elif action == 'place left':
            vx = 0
            vy = 0
            vyaw = 0.15 
            for i in range(2):
                self.send_to_go2(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次原地左转{RESET}')
                time.sleep(self.dt)   

        # 原地右小转
        elif action == 'place right':
            vx = 0
            vy = 0
            vyaw = -0.15 
            for i in range(2):
                self.send_to_go2(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次原地右转{RESET}')
                time.sleep(self.dt) 

        # 直行
        elif action == 'Move forward':
            vx = 0.2
            vy = 0.0
            vyaw = 0.0
            self.send_to_go2(vx,vy,vyaw)

        # 后退    
        elif action == 'Back':
            vx = -0.1
            vy = 0.0
            vyaw = 0.0
            for i in range(2):
                self.send_to_go2(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次后退{RESET}')
                time.sleep(self.dt)     

        # 使用nav2的规划命令
        elif action == 'Nav2':
            self.send_to_go2(self.nav2_vx, self.nav2_vy, self.nav2_vyaw)

    # 上下楼梯函数
    def climb_stairs(self):
        pass

    # 简化的避障函数
    def avoid_obstacle_simply(self):
        front_distance = self.current_scan.point.x
        left_distance = self.current_scan.point.y
        right_distance = self.current_scan.point.z

        self.get_logger().info(f'{YELLOW}前方障碍距离：{self.current_scan.point.x:.3f},{RESET}'
                        f'{RED}左侧障碍距离：{self.current_scan.point.y:.3f},{RESET}'
                        f'{BLUE}右侧障碍距离：{self.current_scan.point.z:.3f}{RESET}')                          # 打印最新障碍位置

        if front_distance <= SAFE_DISTANCE_HEAD:
            # 后退
            action = "Back"

        elif left_distance <= SAFE_DISTANCE_FLANK:
            # 右平移
            action = "right move" 

        elif right_distance <= SAFE_DISTANCE_FLANK:
            # 左平移
            action = "left move"
        else:
            action = "Nav2"

        self.get_logger().info(f'{RED}当前运动方向{action}{RESET}')                   # 打印该步运动方向
        return action  
        
        
        
        



    # 避障函数
    def avoid_obstacle(self):
        front_distance = self.current_scan.point.x
        left_distance = self.current_scan.point.y
        right_distance = self.current_scan.point.z

        self.get_logger().info(f'{YELLOW}前方障碍距离：{self.current_scan.point.x:.3f},{RESET}'
                            f'{RED}左侧障碍距离：{self.current_scan.point.y:.3f},{RESET}'
                            f'{BLUE}右侧障碍距离：{self.current_scan.point.z:.3f}{RESET}')                          # 打印最新障碍位置

        # 前方有障碍物，左右两侧有出口
        if front_distance < SAFE_DISTANCE_HEAD:            # 如果前方距离小于安全距离  
            # 左侧距离不足
            if left_distance < SAFE_DISTANCE_FLANK :
                # 死胡同
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 后退
                    action = "Back" 

                # 门逢右侧宽 & 过道右侧宽 
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 原地右小转
                    action = "Nav2" 

                # 左小右大斜撞墙
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 原地右小转
                    action = "Nav2" 

                else :
                    action = 'Nav2'

            # 左侧距离适中
            elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                # 门逢左侧宽 & 过道左侧宽 
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 原地左小转
                    action = "Nav2"    

                # 左右距离均适中，到底哪个方向由nav2决定
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # nav2规划
                    action = "Nav2" 


                # 左小右大斜撞墙
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 右小转
                    action = "little Rotate right" 


                else :
                    action = 'Nav2'


            # 左侧距离大
            elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :
                # 右大左小斜撞墙
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 原地左小转
                    action = "Nav2" 
   

                # 右大左小斜撞墙
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 左小转
                    action = "little Rotate left" 
 

                # 垂直迎向障碍物
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 测试给nav2
                    action = "Nav2"  

                    # # 左转
                    # self.left_confirm_count += 1
                    # if self.left_confirm_count >= CONFIRM_TIME :
                    #     action = "Turn left"    
                    #     self.left_confirm_count = 0
                    # else:
                    #     action = "Move forward" 
        
            else :
                action = 'Nav2'


        # 前方大于出口距离
        elif (front_distance >= GO_DISTANCE or math.isinf(front_distance)):
            # 左侧距离不足
            if left_distance < SAFE_DISTANCE_FLANK :

                # 狭窄胡同 & 狭窄门缝
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 后退
                    action = "Nav2" 

                # 左侧有平行障碍物
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 右平移
                    action = "right move" 

                # 前 右空间大
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 测试给nav2
                    action = "Nav2"  

                    # # 右转
                    # self.right_confirm_count += 1
                    # if self.right_confirm_count >= CONFIRM_TIME :
                    #     action = "Turn right"
                    #     self.right_confirm_count = 0
                    # else:
                    #     action = "Move forward" 

                else :
                    action = 'Nav2'


            # 左侧距离适中
            elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                # 右侧有平行障碍物
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 左平移
                    action = "left move"   

                # 可通行场景
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 直行
                    action = "Move forward" 


                # 前 右空间大
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 测试给nav2
                    action = "Nav2"  

                    # # 右转
                    # self.right_confirm_count += 1
                    # if self.right_confirm_count >= CONFIRM_TIME :
                    #     action = "Turn right"
                    #     self.right_confirm_count = 0
                    # else:
                    #     action = "Move forward" 

                else :
                    action = 'Nav2'


            # 左侧距离大
            elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :
                # 前 左空间大
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 测试给nav2
                    action = "Nav2"  

                    # # 左转
                    # self.left_confirm_count += 1
                    # if self.left_confirm_count >= CONFIRM_TIME :
                    #     action = "Turn left"    
                    #     self.left_confirm_count = 0
                    # else:
                    #     action = "Move forward" 

                # 前 左空间大
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 测试给nav2
                    action = "Nav2"  

                    # # 左转
                    # self.left_confirm_count += 1
                    # if self.left_confirm_count >= CONFIRM_TIME :
                    #     action = "Turn left"    
                    #     self.left_confirm_count = 0
                    # else:
                    #     action = "Move forward" 

                # 可通行场景
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 直行
                    action = "Move forward" 


                else :
                    action = 'Nav2' 
            
            else :
                action = 'Nav2' 

        # 前方在出口距离与安全距离之间
        else :   
            # 左侧距离不足
            if left_distance < SAFE_DISTANCE_FLANK :

                # 潜在死胡同
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 后退
                    action = "Back" 

                # 门逢右侧宽 & 过道右侧宽 
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 原地右小转
                    action = "Nav2" 


                # 左小右大斜撞墙
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 原地右小转
                    action = "Nav2" 


                else :
                    action = 'Nav2'


            # 左侧距离适中
            elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                # 门逢左侧宽 & 过道左侧宽 
                if right_distance < SAFE_DISTANCE_FLANK :
                    # 原地左小转
                    action = "Nav2" 
 

                # 可通行场景
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 直行
                    action = "Nav2" 


                # 可通行场景
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 直行
                    action = "Nav2" 


                else :
                    action = 'Nav2'

            # 左侧距离大
            elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :

                # 右小左大斜撞墙
                if right_distance < SAFE_DISTANCE_FLANK : 
                    # 原地左小转
                    action = "Nav2" 


                # 可通行场景
                elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                    # 直行
                    action = "Nav2" 

                # 可通行场景
                elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                    # 直行
                    action = "Nav2" 


                else :
                    action = 'Nav2'  
            
            else :
                action = 'Nav2'                     

        self.get_logger().info(f'{RED}当前运动方向{action}{RESET}')                   # 打印该步运动方向
        return action  





def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()