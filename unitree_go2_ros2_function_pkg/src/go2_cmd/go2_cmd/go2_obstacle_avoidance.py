# slam_node.py
"""
雷达话题发布频率太低，导致运动控制的callback频率不能过高

"""

import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import (LaserScan,PointCloud2)               # 自带雷达点云消息类型
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped                          # 自带雷达障碍距离消息类型
from unitree_go.msg import SportModeState                           # 导入SportModeState消息类型，订阅状态信息
from unitree_api.msg import Request                                 # 导入Request消息类型，发送运动指令
from go2_sport.ros2_sport_client import (SportClient,PathPoint)     # 高层运动控制接口API调用 

from go2_test.logdata_clean import PathData                         # 引入路径log文件参数调试类

import time
import math
import numpy as np

import threading
import queue

# 自定义服务类型（用于响应主控程序的slam请求指令）
from custom_interface.srv import UnitreeSlam

# 定义话题名称
UTLIDARRANGETOPIC = "utlidar/range_info"                             # 添加障碍物雷达测距值话题
SPORTTOPIC = "api/sport/request"                                     # 添加运动控制话题

# 定义服务名称
SLAMCOMMANDSERVICE = "unitree_slam_command"                          # unitree的slam步骤指令服务

# 定义调试开关
TESTSWITCH = True                       # slam节点开关
MOVETYPE = 1                            # 移动路径开关  1：沿右侧逆时针绕行 2：普通直行 

# 定义安全距离 
SAFE_DISTANCE_HEAD = 0.7                    # 前侧安全距离，单位：米
SAFE_DISTANCE_FLANK = 0.35                   # 两侧安全距离，单位：米  
GO_DISTANCE = 2                             # 可前进距离 
CONFIRM_TIME = 2                            # 90度转弯传感器确认次数
BREAK_CONFIRM_TIME = 2                      # 转弯直行打断步数

# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色



# 定义通过日志数据调试所需变量
log_file_path = '/home/flyivan/dog_robot/ros2_demo/analy/python3_7690_1730273139382.log'                   # 日志文件路径
keyword = '前方障碍距离' 
LOGTEST = 0                                                                                    # 日志关键词，=0取传感器真实数据，=1取日志模拟数据

class SlamNode(Node):
    def __init__(self,name):
        super().__init__(name)
        
        # 创建机器人速度的发布者 
        self.cmd_vel_pub = self.create_publisher(Request, SPORTTOPIC, 10)               # 创建运动控制的发布者       
        self.req = Request()                                                            # 初始化Request消息
        self.sport_req = SportClient()                                                  # 实例化一个SportClient对象(这里是json封装的client)
        self.range_msg = PointStamped()                                                 # 实例花一个雷达消息类型对象


        # 初始化变量
        self.current_scan = None
        self.current_odom = None
        self.action = 'Move forward'
        self.start_time = -2                                                           # 初始化运行时间计数
        self.vx0 = 0                                                                    # 初始化初始x位置
        self.vy0 = 0                                                                    # 初始化初始y位置
        self.vyaw0 = 0                                                                  # 初始化初始偏航角

        # 初始化方向切换确认变量
        self.front_confirm_count = 0
        self.left_confirm_count = 0
        self.right_confirm_count = 0

        self.break_confirm_count = 0


        # 初始化变向定时器传感参数
        self.front = 100.0  # 初始化前方距离
        self.left = 100.0   # 初始化左侧距离
        self.right = 100.0  # 初始化右侧距离
      

        # 创建自带激光雷达数据的订阅者 
        self.scan_sub = self.create_subscription(PointStamped, UTLIDARRANGETOPIC, self.scan_callback, 10)
        

        # 创建里程计数据的订阅者
        self.odom_sub = self.create_subscription(Odometry, '/lio_sam_ros2/mapping/re_location_odometry', self.odom_callback, 10)
        

        # 创建slam服务请求客户端
        if TESTSWITCH:
            self.slam_client = self.create_client(UnitreeSlam, SLAMCOMMANDSERVICE)          # 创建服务客户端对象（服务接口类型，服务名）
            while not self.slam_client.wait_for_service(timeout_sec=1.0):                   # 循环等待服务器端成功启动  
                self.get_logger().info('unitree的slam服务尚未启动, 请耐心等待...') 
            self.request = UnitreeSlam.Request()                                            # 创建服务请求的数据对象


        # 创建定时器，定期发送速度命令   
        self.dt = 0.5                                                                   # 设置控制时间步长   
        self.detect_dt = 0.5                                                            # 不能小于控制步长
        self.timer = self.create_timer(self.dt, self.timer_callback)                    # 创建一个定时器，每0.5秒（create_time单位秒）调用一次timer_callback函数
        self.turn_timer = self.create_timer(self.detect_dt, self.turn_timer_callback)   # 转弯期间的定时器
        self.condition_trigger = False
        self.execute_move = False  # 标志变量 


        # 创建建图等待时长
        self.map_dt = 240
    
        # 初始化测试常用变量
        self.test_scan = PathData()
        self.log_count = 0

    # 激光雷达数据的回调函数
    def scan_callback(self, msg):
        self.current_scan = msg


    # 里程计数据的回调函数
    def odom_callback(self, msg):
        self.current_odom = msg


    # 主定时器回调函数
    def timer_callback(self):
        # self.get_logger().info(f'{GREEN}主回调启动{RESET}')                               # 添加日志确认回调被调用
        self.start_time += self.dt                                                      # 等待2秒后启动
        if 0 <= self.start_time < self.map_dt:                                          # 检查运行时间计数是否为非负，是否到达建图时间上限
                
                self.execute_move = True
                self.log_count += 1
                self.get_logger().info(f'当前运行时长:{self.start_time}')


        # 待调整
        elif self.start_time >= self.map_dt:
            self.execute_move = False
            self.get_logger().info(f'已运行{self.map_dt}秒，建图结束..')
            self.unitree_slam('e')
            self.timer.cancel()  # 取消主定时器
            self.turn_timer.cancel() # 取消转向定时器
            return
        

    # 转向定时器回调函数
    def turn_timer_callback(self):
        # self.get_logger().info(f'{GREEN}转向定时器启动{RESET}')  # 添加日志确认回调被调用

        if self.current_scan is not None:
            self.front = self.current_scan.point.x                  # 订阅当前障碍距离
            self.left = self.current_scan.point.y
            self.right = self.current_scan.point.z

            self.get_logger().info(f'{BLUE}最新！！前距障碍物：{self.front},左距障碍物：{self.left},右距障碍物：{self.right}{RESET}') 
            if ((self.front <= SAFE_DISTANCE_HEAD and not math.isinf(self.front)) or 
                (self.left <= SAFE_DISTANCE_FLANK and not math.isinf(self.left)) or 
                (self.right <= SAFE_DISTANCE_FLANK) and not math.isinf(self.right)) :

                self.break_confirm_count += 1
                if self.break_confirm_count >= BREAK_CONFIRM_TIME :
                    self.condition_trigger = True
                    self.break_confirm_count = 0
                    self.get_logger().info(f'{YELLOW}触发停止转向条件，标志位置为:{self.condition_trigger}{RESET}')

            else :
                self.break_confirm_count = 0

        else:
            # self.front = 100
            # self.left = 100
            # self.right = 100
            self.get_logger().warning('当前扫描数据为空，无法更新距离信息')


    # 启动运动控制模块函数
    def trigger_motion_control(self):
        self.get_logger().info('启动运动控制模块') 

        # 导入日志模拟数据（如需）
        self.test_scan.parse_log_file(log_file_path,keyword)

        self.autonomous_motion()

    # 运动方向策略执行测试模块
    def action_test(self):

        # 单独测试各类行进模式
    
        vx = 0.05
        vy = 0.0
        vyaw = -np.pi/8                 # 转向速度要足够，不然来不及转      
        for i in range(11):
            self.vel_contrl(vx,vy,vyaw)
            self.get_logger().info(f'{RED}第{i+1}次右转{RESET}')
            time.sleep(self.dt)        

        for i in range(35):
            # 在每次迭代中调用 spin_once
            rclpy.spin_once(self)  # 处理事件循环
            time.sleep(self.detect_dt)

            self.get_logger().info(f'{YELLOW}触发停止转向条件，标志位:{self.condition_trigger}{RESET}')
            if self.condition_trigger :
                self.get_logger().info(f'{RED}周围有障碍, 停止行进，重新规划{RESET}')
                self.condition_trigger = False
                break

            self.vel_contrl(0.1,vy,0)
            self.get_logger().info(f'{YELLOW}第{i+1}次直行稳定{RESET}')

    def action_stand(self):
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        self.vel_contrl(vx,vy,vyaw)

    # 运动方向策略执行模块
    def action_formal(self):
        #调用避障方法
        action = self.avoid_obstacle()


        """
        经测试，按pi/8的角速度，每转90度，所需步数和步长大致对应关系如下
        8次/1秒
        10次/0.5秒
        48次/0.1秒
        """
        """
        经测试，按0.1的线速度，所行走步数和对应步长如下
        10步/0.5米——1个瓷砖格
        30步/1.5米
        """


        if action == 'Turn left':

            vx = 0.05
            vy = 0.0
            vyaw = np.pi/8                                         # 转向速度要足够，不然来不及转      
            for i in range(11):
                self.vel_contrl(vx,vy,vyaw)
                self.get_logger().info(f'{RED}第{i+1}次左转{RESET}')
                time.sleep(self.dt)        

            for i in range(35):
                # 在每次迭代中调用 spin_once
                rclpy.spin_once(self)                               # 处理事件循环
                time.sleep(self.detect_dt)

                self.get_logger().info(f'{BLUE}当前转向标志位:{self.condition_trigger}{RESET}')
                if self.condition_trigger :
                    self.get_logger().info(f'{RED}周围有障碍, 停止行进，重新规划{RESET}')
                    self.condition_trigger = False
                    break

                self.vel_contrl(0.1,vy,0)
                self.get_logger().info(f'{YELLOW}第{i+1}次直行稳定{RESET}')
                

                
        elif action == 'Turn right':

            vx = 0.05
            vy = 0.0
            vyaw = -np.pi/8                                         # 转向速度要足够，不然来不及转      
            for i in range(10):
                self.vel_contrl(vx,vy,vyaw)
                self.get_logger().info(f'{RED}第{i+1}次右转{RESET}')
                time.sleep(self.dt)        

            for i in range(35):
                # 在每次迭代中调用 spin_once
                rclpy.spin_once(self)                               # 处理事件循环
                time.sleep(self.detect_dt)

                # self.get_logger().info(f'{BLUE}当前转向标志位:{self.condition_trigger}{RESET}')
                if self.condition_trigger :
                    self.get_logger().info(f'{RED}周围有障碍, 停止行进，重新规划{RESET}')
                    self.condition_trigger = False
                    break

                self.vel_contrl(0.1,vy,0)
                self.get_logger().info(f'{YELLOW}第{i+1}次直行稳定{RESET}')
                
            
        elif action == 'little Rotate left':
            vx = 0.05
            vy = 0
            vyaw = 0.15
            self.vel_contrl(vx,vy,vyaw)

        elif action == 'little Rotate right':
            vx = 0.05
            vy = 0
            vyaw = -0.15 
            self.vel_contrl(vx,vy,vyaw)  

        elif action == 'left move':
            vx = 0
            vy = 0.05
            vyaw = 0
            self.vel_contrl(vx,vy,vyaw) 

        elif action == 'right move' :
            vx = 0
            vy = -0.05
            vyaw = 0
            self.vel_contrl(vx,vy,vyaw) 

        elif action == 'place left':
            vx = 0
            vy = 0
            vyaw = 0.15 
            for i in range(2):
                self.vel_contrl(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次原地左转{RESET}')
                time.sleep(self.dt)   

        elif action == 'place right':
            vx = 0
            vy = 0
            vyaw = -0.15 
            for i in range(2):
                self.vel_contrl(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次原地右转{RESET}')
                time.sleep(self.dt) 

        elif action == 'Move forward':
            vx = 0.2
            vy = 0.0
            vyaw = 0.0
            self.vel_contrl(vx,vy,vyaw)
        
        elif action == 'Back':
            vx = -0.1
            vy = 0.0
            vyaw = 0.0
            for i in range(5):
                self.vel_contrl(vx,vy,vyaw)  
                self.get_logger().info(f'{YELLOW}第{i+1}次后退{RESET}')
                time.sleep(self.dt)     
        
        elif action == 'Stand':                       # 原地不动
            vx = 0.0
            vy = 0.0
            vyaw = 0.0
            self.vel_contrl(vx,vy,vyaw)

    # 自主运动逻辑
    def autonomous_motion(self):

        # self.get_logger().info(f'{RED}重置转向条件标志位为:{self.condition_trigger}{RESET}')
        self.condition_trigger = False                                                          # 每次运动控制前，需重置转向停止标志位
        self.action_formal()                                                                    # 运动控制正式方法


        # self.action_test()                                                                    # 运动控制测试方法
        # self.action_stand()
        
        # 调用TrajectoryFollow回到起始点

    # 向狗子高层控制接口发送速度指令
    def vel_contrl(self,vx :float,vy : float,vyaw : float):

        self.sport_req.Move(self.req,vx, vy, vyaw)                              # 获取与高级运动命令对应的请求消息
        self.cmd_vel_pub.publish(self.req)                                      # 发布速度命令



    # 简单的避障逻辑
    def avoid_obstacle(self):             
        self.get_logger().info(f'启动避障逻辑')

        # 获取前侧、左侧、右侧的距离
        if LOGTEST == 0:
            front_distance = self.current_scan.point.x
            left_distance = self.current_scan.point.y
            right_distance = self.current_scan.point.z

            self.get_logger().info(f'{YELLOW}前方障碍距离：{self.current_scan.point.x:.3f},{RESET}'
                                f'{RED}左侧障碍距离：{self.current_scan.point.y:.3f},{RESET}'
                                f'{BLUE}右侧障碍距离：{self.current_scan.point.z:.3f}{RESET}')                          # 打印最新障碍位置


        # 模拟数据获取前侧、左侧、右侧的距离
        elif LOGTEST == 1:
            all_distances = self.test_scan.get_all_distances()

            if 0 <= self.log_count < len(all_distances) :
                front_distance, left_distance, right_distance = all_distances[self.log_count]
                self.get_logger().info(f"第 {self.log_count+1} 组距离值：前方距离：{front_distance}, 左侧距离：{left_distance}, 右侧距离：{right_distance}")
            else:
                self.get_logger().info(f"索引 {self.log_count} 超出范围，无法访问该组距离值。")

     


        if MOVETYPE == 1:
            # 右转遍历路径

            # 前方有障碍物，左右两侧有出口
            if front_distance < SAFE_DISTANCE_HEAD:            # 如果前方距离小于安全距离  
                # 左侧距离不足
                if left_distance < SAFE_DISTANCE_FLANK :
                    # 死胡同
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 后退
                        action = "Back" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0
                    # 门逢右侧宽 & 过道右侧宽 
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 原地右小转
                        action = "place right" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0
                    # 左小右大斜撞墙
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 原地右小转
                        action = "place right" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand'

                # 左侧距离适中
                elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                    # 门逢左侧宽 & 过道左侧宽 
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 原地左小转
                        action = "place left" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0      

                    # 左右距离均适中，到底哪个方向由nav2决定
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 直行
                        if left_distance > right_distance + SAFE_DISTANCE_FLANK :
                            # 原地左小转
                            action = "place left" 
                            self.left_confirm_count = 0
                            self.right_confirm_count = 0

                        else :
                            # 原地右小转
                            action = "place right" 
                            self.left_confirm_count = 0
                            self.right_confirm_count = 0  

                    # 左小右大斜撞墙
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 右小转
                        action = "little Rotate right" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand'


                # 左侧距离大
                elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :
                    # 右大左小斜撞墙
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 原地左小转
                        action = "place left" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0     

                    # 右大左小斜撞墙
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 左小转
                        action = "little Rotate left" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0     

                    # 垂直迎向障碍物
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 左转
                        self.left_confirm_count += 1
                        if self.left_confirm_count >= CONFIRM_TIME :
                            action = "Turn left"    
                            self.left_confirm_count = 0
                        else:
                            action = "Move forward" 
            
                else :
                    action = 'Stand'


            # 前方大于出口距离
            elif (front_distance >= GO_DISTANCE or math.isinf(front_distance)):
                # 左侧距离不足
                if left_distance < SAFE_DISTANCE_FLANK :

                    # 狭窄胡同 & 狭窄门缝
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 后退
                        action = "Back" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 左侧有平行障碍物
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 右平移
                        action = "right move" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 前 右空间大
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 右转
                        self.right_confirm_count += 1
                        if self.right_confirm_count >= CONFIRM_TIME :
                            action = "Turn right"
                            self.right_confirm_count = 0
                        else:
                            action = "Move forward" 

                    else :
                        action = 'Stand'


                # 左侧距离适中
                elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                    # 右侧有平行障碍物
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 左平移
                        action = "left move" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0     

                    # 可通行场景
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 前 右空间大
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 右转
                        self.right_confirm_count += 1
                        if self.right_confirm_count >= CONFIRM_TIME :
                            action = "Turn right"
                            self.right_confirm_count = 0
                        else:
                            action = "Move forward" 

                    else :
                        action = 'Stand'


                # 左侧距离大
                elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :
                    # 前 左空间大
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 左转
                        self.left_confirm_count += 1
                        if self.left_confirm_count >= CONFIRM_TIME :
                            action = "Turn left"    
                            self.left_confirm_count = 0
                        else:
                            action = "Move forward" 
                    # 前 左空间大
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 左转
                        self.left_confirm_count += 1
                        if self.left_confirm_count >= CONFIRM_TIME :
                            action = "Turn left"    
                            self.left_confirm_count = 0
                        else:
                            action = "Move forward" 
                    # 可通行场景
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand' 
                
                else :
                    action = 'Stand' 

            # 前方在出口距离与安全距离之间
            else :   
                # 左侧距离不足
                if left_distance < SAFE_DISTANCE_FLANK :

                    # 潜在死胡同
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 后退
                        action = "Back" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 门逢右侧宽 & 过道右侧宽 
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 原地右小转
                        action = "place right" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 左小右大斜撞墙
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 原地右小转
                        action = "place right" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand'


                # 左侧距离适中
                elif left_distance >= SAFE_DISTANCE_FLANK and left_distance < GO_DISTANCE :
                    # 门逢左侧宽 & 过道左侧宽 
                    if right_distance < SAFE_DISTANCE_FLANK :
                        # 原地左小转
                        action = "place left" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0     

                    # 可通行场景
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 可通行场景
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand'

                # 左侧距离大
                elif (left_distance >= SAFE_DISTANCE_FLANK or math.isinf(left_distance)) :

                    # 右小左大斜撞墙
                    if right_distance < SAFE_DISTANCE_FLANK : 
                        # 原地左小转
                        action = "place left" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0  

                    # 可通行场景
                    elif right_distance >= SAFE_DISTANCE_FLANK and right_distance < GO_DISTANCE :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    # 可通行场景
                    elif (right_distance >= SAFE_DISTANCE_FLANK or math.isinf(right_distance)) :
                        # 直行
                        action = "Move forward" 
                        self.left_confirm_count = 0
                        self.right_confirm_count = 0

                    else :
                        action = 'Stand'  
                
                else :
                    action = 'Stand'                     
          



        elif MOVETYPE == 2:
            # 普通行走
            pass
              
        self.get_logger().info(f'当前运动方向{action}')                   # 打印该步运动方向

        return action  


    # 记录起始点及回到起始点
    def start_end_record(self):
        pass


    # 实现 SLAM 算法
    # 算法1：宇数自带slam算法节点
    def unitree_slam(self,command_key):                                                 # 创建一个发送服务请求的函数
        self.request.command = command_key
        self.future = self.slam_client.call_async(self.request)                         # 异步方式发送服务请求

        return self.future
        # # 添加日志，记录开始等待服务响应
        # self.get_logger().info('等待服务器的响应...')

        # # 等待服务器响应结果
        # rclpy.spin_until_future_complete(self, self.future)


        # # 添加日志，记录服务响应已接收
        # self.get_logger().info('收到服务器响应！')
        
        # try:
        #     response = self.future.result()
        #     self.get_logger().info(f'服务器的响应结果为: {response.result}')
        #     return response.result
        # except Exception as e:                                                          # 把异常类捕获的具体异常实例付给e
        #     self.get_logger().error(f'服务器响应失败，失败原因: {e}')
        #     return None


def wait_for_future(node, future, result):
    # 添加日志，记录开始等待服务响应
    node.get_logger().info('等待服务器的响应...')

    rclpy.spin_until_future_complete(node, future)

    # 添加日志，记录服务响应已接收
    node.get_logger().info('收到服务器响应！')

    # 处理完成后的逻辑
    try:
        response = future.result()
        node.get_logger().info(f'服务器的响应结果为: {response.result}')
        result.put(response.result)
        # return response.result
    except Exception as e:                                                          # 把异常类捕获的具体异常实例付给e
        node.get_logger().error(f'服务器响应失败，失败原因: {e}')
        result.put(None)
        # return None


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode('go2_obstacle_avoidance')


    """
    调试控制节点专用开关
    """
    if TESTSWITCH == True :
        # result = node.unitree_slam('w')

        # 使用类方法进行异步服务调用
        future = node.unitree_slam('w')

        # 创建一个队列用于获取线程返回值
        result = queue.Queue()

        # 在单独的线程中等待 future 完成
        threading.Thread(target=wait_for_future, args=(node, future, result)).start()

        if result:
        # 倒计时30秒，等建图服务开启
            for remaining in range(30, 0, -1):
                    node.get_logger().info(f"倒计时: {remaining}秒")
                    # print(f"倒计时: {remaining}秒", end='\r',flush=True)
                    time.sleep(1)
            node.get_logger().info("倒计时结束！")      

            while rclpy.ok():
                rclpy.spin_once(node)  # 处理事件循环

                # 检查主定时器标志变量并执行运动控制的逻辑
                if node.execute_move:
                    # node.get_logger().info('unitree的SLAM服务已成功启动, 准备行进中建图...')
                    node.trigger_motion_control()  # 调用运动控制方法
                    node.execute_move = False  # 重置标志变量
                
        else:
            node.get_logger().error('Failed to receive a successful response from SLAM service.')

    elif TESTSWITCH == False :  

        for remaining in range(2, 0, -1):
            node.get_logger().info(f"倒计时: {remaining}秒")
            time.sleep(1)
        node.get_logger().info("倒计时结束！")  
        
        while rclpy.ok():
            rclpy.spin_once(node)  # 处理事件循环

            # 检查主定时器标志变量并执行运动控制的逻辑
            if node.execute_move:
                node.trigger_motion_control()  # 调用运动控制方法
                node.execute_move = False  # 重置标志变量




        # elif not node.execute_move :
        #     node.get_logger().info('SLAM服务开始关闭')
        #     node.unitree_slam('e')
 
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()


