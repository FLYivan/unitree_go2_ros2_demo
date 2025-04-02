#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@说明: 通过go2提供的ROS2接口-向狗子发送高层运动指令，如等待1秒后，在X方向往复运动
"""

import rclpy  # 导入ROS2 Python客户端库
from rclpy.node import Node  # 从rclpy中导入Node类
from unitree_go.msg import SportModeState  # 导入SportModeState消息类型
from unitree_api.msg import Request  # 导入Request消息类型
from go2_sport.ros2_sport_client import (SportClient,PathPoint)


import time

import math

class SportRequest(Node):
    def __init__(self,name):
        super().__init__(name)                                                                                  # 使用名称"dog_control_pub"初始化节点
        self.state_suber = self.create_subscription(SportModeState, 'sportmodestate', self.state_callback, 10)  # 创建对'sportmodestate'的订阅

        self.req_puber = self.create_publisher(Request, '/api/sport/request', 10)                               # 创建'/api/sport/request'的发布者

        self.dt = 0.002                                                                 # 设置控制时间步长
        self.timer = self.create_timer(self.dt, self.timer_callback)                    # 创建一个定时器，每0.002秒（create_time单位秒）调用一次timer_callback函数
        self.req = Request()                                                            # 初始化Request消息
        self.sport_req = SportClient()                                                  # 实例化一个SportClient对象(这里是json封装的client)
        self.t = -1                                                                     # 初始化运行时间计数
     
        self.px0 = 0                                                                    # 初始化初始x位置
        self.py0 = 0                                                                    # 初始化初始y位置
        self.yaw0 = 0                                                                   # 初始化初始偏航角

    def timer_callback(self):
        self.t += self.dt                                                               # 等待1秒后启动
        if self.t >= 0:                                                                 # 检查运行时间计数是否为非负
            path = self.generate_path()                                                 # 调用生成路径的方法

            
            self.sport_req.TrajectoryFollow(self.req,path)                              # 获取与高级运动命令对应的请求消息
            self.req_puber.publish(self.req)                                            # 发布请求消息

    def state_callback(self, data):
        if self.t < 0:                                                                                  # 检查运行时间计数是否为负
            self.px0 = data.position[0]                                                                 # 获取初始x位置
            self.py0 = data.position[1]                                                                 # 获取初始y位置
            self.yaw0 = data.imu_state.rpy[2]                                                           # 获取初始偏航角
            self.get_logger().info(f'初始x方向坐标：{self.px0:.3f}, 初始y方向坐标：{self.py0}, 初始偏航角：{self.yaw0:.3f}')     # 记录初始位置和偏航角
        else :
            self.get_logger().info(f'当前x方向坐标：{self.px0:.3f}, 当前y方向坐标：{self.py0}, 当前偏航角：{self.yaw0:.3f}')     # 记录当前位置和偏航角

    def generate_path(self):                                                             # 在这里实现路径生成逻辑
        time_seg = 0.2                                                                   # 设置时间间隔      
        time_temp = self.t - time_seg                                                    # 初始化时间变量，这样，在第一次迭代中，time_temp 加上 time_seg 就会等于 self.t
                                                                                         # 确保路径点的时间从当前时间 self.t 开始。    
        path = []                                                                        # 初始化路径列表
        
        for i in range(30):                                                              # 遍历路径点            
            time_temp += time_seg                                                        # 更新时间            
            px_local = 0.5 * math.sin(0.5 * time_temp)                                   # 通过正弦波，实现x方向上在[-0.5,0.5]之间往复运动
            py_local = 0
            yaw_local = 0
            vx_local = 0.5 * math.cos(0.5 * time_temp)                                   # 速度是位置的导数，所以用余弦波表示
            vy_local = 0
            vyaw_local = 0

           
            path_point_tmp = PathPoint(                                # 创建路径点对象
                timeFromStart=0.0,
                x=0.0,
                y=0.0,
                yaw=0.0,
                vx=0.0,
                vy=0.0,
                vyaw=0.0
            )                             # 创建路径点对象

            
            path_point_tmp.timeFromStart = i * time_seg                                   # 设置路径点的时间戳
            
            path_point_tmp.x = (px_local * math.cos(self.yaw0)- py_local * math.sin(self.yaw0)+ self.px0)     # 转换局部坐标系下的路径点位置到全局坐标系               
            path_point_tmp.y = (px_local * math.sin(self.yaw0)+ py_local * math.cos(self.yaw0)+ self.py0)        
            path_point_tmp.yaw = yaw_local + self.yaw0                                     # 设置路径点的航向角        
            path_point_tmp.vx = vx_local * math.cos(self.yaw0) - vy_local * math.sin(self.yaw0)     # 转换局部坐标系下的速度到全局坐标系           
            path_point_tmp.vy = vx_local * math.sin(self.yaw0) + vy_local * math.cos(self.yaw0)              
            path_point_tmp.vyaw = vyaw_local                                                # 设置路径点的航向角速度

            
            path.append(path_point_tmp)                                                     # 将路径点添加到路径列表中

        return path

                                                                                            
 

def main(args=None):
    rclpy.init(args=args)                                                                   # 初始化rclpy
    dog_control_node = SportRequest("dog_control_pub")                                      # 创建SportRequest节点的实例
    rclpy.spin(dog_control_node)                                                            # 保持节点运行
    rclpy.shutdown()                                                                        # 关闭rclpy

if __name__ == '__main__':
    main()                                                                                  # 运行main函数


