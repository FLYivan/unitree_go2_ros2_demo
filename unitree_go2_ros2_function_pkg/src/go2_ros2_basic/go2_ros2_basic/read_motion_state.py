#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@说明: 通过go2提供的ROS2接口-订阅狗子当前状态
"""

import rclpy
import numpy as np
import time
from rclpy.node import Node
from unitree_go.msg import SportModeState

# foot_state = 1                     # Set 1 to info foot states (foot position and velocity in body frame)
# dog_freq = 0                       # Set 1 to subscribe to motion states with high frequencies (500Hz)


class MotionStateSubscriber(Node):
     def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化

        self.declare_parameter('INFO_FOOT_STATE', 0)                    # 获取足部状态的参数的参数化
        self.declare_parameter('CHOOSE_FREQ', 1)                        # 状态订阅参数的参数化

        dog_freq =self.get_parameter('CHOOSE_FREQ').get_parameter_value().integer_value # 根据这个参数切换是订阅高频状态还是低频状态
        

        topic_name = "lf/sportmodestate"
        if (dog_freq):
            topic_name = "sportmodestate"
     
        self.subscription = self.create_subscription(
            SportModeState,
            topic_name,
            self.topic_callback,
            10)   

        self.foot_pos = np.zeros(12,dtype=float)                    # 数组用于存储机器人的12个足部位置数据
        self.foot_vel = np.zeros(12,dtype=float)                    # 数组用于存储机器人的12个足部速度数据


        

     def topic_callback(self, data):
        # Info motion states
        # Gait type and foot raise height
        # Robot position (Odometry frame)
        # Robot velocity (Odometry frame)
        self.get_logger().info(f"lyf_dog Gait state -- gait type: {data.gait_type}; raise height: {data.foot_raise_height}")
        self.get_logger().info(f"lyf_dog Position -- x: {data.position[0]}; y: {data.position[1]}; z: {data.position[2]}; body height: {data.body_height}")
        self.get_logger().info(f"lyf_dog Velocity -- vx: {data.velocity[0]}; vy: {data.velocity[1]}; vz: {data.velocity[2]}; yaw: {data.yaw_speed}")

        foot_state = self.get_parameter('INFO_FOOT_STATE').get_parameter_value().integer_value  # Assuming this is a constant, adjust as needed
        
        if (foot_state):
            # Info foot states (foot position and velocity in body frame)
            for i in range(12):
                self.foot_pos[i] = data.foot_position_body[i]
                self.foot_vel[i] = data.foot_speed_body[i]

            for i in range(4):
                self.get_logger().info(f"Foot position and velocity relative to body -- num: {i}; " +
                                    f"x: {self.foot_pos[i*3]}; y: {self.foot_pos[i*3+1]}; z: {self.foot_pos[i*3+2]}, " +
                                    f"vx: {self.foot_vel[i*3]}; vy: {self.foot_vel[i*3+1]}; vz: {self.foot_vel[i*3+2]}")
                

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    motion_state_subscriber = MotionStateSubscriber("dog_state_sub") 

    try:
        while rclpy.ok():
            rclpy.spin_once(motion_state_subscriber)  # 处理一次回调
            time.sleep(2)  # 设置时间间隔为2秒
    except KeyboardInterrupt:
        pass
    finally:
        motion_state_subscriber.destroy_node()
        rclpy.shutdown()



    # rclpy.spin(motion_state_subscriber)              # 循环等待ROS2退出
    # motion_state_subscriber.destroy_node()           # 销毁节点对象
    # rclpy.shutdown()                                 # 关闭ROS2 Python接口



 





if __name__ == '__main__':                          # 检查当前模块是否是主程序。
    main()                                          
