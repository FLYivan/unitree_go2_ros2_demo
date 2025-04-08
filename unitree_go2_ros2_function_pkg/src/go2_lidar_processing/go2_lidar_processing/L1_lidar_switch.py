#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class LidarSwtichPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 创建一个发布者，发布到控制指令话题
        self.publisher = self.create_publisher(String, '/utlidar/switch', 10)
        
        # 设置发布频率
        self.timer = self.create_timer(0.1, self.publish_lidar_switch)  # 每0.1秒发布一次

        self.start_time = self.get_clock().now()  # 获取开始时间

        # 标记非循环类控制指令是否已发布
        self.switchon_published = False
        self.switchoff_published = False


        self.get_logger().info('控制模式节点初始化成功')

    def publish_lidar_switch(self):

        """根据状态发布消息"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # 转换为秒

        if elapsed_time < 10 and not self.switchoff_published:
            # 前10秒发布A消息
            switch_msg = String()
            switch_msg.data = 'OFF'
            self.publisher.publish(switch_msg)
            self.get_logger().info(f'Published: {switch_msg.data}')
            self.switchoff_published = True

        
        elif elapsed_time >= 10 and not self.switchon_published:
            # 10秒后切换到发布趴下消息
            switch_msg = String()
            switch_msg.data = 'ON'
            self.publisher.publish(switch_msg)
            self.get_logger().info(f'Published: {switch_msg.data}')
            self.switchon_published = True
            self.timer.cancel()  # 停止定时器
        


def main(args=None):
    rclpy.init(args=args)
    node = LidarSwtichPublisher("L1_lidar_switch")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()