#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class VelocityPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 创建一个发布者，发布到控制指令话题
        self.publisher = self.create_publisher(String, 'cmd_control_command', 10)
        
        # 设置发布频率
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 每0.1秒发布一次

        self.start_time = self.get_clock().now()  # 获取开始时间

        # 标记非循环类控制指令是否已发布
        self.standup_published = False
        self.liedown_published = False

        self.get_logger().info('控制模式节点初始化成功')

    def publish_cmd(self):

        """根据状态发布消息"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # 转换为秒

        if elapsed_time < 10 :
            # 前10秒发布A消息
            cmd_msg = String()
            cmd_msg.data = 'velocity_control'
            self.publisher.publish(cmd_msg)
            self.get_logger().info(f'Published: {cmd_msg.data}')
            self.standup_published = False
            self.liedown_published = False

        
        elif elapsed_time >= 10 and not self.liedown_published:
            # 10秒后切换到发布趴下消息
            cmd_msg = String()
            cmd_msg.data = 'lie_down'
            self.publisher.publish(cmd_msg)
            self.get_logger().info(f'Published: {cmd_msg.data}')
            self.liedown_published = True
        
        elif elapsed_time >= 15 and not self.standup_published:
            # 等待5秒后发布起立消息
            cmd_msg = String()
            cmd_msg.data = 'stand'
            self.publisher.publish(cmd_msg)
            self.get_logger().info(f'Published: {cmd_msg.data}')
            self.standup_published = True
            self.timer.cancel()  # 停止定时器



def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher("cmd_pub_test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()