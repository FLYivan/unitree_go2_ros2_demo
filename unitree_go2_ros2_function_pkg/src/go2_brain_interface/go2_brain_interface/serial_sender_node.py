#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

class SerialSenderNode(Node):
    def __init__(self):
        super().__init__('serial_sender_node')
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB1',  # 使用不同的串口，避免与接收端冲突
                baudrate=115200,      # 保持与接收端相同的波特率
                timeout=0.1
            )
            self.get_logger().info('串口初始化成功')
        except serial.SerialException as e:
            self.get_logger().error(f'串口初始化失败: {str(e)}')
            raise

        # 创建状态发布器（用于调试）
        self.status_pub = self.create_publisher(String, 'serial_send_status', 10)
        
        # 创建定时器，定期发送数据
        self.create_timer(0.1, self.timer_callback)  # 10Hz的发送频率
        
        # 初始化计数器
        self.counter = 0
        
        self.get_logger().info('串口发送节点已启动')

    def timer_callback(self):
        try:
            # 发送数据 "1"
            data = "1"
            self.ser.write(data.encode())
            self.counter += 1
            
            # 发布状态信息（用于调试）
            status_msg = String()
            status_msg.data = f'发送数据: {data}, 计数: {self.counter}, 时间: {time.time()}'
            self.status_pub.publish(status_msg)
            
            self.get_logger().debug(f'已发送数据: {data}')
            
        except Exception as e:
            self.get_logger().error(f'数据发送错误: {str(e)}')

    def __del__(self):
        """析构函数，确保关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 