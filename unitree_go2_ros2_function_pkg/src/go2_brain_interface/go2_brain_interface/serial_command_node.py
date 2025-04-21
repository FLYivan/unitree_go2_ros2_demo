#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialCommandNode(Node):
    def __init__(self):
        super().__init__('serial_command_node')
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',  # 串口设备名，可能需要根据实际情况修改
                baudrate=115200,      # 波特率
                timeout=0.1           # 读取超时时间
            )
            self.get_logger().info('串口初始化成功')
        except serial.SerialException as e:
            self.get_logger().error(f'串口初始化失败: {str(e)}')
            raise

        # 创建运动控制发布器
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建状态发布器（用于调试）
        self.status_pub = self.create_publisher(String, 'serial_status', 10)
        
        # 创建定时器，定期检查串口数据
        self.create_timer(0.01, self.timer_callback)  # 100Hz的检查频率
        
        # 初始化计数器
        self.counter = 0
        
        self.get_logger().info('串口命令节点已启动')

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                # 读取串口数据
                data = self.ser.read(self.ser.in_waiting).decode().strip()
                self.counter += 1
                
                # 发布状态信息（用于调试）
                status_msg = String()
                status_msg.data = f'接收到数据: {data}, 计数: {self.counter}, 时间: {time.time()}'
                self.status_pub.publish(status_msg)
                
                # 根据不同的数据值调用不同的运控函数
                self.process_command(data)
                
            except Exception as e:
                self.get_logger().error(f'数据处理错误: {str(e)}')

    def process_command(self, data):
        """根据接收到的数据执行相应的运动控制"""
        cmd_vel = Twist()
        
        try:
            # 根据不同的命令设置不同的运动参数
            if data == 'forward':
                cmd_vel.linear.x = 0.5  # 向前移动
                self.get_logger().info('执行前进命令')
            elif data == 'backward':
                cmd_vel.linear.x = -0.5  # 向后移动
                self.get_logger().info('执行后退命令')
            elif data == 'left':
                cmd_vel.angular.z = 0.5  # 向左转
                self.get_logger().info('执行左转命令')
            elif data == 'right':
                cmd_vel.angular.z = -0.5  # 向右转
                self.get_logger().info('执行右转命令')
            elif data == 'stop':
                # 所有速度设为0
                self.get_logger().info('执行停止命令')
            else:
                self.get_logger().warn(f'未知命令: {data}')
                return
            
            # 发布运动控制命令
            self.cmd_vel_pub.publish(cmd_vel)
            
        except Exception as e:
            self.get_logger().error(f'命令执行错误: {str(e)}')

    def __del__(self):
        """析构函数，确保关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 