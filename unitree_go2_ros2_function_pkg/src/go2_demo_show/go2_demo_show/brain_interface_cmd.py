#!/usr/bin/env python3

"""
接受脑机接口端给出的控制信号，并转换为go2的实际运控接口调用
"""

import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String
from unitree_api.msg import Request  # 导入Request消息类型
from go2_sport.ros2_sport_client import (SportClient,PathPoint)

class SerialCommandNode(Node):
    def __init__(self):
        super().__init__('brain_interface_cmd')
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',  # 串口设备名，可能需要根据实际情况修改
                baudrate=921600,      # 波特率
                timeout=0.1           # 读取超时时间
            )
            self.ser.close()  # 先关闭串口
            self.ser.open()   # 再打开串口
            self.get_logger().info('串口初始化成功')
        except serial.SerialException as e:
            self.get_logger().error(f'串口初始化失败: {str(e)}')
            raise

         # 初始化发布者
        self.req_puber = self.create_publisher(Request, '/api/sport/request', 10)    
        
        # 创建状态发布器（用于调试）
        self.status_pub = self.create_publisher(String, 'serial_status', 10)
        
        # 创建定时器，定期检查串口数据
        self.create_timer(0.01, self.timer_callback)  # 100Hz的检查频率

        self.req = Request()                                                            # 初始化Request消息
        self.sport_req = SportClient()                                                  # 实例化一个SportClient对象(这里是json封装的client)

        self.vx = 0.0                                                                    # 初始化初始x位置
        self.vy = 0.0                                                                    # 初始化初始y位置
        self.vyaw = 0.0                                                                  # 初始化初始偏航角
        
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
                self.get_logger().info(status_msg.data)
                self.status_pub.publish(status_msg)
                
                # self.get_logger().info(f'{data}')

                # 根据不同的数据值调用不同的运控函数
                self.process_command(data)
                
            except Exception as e:
                self.get_logger().error(f'数据处理错误: {str(e)}')

        # # 测试运控方法
        # data = "1"
        # self.process_command(data)

    def process_command(self, data):
        """根据接收到的数据执行相应的运动控制"""
        
        try:
            # 根据不同的命令设置不同的运动参数
            if data == '1':                             
                self.vx = 0.3 # 向前移动
                self.vel_contrl(self.vx, self.vy, self.vyaw)
                self.get_logger().info('执行前进命令')
            elif data == '2':
                self.vx = -0.1  # 向后移动
                self.vel_contrl(self.vx, self.vy, self.vyaw)
                self.get_logger().info('执行后退命令')
            elif data == '3':
                self.vx = 0.1
                self.vyaw = 0.5 # 向左转
                self.vel_contrl(self.vx, self.vy, self.vyaw)
                self.get_logger().info('执行左转命令')
            elif data == '4':
                self.vx = 0.1
                self.vyaw = -0.5  # 向右转
                self.vel_contrl(self.vx, self.vy, self.vyaw)
                self.get_logger().info('执行右转命令')
            elif data == '5':
                self.sport_req.StopMove(self.req)                                           # 获取与高级运动命令对应的请求消息
                self.req_puber.publish(self.req)                                            # 发布请求消息   
                self.get_logger().info('执行停止命令')
            else:
                self.get_logger().warn(f'未知命令: {data}')
                return
            
            
        except Exception as e:
            self.get_logger().error(f'命令执行错误: {str(e)}')


    # 向狗子高层控制接口发送速度指令
    def vel_contrl(self,vx:float, vy:float, vyaw : float):
        
        self.sport_req.Move(self.req,vx, vy, vyaw)                              # 获取与高级运动命令对应的请求消息
        self.req_puber.publish(self.req)                                        # 发布速度命令
        # self.get_logger().info(f'当前x方向速度{vx}， y方向速度{vy}， 偏航角速度{vyaw}')

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