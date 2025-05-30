#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState
import numpy as np
import math

class Go2IMUNode(Node):
    def __init__(self):
        super().__init__('go2_imu_node')
        
        # 创建IMU发布器,发布频率500Hz
        self.imu_pub = self.create_publisher(Imu, '/go2/imu', 10)
        
        # 订阅sport话题
        self.sport_sub = self.create_subscription(
            SportModeState,
            '/sportmodestate', 
            self.sport_callback,
            10)
            
        # 创建定时器,500Hz
        self.timer = self.create_timer(1.0/500.0, self.timer_callback)
        
        # 存储最新的IMU数据
        self.latest_imu = Imu()
        
        
        self.get_logger().info('Go2 IMU节点已启动')

    def sport_callback(self, msg):
        """处理sport话题回调"""
        try:
            # 获取原始角速度
            x = float(msg.imu_state.gyroscope[0])
            y = float(msg.imu_state.gyroscope[1]) 
            z = float(msg.imu_state.gyroscope[2])

     

            # 获取原始加速度
            acc_x = float(msg.imu_state.accelerometer[0])
            acc_y = float(msg.imu_state.accelerometer[1])
            acc_z = float(msg.imu_state.accelerometer[2])


            # 更新IMU消息
            self.latest_imu.header.stamp = self.get_clock().now().to_msg()
            self.latest_imu.header.frame_id = "imu_link"

            # 设置四元数
            self.latest_imu.orientation.w = float(msg.imu_state.quaternion[0])
            self.latest_imu.orientation.x = float(msg.imu_state.quaternion[1])
            self.latest_imu.orientation.y = float(msg.imu_state.quaternion[2]) 
            self.latest_imu.orientation.z = float(msg.imu_state.quaternion[3])

            # 设置角速度
            self.latest_imu.angular_velocity.x = x
            self.latest_imu.angular_velocity.y = y
            self.latest_imu.angular_velocity.z = z

            # 设置线加速度
            self.latest_imu.linear_acceleration.x = acc_x
            self.latest_imu.linear_acceleration.y = acc_y
            self.latest_imu.linear_acceleration.z = acc_z

            # 设置协方差矩阵
            self.latest_imu.orientation_covariance = np.zeros(9).tolist()
            self.latest_imu.angular_velocity_covariance = np.zeros(9).tolist()
            self.latest_imu.linear_acceleration_covariance = np.zeros(9).tolist()

        except Exception as e:
            self.get_logger().error(f'处理IMU数据时发生错误: {str(e)}')

    def timer_callback(self):
        """定时器回调,500Hz发布IMU数据"""
        try:
            if self.latest_imu.header.stamp.sec != 0:  # 确保有数据
                self.latest_imu.header.stamp = self.get_clock().now().to_msg()
                self.imu_pub.publish(self.latest_imu)
        except Exception as e:
            self.get_logger().error(f'发布IMU数据时发生错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = Go2IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
