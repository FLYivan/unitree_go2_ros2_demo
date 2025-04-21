#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, CompressedImage, LaserScan
import message_filters
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')
        
        # 创建计数器字典，用于跟踪每个话题的接收情况
        self.msg_counters = {
            '/camera/color/image_raw': 0,
            '/camera/depth/image_rect_raw': 0,
            'rgb/image/compressed': 0,
            'depth/image/compressed': 0,
            '/camera/color/camera_info': 0,
            '/lidar_points': 0,
            '/livox/imu': 0,
            '/scan': 0
        }
        
        # 创建最后接收时间字典
        self.last_msg_times = {topic: None for topic in self.msg_counters.keys()}
        
        # 创建CV桥接器用于图像转换
        self.bridge = CvBridge()
        
        # 创建发布器
        self.rgb_pub = self.create_publisher(Image, 'sync/rgb/image', 10)
        self.rgb_compressed_pub = self.create_publisher(CompressedImage, 'sync/rgb/image/compressed', 10)
        self.depth_pub = self.create_publisher(Image, 'sync/depth/image', 10)
        self.depth_compressed_pub = self.create_publisher(CompressedImage, 'sync/depth/image/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'sync/rgb/camera_info', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'sync/points', 10)
        self.imu_pub = self.create_publisher(Imu, 'sync/imu', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'sync/scan', 10)

        # 创建单独的回调来监控每个话题
        self.rgb_monitor = self.create_subscription(
            Image, '/camera/color/image_raw',
            lambda msg: self.monitor_callback(msg, '/camera/color/image_raw'), 10)
        
        self.depth_monitor = self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            lambda msg: self.monitor_callback(msg, '/camera/depth/image_rect_raw'), 10)
        
        self.rgb_compressed_monitor = self.create_subscription(
            CompressedImage, 'rgb/image/compressed',
            lambda msg: self.monitor_callback(msg, 'rgb/image/compressed'), 10)
        
        self.depth_compressed_monitor = self.create_subscription(
            CompressedImage, 'depth/image/compressed',
            lambda msg: self.monitor_callback(msg, 'depth/image/compressed'), 10)
        
        self.camera_info_monitor = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            lambda msg: self.monitor_callback(msg, '/camera/color/camera_info'), 10)
        
        self.pointcloud_monitor = self.create_subscription(
            PointCloud2, '/lidar_points',
            lambda msg: self.monitor_callback(msg, '/lidar_points'), 10)
        
        self.imu_monitor = self.create_subscription(
            Imu, '/livox/imu',
            lambda msg: self.monitor_callback(msg, '/livox/imu'), 10)
            
        self.scan_monitor = self.create_subscription(
            LaserScan, '/scan',
            lambda msg: self.monitor_callback(msg, '/scan'), 10)

        # 创建订阅器列表用于时间同步
        subs = []
        try:
            # 逐个创建订阅器并添加到列表中
            self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
            subs.append(self.rgb_sub)
            
            self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')
            subs.append(self.depth_sub)        
            
            self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
            subs.append(self.camera_info_sub)
            
            self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, '/lidar_points')
            subs.append(self.pointcloud_sub)
            
            self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
            subs.append(self.scan_sub)

            # 创建近似时间同步器
            self.ts = message_filters.ApproximateTimeSynchronizer(
                subs,
                queue_size=10,
                slop=0.1
            )
            self.ts.registerCallback(self.sync_callback)

        except Exception as e:
            self.get_logger().error(f'初始化时出错: {str(e)}')
            raise

        # 创建定时器，每秒打印一次状态报告
        self.create_timer(1.0, self.print_status_report)
        self.get_logger().info('传感器同步节点初始化完成')

    def monitor_callback(self, msg, topic_name):
        """监控每个话题的接收情况"""
        self.msg_counters[topic_name] += 1
        self.last_msg_times[topic_name] = datetime.now()
        
    def print_status_report(self):
        """打印状态报告"""
        self.get_logger().info("\n=== 传感器数据接收状态报告 ===")
        current_time = datetime.now()
        
        for topic, count in self.msg_counters.items():
            last_time = self.last_msg_times[topic]
            if last_time is None:
                status = "尚未收到数据"
                time_diff = "N/A"
            else:
                time_diff = (current_time - last_time).total_seconds()
                if time_diff < 1.0:
                    status = "活跃"
                elif time_diff < 5.0:
                    status = "最近有数据"
                else:
                    status = "长时间无数据"
                    
            self.get_logger().info(
                f"话题: {topic}\n"
                f"  - 总接收消息数: {count}\n"
                f"  - 状态: {status}\n"
                f"  - 上次接收时间: {time_diff if isinstance(time_diff, str) else f'{time_diff:.2f}秒前'}\n"
            )

    def sync_callback(self, rgb_msg, depth_msg,
                     camera_info_msg, pointcloud_msg, scan_msg):
        """处理同步后的传感器数据"""
        try:
            self.get_logger().info('收到同步数据，准备处理...')
            
            # 打印每个消息的关键信息
            self.get_logger().info(f'RGB消息时间戳: {rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec}')
            self.get_logger().info(f'深度消息时间戳: {depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec}')
            self.get_logger().info(f'相机信息消息时间戳: {camera_info_msg.header.stamp.sec}.{camera_info_msg.header.stamp.nanosec}')
            self.get_logger().info(f'点云消息时间戳: {pointcloud_msg.header.stamp.sec}.{pointcloud_msg.header.stamp.nanosec}')
            self.get_logger().info(f'激光扫描消息时间戳: {scan_msg.header.stamp.sec}.{scan_msg.header.stamp.nanosec}')
            
            # 获取当前ROS时间
            current_time = self.get_clock().now().to_msg()
            
            # 更新时间戳
            rgb_msg.header.stamp = current_time
            depth_msg.header.stamp = current_time
            camera_info_msg.header.stamp = current_time
            pointcloud_msg.header.stamp = current_time
            scan_msg.header.stamp = current_time
            
            # 发布同步后的消息
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            self.camera_info_pub.publish(camera_info_msg)
            self.pointcloud_pub.publish(pointcloud_msg)
            self.scan_pub.publish(scan_msg)
            
            self.get_logger().info('同步数据处理完成并发布')
            
        except Exception as e:
            self.get_logger().error(f'处理同步数据时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()