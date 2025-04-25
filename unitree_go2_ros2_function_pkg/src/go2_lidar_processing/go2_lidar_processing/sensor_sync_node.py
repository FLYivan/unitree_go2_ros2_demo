#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, CompressedImage, LaserScan
import message_filters
from cv_bridge import CvBridge
import cv2
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sync_rgb', True),
                ('sync_depth', True), 
                ('sync_camera_info', True),
                ('sync_pointcloud', False),
                ('sync_imu', False),
                ('sync_scan', True),
                ('sync_rgb_compressed', False),
                ('sync_depth_compressed', False)
            ]
        )

        # 获取参数值
        self.sync_rgb = self.get_parameter('sync_rgb').value
        self.sync_depth = self.get_parameter('sync_depth').value
        self.sync_camera_info = self.get_parameter('sync_camera_info').value
        self.sync_pointcloud = self.get_parameter('sync_pointcloud').value
        self.sync_imu = self.get_parameter('sync_imu').value
        self.sync_scan = self.get_parameter('sync_scan').value
        self.sync_rgb_compressed = self.get_parameter('sync_rgb_compressed').value
        self.sync_depth_compressed = self.get_parameter('sync_depth_compressed').value

        # 创建CV桥接器用于图像转换
        self.bridge = CvBridge()

        # 发布使用RELIABLE QoS
        scan_pub_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST, 
                depth=10
            )

        # 根据参数创建发布器
        self.topic_publishers = {}
        if self.sync_rgb:
            self.topic_publishers['rgb'] = self.create_publisher(Image, 'sync/rgb/image', 10)
        if self.sync_depth:
            self.topic_publishers['depth'] = self.create_publisher(Image, 'sync/depth/image', 10)
        if self.sync_camera_info:
            self.topic_publishers['camera_info'] = self.create_publisher(CameraInfo, 'sync/rgb/camera_info', 10)
        if self.sync_pointcloud:
            self.topic_publishers['pointcloud'] = self.create_publisher(PointCloud2, 'sync/points', 10)
        if self.sync_imu:
            self.topic_publishers['imu'] = self.create_publisher(Imu, 'sync/imu', 10)
        if self.sync_scan:
            self.topic_publishers['scan'] = self.create_publisher(LaserScan, 'sync/scan', qos_profile=scan_pub_qos)
        if self.sync_rgb_compressed:
            self.topic_publishers['rgb_compressed'] = self.create_publisher(CompressedImage, 'sync/rgb/image/compressed', 10)
        if self.sync_depth_compressed:
            self.topic_publishers['depth_compressed'] = self.create_publisher(CompressedImage, 'sync/depth/image/compressed', 10)

        # 创建订阅器列表用于时间同步
        subs = []
        try:
            # 根据参数创建订阅器
            if self.sync_rgb:
                self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
                subs.append(self.rgb_sub)

            if self.sync_depth:
                self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')
                subs.append(self.depth_sub)

            if self.sync_camera_info:
                self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
                subs.append(self.camera_info_sub)
            
            if self.sync_pointcloud:
                self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, '/lidar_points')
                subs.append(self.pointcloud_sub)

            if self.sync_imu:
                self.imu_sub = message_filters.Subscriber(self, Imu, '/livox/imu')
                subs.append(self.imu_sub)
            
            if self.sync_scan:
                # 订阅使用BEST_EFFORT QoS
                scan_sub_qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10
                )
                self.scan_sub = message_filters.Subscriber(self, 
                                                         LaserScan, 
                                                         '/scan',
                                                         qos_profile=scan_sub_qos,
                                                         )
                subs.append(self.scan_sub)

            if self.sync_rgb_compressed:
                rgb_compressed_qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10
                )
                self.rgb_compressed_sub = message_filters.Subscriber(
                    self, 
                    CompressedImage, 
                    '/camera/color/image_raw/compressed',
                    qos_profile=rgb_compressed_qos
                )
                subs.append(self.rgb_compressed_sub)

            if self.sync_depth_compressed:
                depth_compressed_qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10
                )
                self.depth_compressed_sub = message_filters.Subscriber(
                    self, 
                    CompressedImage, 
                    '/camera/depth/image_rect_raw/compressedDepth',
                    qos_profile=depth_compressed_qos
                )
                subs.append(self.depth_compressed_sub)

            if len(subs) > 0:
                # 创建近似时间同步器
                self.ts = message_filters.ApproximateTimeSynchronizer(
                    subs,
                    queue_size=1000,
                    slop=0.3
                )
                self.ts.registerCallback(self.sync_callback)
            else:
                self.get_logger().warn('没有选择任何传感器进行同步!')

        except Exception as e:
            self.get_logger().error(f'初始化时出错: {str(e)}')
            raise

        self.get_logger().info('传感器同步节点初始化完成')

    def sync_callback(self, *args):             # *args可以接受任意数量的参数，打包成一个元组
        """处理同步后的传感器数据"""
        try:
            self.get_logger().info('收到同步数据，准备处理...')
            
            # 获取当前ROS时间
            current_time = self.get_clock().now().to_msg()
            
            # 遍历所有接收到的消息
            for msg, topic_type in zip(args, self.topic_publishers.keys()):
                # 更新时间戳
                msg.header.stamp = current_time
                # 发布消息
                self.topic_publishers[topic_type].publish(msg)
                self.get_logger().info(f'{topic_type}消息时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            
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