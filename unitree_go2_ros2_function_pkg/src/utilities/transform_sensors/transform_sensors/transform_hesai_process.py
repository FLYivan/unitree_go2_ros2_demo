#!/usr/bin/env python
import rclpy  # 导入ROS2 Python客户端库
from rclpy.node import Node  # 导入ROS2节点类
from rclpy.time import Time  # 导入ROS2时间类
from sensor_msgs.msg import Imu  # 导入IMU消息类型
from sensor_msgs.msg import PointCloud2, PointField  # 导入点云消息类型
from geometry_msgs.msg import TransformStamped, Vector3  # 导入变换和向量消息类型
import sensor_msgs_py.point_cloud2 as pc2  # 导入点云处理工具
import tf_transformations  # 导入TF变换工具

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from transforms3d.quaternions import quat2mat  # 导入四元数到矩阵转换工具

from unitree_go.msg import SportModeState

import numpy as np  # 导入numpy库


class Repuber(Node):  # 定义传感器转换节点类
    def __init__(self):  # 初始化方法
        super().__init__('transform_hesai_process')  # 调用父类初始化方法


        # self.cloud_sub = self.create_subscription(PointCloud2, '/lidar_points', self.cloud_callback, 50)  # 创建点云订阅者
        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_result', self.cloud_callback, 50)  # 创建降采样点云订阅者

        self.cloud_pub = self.create_publisher(PointCloud2, '/lidar_after_process', 50)  # 创建转换后点云发布者

        self.imu_stationary_list = []  # 初始化IMU静止列表
        
        self.hesai_time_stamp_offset = 0  # 初始化hesai雷达时间戳偏移
        self.hesai_time_stamp_offset_set = False  # 初始化hesai雷达时间戳偏移设置标志
        self.go2imu_time_stamp_offset = 0  # 初始化go2自带IMU时间戳偏移
        self.go2imu_time_stamp_offset_set = False  # 初始化go2自带IMU时间戳偏移设置标志
        
        self.cam_offset = 0.2908  # 设置相机偏移量


        self.body2cloud_trans = TransformStamped()  # 创建机体到点云的变换
        self.body2cloud_trans.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        self.body2cloud_trans.header.frame_id = "body"  # 设置父坐标系
        self.body2cloud_trans.child_frame_id = "hesai_lidar_1"  # 设置子坐标系
        self.body2cloud_trans.transform.translation.x = 0.0  # 设置X轴平移
        self.body2cloud_trans.transform.translation.y = 0.0  # 设置Y轴平移
        self.body2cloud_trans.transform.translation.z = 0.0  # 设置Z轴平移
        # quat = tf_transformations.quaternion_from_euler(0, 0, 1.5708)  # 计算欧拉角到四元数的转换
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)  # 计算欧拉角到四元数的转换
        self.body2cloud_trans.transform.rotation.x = quat[0]  # 设置四元数X分量
        self.body2cloud_trans.transform.rotation.y = quat[1]  # 设置四元数Y分量
        self.body2cloud_trans.transform.rotation.z = quat[2]  # 设置四元数Z分量
        self.body2cloud_trans.transform.rotation.w = quat[3]  # 设置四元数W分量
        
        self.body2imu_trans = TransformStamped()  # 创建机体到IMU的变换
        self.body2imu_trans.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        self.body2imu_trans.header.frame_id = "body"  # 设置父坐标系
        self.body2imu_trans.child_frame_id = "go2_imu_1"  # 设置子坐标系
        self.body2imu_trans.transform.translation.x = 0.0  # 设置X轴平移
        self.body2imu_trans.transform.translation.y = 0.0  # 设置Y轴平移
        self.body2imu_trans.transform.translation.z = 0.0  # 设置Z轴平移
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)  # 计算欧拉角到四元数的转换
        self.body2imu_trans.transform.rotation.x = quat[0]  # 设置四元数X分量
        self.body2imu_trans.transform.rotation.y = quat[1]  # 设置四元数Y分量
        self.body2imu_trans.transform.rotation.z = quat[2]  # 设置四元数Z分量
        self.body2imu_trans.transform.rotation.w = quat[3]  # 设置四元数W分量
        
        # 实现了点云的空间过滤，定义了一个过滤框，在这个范围内的点会被过滤掉。
        self.x_filter_min = -0.55  # 设置X轴过滤最小值
        self.x_filter_max = 0.2  # 设置X轴过滤最大值
        self.y_filter_min = -0.15  # 设置Y轴过滤最小值
        self.y_filter_max = 0.15  # 设置Y轴过滤最大值
        self.z_filter_min = -0.5 
        self.z_filter_max = 0 

        rclpy.spin(self)  # 运行节点
                
    def is_in_filter_box(self, point):  # 检查点是否在过滤框内
        # 检查点是否在过滤框内
        is_in_box = point[0] > self.x_filter_min and \
                    point[0] < self.x_filter_max and \
                    point[1] > self.y_filter_min and \
                    point[1] < self.y_filter_max and \
                    point[2] > self.z_filter_min and \
                    point[2] < self.z_filter_max  # 检查所有维度是否在范围内
        return is_in_box  # 返回检查结果

    def cloud_callback(self, data):  # 点云回调函数

        # 第一次接收到点云数据时计算时间偏移
        # 计算当前ROS时间和传感器时间戳之间的差值：
        # offset = 当前ROS时间 - 传感器时间戳
        # (这个偏移量只计算一次，后续所有数据都使用这个偏移量)
        if not self.hesai_time_stamp_offset_set:  # 如果时间戳偏移未设置
            self.hesai_time_stamp_offset = self.get_clock().now().nanoseconds - Time.from_msg(data.header.stamp).nanoseconds  # 计算时间戳偏移
            self.hesai_time_stamp_offset_set = True  # 标记时间戳偏移已设置
                
        # 点云做切割处理        
        cloud_arr = pc2.read_points_list(data)  # 读取点云数据
        points = np.array(cloud_arr)  # 转换为numpy数组

        transform = self.body2cloud_trans.transform  # 获取变换信息
        mat = quat2mat(np.array([transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]))  # 四元数转旋转矩阵
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])  # 获取平移向量
        
        transformed_points = points  # 初始化变换后的点
        transformed_points[:, 0:3] = points[:, 0:3] @ mat.T + translation  # 应用旋转和平移
        transformed_points[:, 2] += self.cam_offset  # 应用相机偏移
        i = 0  # 初始化计数器
        remove_list = []  # 初始化移除列表
        transformed_points = transformed_points.tolist()  # 转换为列表

        for i in range(len(transformed_points)):  # 遍历所有点
            transformed_points[i][4] = int(transformed_points[i][4])  # 转换点云线数值为整数
            if self.is_in_filter_box(transformed_points[i]):  # 检查点是否在过滤框内
                remove_list.append(i)  # 添加到移除列表

        remove_list.sort(reverse=True)  # 反向排序移除列表

        for id_to_remove in remove_list:  # 遍历移除列表
            del transformed_points[id_to_remove]  # 移除点
        
        # 应用时间偏移到转换后的点云消息
        # 新时间戳 = 原始时间戳 + 时间偏移量
        elevated_cloud = pc2.create_cloud(data.header, data.fields, transformed_points)  # 创建新的点云消息
        elevated_cloud.header.stamp = Time(nanoseconds=Time.from_msg(elevated_cloud.header.stamp).nanoseconds + self.hesai_time_stamp_offset).to_msg()  # 更新时间戳
        elevated_cloud.header.frame_id = "body"  # 设置坐标系
        elevated_cloud.is_dense = data.is_dense  # 设置密度标志

        self.cloud_pub.publish(elevated_cloud)  # 发布转换后的点云



        # # 仅做时间同步，不做其他处理
        # raw_cloud = data  # 创建点云消息
        # raw_cloud.header.stamp = Time(nanoseconds=Time.from_msg(raw_cloud.header.stamp).nanoseconds + self.hesai_time_stamp_offset).to_msg()  # 更新时间戳
        # raw_cloud.header.frame_id = "body"  # 设置坐标系
        # raw_cloud.is_dense = data.is_dense  # 设置密度标志

        # self.cloud_pub.publish(raw_cloud)  # 发布转换后的点云


def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化ROS2

    transform_node = Repuber()  # 创建节点实例

    rclpy.spin(transform_node)  # 运行节点

    Repuber.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':  # 主程序入口
    main()  # 运行主函数
