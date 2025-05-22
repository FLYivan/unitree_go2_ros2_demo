import rclpy  # 导入ROS2 Python客户端库
from nav_msgs.msg import Odometry  # 导入里程计消息类型
from sensor_msgs.msg import PointCloud2  # 导入点云消息类型

from .conversions import read_points  # 导入点云数据读取函数

import numpy as np  # 导入numpy用于数值计算
import open3d as o3d  # 导入open3d用于点云处理

from scipy.spatial.transform import Rotation  # 导入旋转矩阵转换工具

class LidarOdometry:  # 定义激光雷达里程计类
    def __init__(self):  # 初始化函数
        self.node = rclpy.create_node('lidar_odometry_node')  # 创建ROS2节点

        self.hesaicloud_pub = self.node.create_publisher(PointCloud2, 'hesai/after_point_cloud', 10)  # 创建里程计发布器

        self.hesaicloud_sub = self.node.create_subscription(  # 创建点云订阅器
            PointCloud2,
            '/lidar_points',
            self.listener_callback,
            10)
        self.hesaicloud_sub  # 保持订阅器引用


        self.prev_cloud = PointCloud2()  # 初始化前一帧点云为空




    def listener_callback(self, msg):  # 点云回调函数

        cloud = self.pointcloud2_to_pointcloud(msg)  # 将ROS点云消息转换为Open3D点云

        if self.prev_cloud is None:  # 如果是第一帧点云
            self.prev_cloud = cloud  # 保存当前点云作为参考帧

            return  # 返回等待下一帧

        self.prev_cloud = cloud  # 更新参考帧点云

        self.hesaicloud_pub.publish(self.prev_cloud)

    def pointcloud2_to_pointcloud(self, msg):  # ROS点云转Open3D点云函数

        data = read_points(msg, skip_nans=True, field_names=("y", "x", "z"))  # 读取点云数据

        cloud = o3d.geometry.PointCloud()  # 创建Open3D点云对象
        cloud.points = o3d.utility.Vector3dVector(data)  # 设置点云数据
        downcloud = cloud.voxel_down_sample(voxel_size=0.05)  # 进行体素降采样
        return downcloud  # 返回降采样后的点云





def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化ROS2

    lidar_odometry_node = LidarOdometry()  # 创建激光雷达里程计节点

    rclpy.spin(lidar_odometry_node.node)  # 运行节点

    lidar_odometry_node.node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':  # 主程序入口
    main()  # 运行主函数