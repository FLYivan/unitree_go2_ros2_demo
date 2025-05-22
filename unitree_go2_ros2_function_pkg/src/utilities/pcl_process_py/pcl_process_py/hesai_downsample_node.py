import rclpy
from sensor_msgs.msg import PointCloud2
from .conversions import read_points
import numpy as np
import open3d.cpu.pybind as o3d  # 使用CPU版本的Open3D

class LidarOdometry:
    def __init__(self):
        self.node = rclpy.create_node('hesai_downsample_node')
        self.hesaicloud_pub = self.node.create_publisher(PointCloud2, 'hesai/after_point_cloud', 10)
        self.hesaicloud_sub = self.node.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.listener_callback,
            10)
        self.prev_cloud = None

    def listener_callback(self, msg):
        cloud = self.pointcloud2_to_pointcloud(msg)
        if self.prev_cloud is None:
            self.prev_cloud = cloud
            return
        self.prev_cloud = cloud
        self.hesaicloud_pub.publish(self.prev_cloud)

    def pointcloud2_to_pointcloud(self, msg):
        data = read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(data)
        downcloud = cloud.voxel_down_sample(voxel_size=0.05)
        return downcloud

def main(args=None):
    rclpy.init(args=args)
    lidar_odometry_node = LidarOdometry()
    rclpy.spin(lidar_odometry_node.node)
    lidar_odometry_node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()