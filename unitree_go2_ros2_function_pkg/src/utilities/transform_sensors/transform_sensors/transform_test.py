#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, Vector3
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from transforms3d.quaternions import quat2mat
from unitree_go.msg import SportModeState
import numpy as np
import yaml
import os
from threading import Thread
from queue import Queue, Full
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class Repuber(Node):
    def __init__(self):
        super().__init__('transform_test')
        
        # 创建独立的回调组
        self.imu_callback_group = ReentrantCallbackGroup()
        self.cloud_callback_group = ReentrantCallbackGroup()
        
        # 为IMU创建高性能QoS
        imu_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,  # 使用BEST_EFFORT提高实时性
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # 为点云创建可靠QoS
        cloud_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # IMU相关的订阅和发布使用独立的回调组和QoS
        self.imu_sub = self.create_subscription(
            SportModeState,
            '/sportmodestate',
            self.imu_callback,
            imu_qos,
            callback_group=self.imu_callback_group
        )
        
        self.imu_raw_pub = self.create_publisher(
            Imu,
            '/hesai_go2/transformed_raw_imu',
            imu_qos,
            callback_group=self.imu_callback_group
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/hesai_go2/transformed_imu',
            imu_qos,
            callback_group=self.imu_callback_group
        )
        
        # 点云相关的订阅和发布使用独立的回调组和QoS
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.cloud_callback,
            cloud_qos,
            callback_group=self.cloud_callback_group
        )
        
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/hesai_go2/transformed_cloud',
            cloud_qos,
            callback_group=self.cloud_callback_group
        )
        
        # 创建点云处理队列和线程
        self.cloud_queue = Queue(maxsize=1)
        self.cloud_thread = Thread(target=self.cloud_processing_thread, daemon=True)
        self.cloud_thread.start()
        
        # 初始化变量
        self.init_variables()
        self.setup_transforms()
        
    def init_variables(self):
        self.imu_stationary_list = []
        self.hesai_time_stamp_offset = 0
        self.hesai_time_stamp_offset_set = False
        self.go2imu_time_stamp_offset = 0
        self.go2imu_time_stamp_offset_set = False
        self.cam_offset = 0.0908
        
        # 加载标定数据
        calib_data = {
            'acc_bias_x': 0.0, 'acc_bias_y': 0.0, 'acc_bias_z': 0.0,
            'ang_bias_x': 0.0, 'ang_bias_y': 0.0, 'ang_bias_z': 0.0,
            'ang_z2x_proj': 0.15, 'ang_z2y_proj': -0.28
        }
        
        try:
            home_path = os.path.expanduser('~')
            calib_file_path = os.path.join(home_path, '桌面/imu_calib_data.yaml')
            with open(calib_file_path, 'r') as calib_file:
                calib_data = yaml.load(calib_file, Loader=yaml.FullLoader)
            print("imu_calib.yaml loaded")
        except:
            print("imu_calib.yaml not found, using default values")
        
        # 设置标定参数
        for key, value in calib_data.items():
            setattr(self, key, value)
        
        # 设置点云过滤框
        self.x_filter_min, self.x_filter_max = -0.55, 0.2
        self.y_filter_min, self.y_filter_max = -0.15, 0.15
        self.z_filter_min, self.z_filter_max = -0.5, 0
        
    def setup_transforms(self):
        # 设置机体到点云的变换
        self.body2cloud_trans = TransformStamped()
        self.body2cloud_trans.header.frame_id = "body"
        self.body2cloud_trans.child_frame_id = "hesai_lidar_1"
        quat = tf_transformations.quaternion_from_euler(0, 0, 1.5708)
        self.body2cloud_trans.transform.rotation.x, self.body2cloud_trans.transform.rotation.y, \
        self.body2cloud_trans.transform.rotation.z, self.body2cloud_trans.transform.rotation.w = quat
        
        # 预计算点云变换矩阵
        self.cloud_rotation_matrix = quat2mat([quat[3], quat[0], quat[1], quat[2]])
        self.cloud_translation = np.array([0.0, 0.0, 0.0])
        
        # 设置机体到IMU的变换
        self.body2imu_trans = TransformStamped()
        self.body2imu_trans.header.frame_id = "body"
        self.body2imu_trans.child_frame_id = "go2_imu_1"
        self.body2imu_trans.transform.translation.x = 0.0
        self.body2imu_trans.transform.translation.y = 0.0
        self.body2imu_trans.transform.translation.z = 0.0
        imu_quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.body2imu_trans.transform.rotation.x = imu_quat[0]
        self.body2imu_trans.transform.rotation.y = imu_quat[1]
        self.body2imu_trans.transform.rotation.z = imu_quat[2]
        self.body2imu_trans.transform.rotation.w = imu_quat[3]
        
        # 预计算IMU变换矩阵
        self.imu_rotation_matrix = quat2mat([imu_quat[3], imu_quat[0], imu_quat[1], imu_quat[2]])
        self.imu_translation = np.array([0.0, 0.0, 0.0])
        
    def imu_callback(self, data):
        try:
            # 打印接收到IMU数据的信息
            # self.get_logger().info("收到IMU数据")

            # 快速处理IMU数据
            ros_time = Time(seconds=data.stamp.sec, nanoseconds=data.stamp.nanosec)
            
            if not self.go2imu_time_stamp_offset_set:
                self.go2imu_time_stamp_offset = self.get_clock().now().nanoseconds - ros_time.nanoseconds
                self.go2imu_time_stamp_offset_set = True
            
            # 创建IMU消息
            transformed_imu = Imu()
            transformed_imu.header.frame_id = 'body'
            transformed_imu.header.stamp = Time(nanoseconds=ros_time.nanoseconds + self.go2imu_time_stamp_offset).to_msg()
            
            # 设置姿态
            transformed_imu.orientation.x = float(data.imu_state.quaternion[1])
            transformed_imu.orientation.y = float(data.imu_state.quaternion[2])
            transformed_imu.orientation.z = float(data.imu_state.quaternion[3])
            transformed_imu.orientation.w = float(data.imu_state.quaternion[0])
            
            # 处理角速度
            gyro = np.array(data.imu_state.gyroscope) - np.array([self.ang_bias_x, self.ang_bias_y, self.ang_bias_z])
            gyro[0] += self.ang_z2x_proj * gyro[2]
            gyro[1] += self.ang_z2y_proj * gyro[2]
            
            transformed_imu.angular_velocity.x = gyro[0]
            transformed_imu.angular_velocity.y = gyro[1]
            transformed_imu.angular_velocity.z = gyro[2]
            
            # 处理加速度
            acc = np.array(data.imu_state.accelerometer) - np.array([self.acc_bias_x, self.acc_bias_y, self.acc_bias_z])
            
            transformed_imu.linear_acceleration.x = acc[0]
            transformed_imu.linear_acceleration.y = acc[1]
            transformed_imu.linear_acceleration.z = acc[2]
            
            # 发布原始IMU数据
            self.imu_raw_pub.publish(transformed_imu)
            # self.get_logger().info("已发布原始IMU数据")
            
            # 发布转换后的IMU数据
            transformed_imu_zero = Imu()
            transformed_imu_zero.header = transformed_imu.header
            transformed_imu_zero.orientation.w = 1.0
            self.imu_pub.publish(transformed_imu_zero)
            
        except Exception as e:
            self.get_logger().error(f'IMU处理错误: {str(e)}')
    
    def cloud_callback(self, data):
        try:
            self.cloud_queue.put_nowait(data)
        except Full:
            try:
                self.cloud_queue.get_nowait()
                self.cloud_queue.put_nowait(data)
            except:
                pass
    
    def cloud_processing_thread(self):
        while True:
            try:
                data = self.cloud_queue.get()
                
                if not self.hesai_time_stamp_offset_set:
                    self.hesai_time_stamp_offset = self.get_clock().now().nanoseconds - Time.from_msg(data.header.stamp).nanoseconds
                    self.hesai_time_stamp_offset_set = True
                
                # 高效的点云处理
                points = np.array(pc2.read_points_list(data))
                
                # 应用变换
                points_xyz = points[:, 0:3] @ self.cloud_rotation_matrix.T + self.cloud_translation
                points_xyz[:, 2] += self.cam_offset
                
                # 使用向量化操作进行过滤
                mask = ~((points_xyz[:, 0] > self.x_filter_min) & 
                        (points_xyz[:, 0] < self.x_filter_max) & 
                        (points_xyz[:, 1] > self.y_filter_min) & 
                        (points_xyz[:, 1] < self.y_filter_max) & 
                        (points_xyz[:, 2] > self.z_filter_min) & 
                        (points_xyz[:, 2] < self.z_filter_max))
                
                # 应用过滤并创建新的点云
                filtered_points = points[mask].copy()
                filtered_points[:, 0:3] = points_xyz[mask]
                filtered_points[:, 4] = filtered_points[:, 4].astype(int)
                
                # 创建并发布点云消息
                cloud_msg = pc2.create_cloud(data.header, data.fields, filtered_points.tolist())
                cloud_msg.header.stamp = Time(nanoseconds=Time.from_msg(data.header.stamp).nanoseconds + self.hesai_time_stamp_offset).to_msg()
                cloud_msg.header.frame_id = "body"
                cloud_msg.is_dense = data.is_dense
                
                self.cloud_pub.publish(cloud_msg)
                
            except Exception as e:
                self.get_logger().error(f'点云处理错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = Repuber()
    
    # 创建多线程执行器，使用4个线程
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()