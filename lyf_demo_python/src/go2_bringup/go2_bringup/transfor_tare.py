import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


from unitree_go.msg import SportModeState

foot_state = 0                     # Set 1 to info foot states (foot position and velocity in body frame)
dog_freq = 1                       # Set 1 to subscribe to motion states with high frequencies (500Hz)

if dog_freq == 0 :
        topic_name = "lf/sportmodestate"
elif dog_freq == 1 :
    topic_name = "sportmodestate"

lidar_topic_name = "lidar_points"


# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色




class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('motion_to_tf')
                         
        self.x = 0.0                                    # 初始化x位置
        self.y = 0.0                                    # 初始化y位置
        self.z = 0.0                                    # 初始化z位置
        self.vx = 0.0                                   # 初始化x方向速度
        self.vy = 0.0                                   # 初始化y方向速度
        self.vz = 0.0                                   # 初始化z方向速度
        self.vthx = 0.0                                 # 初始化角速度
        self.vthy = 0.0 
        self.vthz = 0.0                                 
        self.quat_w = 0.0                               # 初始化四元数
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0


        self.accex = 0.0
        self.accey = 0.0
        self.accez = 9.81  # 重力加速度

        
        # 自定义QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,     # 可靠性策略： 确保消息被传递，如果失败会尝试重传
            history=QoSHistoryPolicy.KEEP_LAST,            # 历史策略： 只保留最新的N条消息
            depth=10                                       # N=10
        )

        # 创建一个订阅器，用于订阅消息
        self.motion_sub = self.create_subscription(SportModeState, topic_name, self.motion_callback, qos_profile)   

        self.lidar_sub = self.create_subscription(PointCloud2, lidar_topic_name, self.lidar_callback, qos_profile)

        # 创建一个发布器，用于发布IMU消息
        self.imu_pub = self.create_publisher(Imu, '/hesai/imu', qos_profile)    
        self.imu = Imu()                               # 创建一个Imu消息对象

        # 创建一个发布器，用于发布PointCloud消息
        self.lidar_pub = self.create_publisher(PointCloud2, '/hesai/lidar', qos_profile)    
        self.lidar = PointCloud2()                               # 创建一个PointCloud2消息对象

        # 将定时器间隔设置为0.05秒（即20Hz）
        self.publish_frequency = 100                  # 设置发布频率为20Hz
        self.publish_period = 1.0 / self.publish_frequency                                  # 计算发布周期
        self.timer = self.create_timer(self.publish_period , self.publish_all)        # 动态tf发布的频率为20hz(不可动参数)
        

    def motion_callback(self, msg):                     # 定义运动状态回调函数
        """
        获取基本状态
        """
        # 获取基于坐标系A的位置信息
            # 获取当前位置信息
        position_A = np.array([
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        ])

             # 获取当前四元数
        orientation_A = np.array([
            float(msg.imu_state.quaternion[1]),
            float(msg.imu_state.quaternion[2]),
            float(msg.imu_state.quaternion[3]),
            float(msg.imu_state.quaternion[0])
        ])

        if self.initial_position_A is None and self.initial_orientation_A is None:
            # 设置初始位置和姿态偏移
            self.initial_position_A = position_A
            self.initial_orientation_A = R.from_quat(orientation_A)

            # self.get_logger().info(f'Set initial position: {self.initial_position_A}')
            # self.get_logger().info(f'Set initial orientation: {self.initial_orientation_A.as_quat()}')
            return  # 记录初始位置和姿态后，返回以等待下一个回调
        
        # 计算从A到B的变换
        self.position_offset = self.initial_position_B - self.initial_position_A
        self.rotation_offset = self.initial_orientation_B * self.initial_orientation_A.inv()    # 将坐标系A的方向调整到坐标系B的方向

        # 计算相对于坐标系B的位置信息
        relative_position = self.rotation_offset.apply(position_A - self.initial_position_A)

        # 计算相对于坐标系B的旋转
        rotation_A = R.from_quat(orientation_A)
        relative_rotation = self.rotation_offset * rotation_A


        # 更新角速度
        self.vthx = float(msg.imu_state.gyroscope[0])                           # 从消息中获取x轴角速度
        self.vthy = float(msg.imu_state.gyroscope[1])                           # 从消息中获取y轴角速度
        self.vthz = float(msg.imu_state.gyroscope[2])                           # 从消息中获取z轴角速度


        # 更新线性加速度
        self.accex = float(msg.imu_state.accelerometer[0])      # 从消息中获取x轴加速度
        self.accey = float(msg.imu_state.accelerometer[1])      # 从消息中获取y轴加速度
        self.accez = float(msg.imu_state.accelerometer[2])      # 从消息中获取z轴加速度


        
        """
        IMU话题信息生成
        """
        # 设置消息头
        self.imu.header.stamp = self.get_clock().now().to_msg()    # 以上位机时间戳为准
    
        self.imu.header.frame_id = 'hesai_lidar'                   # 设置消息的坐标系ID
       

        
        # 设置线性加速度
        self.imu.linear_acceleration.x = self.accex
        self.imu.linear_acceleration.y = self.accey
        self.imu.linear_acceleration.z = self.accez
        
        # 设置角速度
        self.imu.angular_velocity.x = self.vthx
        self.imu.angular_velocity.y = self.vthy
        self.imu.angular_velocity.z = self.vthz
        
        # 设置四元数姿态
        self.imu.orientation.w = relative_rotation.as_quat()[3]
        self.imu.orientation.x = relative_rotation.as_quat()[0]
        self.imu.orientation.y = relative_rotation.as_quat()[1]
        self.imu.orientation.z = relative_rotation.as_quat()[2]

    def lidar_callback(self):
        self.lidar.header.stamp = self.get_clock().now().to_msg()    # 以上位机时间戳为准    


    # 回调函数，处理接收到的odom消息，并发布tf
    def publish_all(self):
        
        # 发布IMU消息
        self.imu_pub.publish(self.imu) 

        # 发布点云消息
        self.lidar_pub.publish(self.lidar) 


def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_publisher = DynamicTFPublisher()
    rclpy.spin(dynamic_tf_publisher)
    dynamic_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()