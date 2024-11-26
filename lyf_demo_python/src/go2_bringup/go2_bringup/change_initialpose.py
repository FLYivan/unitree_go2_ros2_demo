import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

from unitree_go.msg import SportModeState

foot_state = 0                     # Set 1 to info foot states (foot position and velocity in body frame)
dog_freq = 1                       # Set 1 to subscribe to motion states with high frequencies (500Hz)

if dog_freq == 0 :
        topic_name = "lf/sportmodestate"
elif dog_freq == 1 :
    topic_name = "sportmodestate"


# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色


class OdomTransformer(Node):

    def __init__(self):
        super().__init__('change_initialpose')

        self.vx = 0.0                                   # 初始化x方向速度
        self.vy = 0.0                                   # 初始化y方向速度
        self.vz = 0.0                                   # 初始化z方向速度
        self.vthx = 0.0                                 # 初始化角速度
        self.vthy = 0.0 
        self.vthz = 0.0   

        # 创建odomB话题发布者
        self.odom_pub = self.create_publisher(Odometry, 'odomB', 10)

        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_transform_stamped = TransformStamped()

        # 初始变换：从坐标系A到坐标系B
        self.initial_position_A = None
        self.initial_orientation_A = None


        # 坐标系B的初始位置和旋转
        self.initial_position_B = np.array([0, 0, 0.05])
        self.initial_orientation_B = R.from_quat([0, 0, 0, 1])


        # 订阅基于坐标系A的odomA话题
        self.motion_sub = self.create_subscription(SportModeState, topic_name, self.odom_callback, 10)   

    def odom_callback(self, msg):
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


        # 更新角速度
        self.vthx = float(msg.imu_state.gyroscope[0])                           # 从消息中获取x轴角速度
        self.vthy = float(msg.imu_state.gyroscope[1])                           # 从消息中获取y轴角速度
        self.vthz = float(msg.imu_state.gyroscope[2])                           # 从消息中获取z轴角速度


        # 获取并更新线速度
        self.vx = float(msg.velocity[0])   # 获取x轴线速度
        self.vy = float(msg.velocity[1])   # 获取y轴线速度
        self.vz = float(msg.velocity[2])   # 获取z轴线速度


        if self.initial_position_A is None and self.initial_orientation_A is None:
            # 设置初始位置和姿态偏移
            self.initial_position_A = position_A
            self.initial_orientation_A = R.from_quat(orientation_A)

            self.get_logger().info(f'Set initial position: {self.initial_position_A}')
            self.get_logger().info(f'Set initial orientation: {self.initial_orientation_A.as_quat()}')
            return  # 记录初始位置和姿态后，返回以等待下一个回调

        # 计算从A到B的变换
        self.position_offset = self.initial_position_B - self.initial_position_A
        self.rotation_offset = self.initial_orientation_B * self.initial_orientation_A.inv()

        # 计算相对于坐标系B的位置信息
        relative_position = self.rotation_offset.apply(position_A - self.initial_position_A)

        # 计算相对于坐标系B的旋转
        rotation_A = R.from_quat(orientation_A)
        relative_rotation = self.rotation_offset * rotation_A

        # 创建并发布odomB消息
        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odomB'
        self.odom_msg.child_frame_id = 'base_link'

        # 设置转换后的位置和方向
        self.odom_msg.pose.pose.position.x = relative_position[0]
        self.odom_msg.pose.pose.position.y = relative_position[1]
        self.odom_msg.pose.pose.position.z = relative_position[2]
        self.odom_msg.pose.pose.orientation.x = relative_rotation.as_quat()[0]
        self.odom_msg.pose.pose.orientation.y = relative_rotation.as_quat()[1]
        self.odom_msg.pose.pose.orientation.z = relative_rotation.as_quat()[2]
        self.odom_msg.pose.pose.orientation.w = relative_rotation.as_quat()[3]

        self.odom_msg.twist.twist.linear.x = self.vx                     # 设置线速度x
        self.odom_msg.twist.twist.linear.y = self.vy                     # 设置线速度y
        self.odom_msg.twist.twist.linear.z = self.vz                     # 设置线速度z
        self.odom_msg.twist.twist.angular.x = self.vthx                  # 设置角速度x
        self.odom_msg.twist.twist.angular.y = self.vthy                  # 设置角速度y
        self.odom_msg.twist.twist.angular.z = self.vthz                  # 设置角速度z

        # self.odom_pub.publish(self.odom_msg)

        # 创建并发布TF变换
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'odomB'
        self.transform.child_frame_id = 'base_link'
        self.transform.transform.translation.x = relative_position[0]
        self.transform.transform.translation.y = relative_position[1]
        self.transform.transform.translation.z = relative_position[2]

        # self.transform.transform.translation.x = position_A[0]
        # self.transform.transform.translation.y = position_A[1]
        # self.transform.transform.translation.z = position_A[2]




        self.transform.transform.rotation.x = relative_rotation.as_quat()[0]
        self.transform.transform.rotation.y = relative_rotation.as_quat()[1]
        self.transform.transform.rotation.z = relative_rotation.as_quat()[2]
        self.transform.transform.rotation.w = relative_rotation.as_quat()[3]


        # self.transform.transform.rotation.x = orientation_A[0]
        # self.transform.transform.rotation.y = orientation_A[1]
        # self.transform.transform.rotation.z = orientation_A[2]
        # self.transform.transform.rotation.w = orientation_A[3]


        self.tf_broadcaster.sendTransform(self.transform)


        """
        静态tf生成
        """
        # 使用雷达的时间戳

        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()

        self.static_transform_stamped.header.frame_id = 'base_link'  # 目标frame_id
        self.static_transform_stamped.child_frame_id = 'ridar'  # 原始frame_id rslidar


        self.static_transform_stamped.transform.translation.x = 0.171
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.0908

        # self.static_transform_stamped.transform.rotation.x = 0.0
        # self.static_transform_stamped.transform.rotation.y = 0.0
        # self.static_transform_stamped.transform.rotation.z = 0.0
        # self.static_transform_stamped.transform.rotation.w = 1.0


        q = R.from_euler('z', np.radians(90)).as_quat()
        self.static_transform_stamped.transform.rotation.x = q[0]
        self.static_transform_stamped.transform.rotation.y = q[1]
        self.static_transform_stamped.transform.rotation.z = q[2]
        self.static_transform_stamped.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(self.static_transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()