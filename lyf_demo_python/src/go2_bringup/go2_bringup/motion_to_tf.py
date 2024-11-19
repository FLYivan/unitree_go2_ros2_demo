import rclpy
import subprocess

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, Buffer
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


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

# 硬编码密码
password = "123"
remote_ip = "192.168.123.18"
user = "unitree"


class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('motion_to_tf')
                         
        self.x = 0.0                                    # 初始化x位置
        self.y = 0.0                                    # 初始化y位置
        self.z = 0.0                                    # 初始化z位置
        self.th = 0.0                                   # 初始化角度
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

        # 自定义QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,     # 可靠性策略： 确保消息被传递，如果失败会尝试重传
            history=QoSHistoryPolicy.KEEP_LAST,            # 历史策略： 只保留最新的N条消息
            depth=10                                       # N=10
        )

        # 创建一个订阅器，用于订阅消息
        self.motion_sub = self.create_subscription(SportModeState, topic_name, self.motion_callback, qos_profile)   


        # 创建一个发布器，用于发布Odometry消息
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)    

        # 创建静态变换广播器
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_transform_stamped = TransformStamped()

        # 创建动态 TF 缓存和广播器
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))  # 设置缓存时间
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()  # 初始化 transform

        # 将定时器间隔设置为0.05秒（即20Hz）
        self.publish_frequency = 20                  # 设置发布频率为20Hz
        self.publish_period = 1.0 / self.publish_frequency                                  # 计算发布周期
        self.timer = self.create_timer(self.publish_period , self.publish_all)        # 动态tf发布的频率为20hz(不可动参数)
        

    def motion_callback(self, msg):                     # 定义运动状态回调函数
        """
        获取基本状态
        """
        # 更新角速度和方向 
        self.vthx = float(msg.imu_state.gyroscope[0])                           # 从消息中获取x轴角速度
        self.vthy = float(msg.imu_state.gyroscope[1])                           # 从消息中获取y轴角速度
        self.vthz = float(msg.imu_state.gyroscope[2])                           # 从消息中获取z轴角速度


        self.quat_w = float(msg.imu_state.quaternion[0])           # 四元数w
        self.quat_x = float(msg.imu_state.quaternion[1])           # 四元数x
        self.quat_y = float(msg.imu_state.quaternion[2])           # 四元数y
        self.quat_z = float(msg.imu_state.quaternion[3])           # 四元数z
    
        self.th  = float(msg.imu_state.rpy[2])                     # 更新yaw度数

        # 获取并更新线速度
        self.vx = float(msg.velocity[0])   # 获取x轴线速度
        self.vy = float(msg.velocity[1])   # 获取y轴线速度
        self.vz = float(msg.velocity[2])   # 获取z轴线速度

        # 获取并更新位置
        self.x = float(msg.position[0])    # 获取x轴位置
        self.y = float(msg.position[1])    # 获取y轴位置
        self.z = float(msg.position[2])    # 获取z轴位置


        """
        时间戳同步
        """
        # 使用 sshpass 获取远程计算机的系统时间
        remote_time = subprocess.check_output(
            ["sshpass", "-p", password, "ssh", f"{user}@{remote_ip}", "date +%s%N"]
        ).strip()
        
        # 将时间戳转换为整数
        timestamp = int(remote_time)

        # 分离秒和纳秒
        sec = timestamp // 1000000000  # 秒部分
        nanosec = timestamp % 1000000000  # 纳秒部分
        self.get_logger().info(f'{RED}系统时间: sec={sec}, nano={nanosec}{RESET}')


        """
        里程计话题信息生成
        """
        self.odom = Odometry()                               # 创建一个Odometry消息对象
        self.odom.header.stamp.sec = sec                     # 设置消息的时间戳
        self.odom.header.stamp.nanosec = nanosec             # 设置消息的时间戳

        self.odom.header.frame_id = 'odom_slamtoolbox'                   # 设置消息的坐标系ID
        self.odom.child_frame_id = 'base'               # 设置子坐标系ID

        self.odom.pose.pose.position.x = self.x              # 设置位置x
        self.odom.pose.pose.position.y = self.y              # 设置位置y
        self.odom.pose.pose.position.z = self.z              # 设置位置z
        self.odom.pose.pose.orientation.x = self.quat_x      # 设置姿态四元数
        self.odom.pose.pose.orientation.y = self.quat_y
        self.odom.pose.pose.orientation.z = self.quat_z
        self.odom.pose.pose.orientation.w = self.quat_w

        self.odom.twist.twist.linear.x = self.vx                     # 设置线速度x
        self.odom.twist.twist.linear.y = self.vy                     # 设置线速度y
        self.odom.twist.twist.linear.z = self.vz                     # 设置线速度z
        self.odom.twist.twist.angular.x = self.vthx                  # 设置角速度x
        self.odom.twist.twist.angular.y = self.vthy                  # 设置角速度y
        self.odom.twist.twist.angular.z = self.vthz                  # 设置角速度z


        """
        静态tf生成
        """
        # 使用雷达的时间戳
        self.static_transform_stamped.header.stamp.sec = sec
        self.static_transform_stamped.header.stamp.nanosec = nanosec

        self.static_transform_stamped.header.frame_id = 'base'  # 目标frame_id
        self.static_transform_stamped.child_frame_id = 'ridar'  # 原始frame_id rslidar


        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.0
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0
        


        """
        动态tf关系生成
        """
        # 设置变换的时间戳
        self.transform.header.stamp.sec = sec
        self.transform.header.stamp.nanosec = nanosec

        # 设置坐标系
        self.transform.header.frame_id = 'odom_slamtoolbox'                          # 设置一个坐标变换的源坐标系
        self.transform.child_frame_id = 'base'                                       # 设置一个坐标变换的目标坐标系

        # 设置转化参数
        self.transform.transform.translation.x = self.x
        self.transform.transform.translation.y = self.y
        self.transform.transform.translation.z = self.z
        self.transform.transform.rotation.x = self.quat_x
        self.transform.transform.rotation.y = self.quat_y
        self.transform.transform.rotation.z = self.quat_z
        self.transform.transform.rotation.w = self.quat_w
        

    # 回调函数，处理接收到的odom消息，并发布tf
    def publish_all(self):
        
        # 发布里程计消息
        self.odom_pub.publish(self.odom) 
        self.get_logger().info(f'{BLUE}里程计时间戳: {self.transform.header.stamp}{RESET}')                           

        # 发布静态变换
        self.static_broadcaster.sendTransform(self.static_transform_stamped)
        self.get_logger().info(f'{RED}静态TF时间戳: {self.transform.header.stamp}{RESET}')


        # 广播坐标变换信息
        self.tf_broadcaster.sendTransform(self.transform)
        self.get_logger().info(f'{YELLOW}动态TF时间戳: {self.transform.header.stamp}{RESET}')


        

def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_publisher = DynamicTFPublisher()
    rclpy.spin(dynamic_tf_publisher)
    dynamic_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()