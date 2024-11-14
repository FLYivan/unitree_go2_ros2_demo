import rclpy                                            # 导入rclpy库
import tf_transformations                               # 导入tf_transformations库
import tf2_ros                                          # 导入tf2_ros库

from rclpy.node import Node                             # 从rclpy.node模块导入Node类
from sensor_msgs.msg import Imu                         # 从sensor_msgs.msg模块导入Imu消息类型
from nav_msgs.msg import Odometry                       # 从nav_msgs.msg模块导入Odometry消息类型
from geometry_msgs.msg import Quaternion, TransformStamped  # 从geometry_msgs.msg模块导入Quaternion和TransformStamped消息类型
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

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

class ImuToOdomNode(Node):                                  # 定义ImuToOdomNode类，继承自Node类
    def __init__(self):         
        super().__init__('motion_to_odom_node')             # 调用父类的初始化方法，并设置节点名称
      
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)                             # 创建一个发布器，用于发布Odometry消息

        # 创建一个订阅器，用于订阅消息
        self.motion_sub = self.create_subscription(SportModeState, topic_name, self.motion_callback, 10)   
    
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)                                # 创建一个TransformBroadcaster对象

        self.last_time = self.get_clock().now()         # 获取当前时间
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

        self.time = self.get_clock().now().to_msg()

        # 创建定时器以发布里程计
        self.publish_frequency = 200.0                   # 设置发布频率为10Hz
        self.publish_period = 1.0 / self.publish_frequency                                      # 计算发布周期
        self.timer = self.create_timer(self.publish_period, self.publish_odom)                  # 创建定时器以发布里程计


    def motion_callback(self, msg):                     # 定义运动状态回调函数
        # current_time = self.get_clock().now()           # 获取当前时间

        # 获取当前时间
        self.time = TimeMsg(sec=msg.stamp.sec, nanosec=msg.stamp.nanosec)

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



    def publish_odom(self):                             # 定义发布里程计的函数
        current_time = self.get_clock().now()           # 获取当前时间

        odom = Odometry()                               # 创建一个Odometry消息对象
        odom.header.stamp = self.time                   # 设置消息的时间戳
        odom.header.frame_id = 'odom'                   # 设置消息的坐标系ID
        odom.child_frame_id = 'base_link'               # 设置子坐标系ID

        odom.pose.pose.position.x = self.x              # 设置位置x
        odom.pose.pose.position.y = self.y              # 设置位置y
        odom.pose.pose.position.z = self.z              # 设置位置z
        odom.pose.pose.orientation.x = self.quat_x      # 设置姿态四元数
        odom.pose.pose.orientation.y = self.quat_y
        odom.pose.pose.orientation.z = self.quat_z
        odom.pose.pose.orientation.w = self.quat_w

        odom.twist.twist.linear.x = self.vx                     # 设置线速度x
        odom.twist.twist.linear.y = self.vy                     # 设置线速度y
        odom.twist.twist.linear.z = self.vz                     # 设置线速度z
        odom.twist.twist.angular.x = self.vthx                  # 设置角速度x
        odom.twist.twist.angular.y = self.vthy                  # 设置角速度y
        odom.twist.twist.angular.z = self.vthz                  # 设置角速度z

        self.odom_pub.publish(odom)                             # 发布里程计消息

        # odom_trans = TransformStamped()                         # 创建一个TransformStamped消息对象
        # odom_trans.header.stamp = current_time.to_msg()         # 设置变换的时间戳
        # odom_trans.header.frame_id = 'odom'                     # 设置变换的坐标系ID
        # odom_trans.child_frame_id = 'base_link'        odom_pub        # 设置子坐标系ID
        # odom_trans.transform.translation.x = self.x             # 设置变换的平移x
        # odom_trans.transform.translation.y = self.y             # 设置变换的平移y
        # odom_trans.transform.translation.z = self.z             # 设置变换的平移z
        # odom_trans.transform.rotation = Quaternion(*odom_quat)  # 设置变换的旋转四元数

        # self.tf_broadcaster.sendTransform(odom_trans)           # 发送变换

def main(args=None):  
    rclpy.init(args=args)                                       # 初始化rclpy
    node = ImuToOdomNode()                                      # 创建ImuToOdomNode节点
    rclpy.spin(node)                                            # 运行节点
    node.destroy_node()                                         # 销毁节点
    rclpy.shutdown()                                            # 关闭rclpy

if __name__ == '__main__':                                      # 如果是主程序
    main()                                                      # 调用主函数


