import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

ODOMTOPIC = 'utlidar/robot_odom'                                      # 自带odom
# ODOMTOPIC = 'lio_sam_ros2/dogOdomProcess/DogOdomGlobal'               # 宇数slam接口中基于禾赛雷达的odom

RIDARTOPIC = 'scan'

class OdomTopic2TF(Node):
    def __init__(self, name):
        super().__init__(name)

        # 自定义QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,     # 可靠性策略： 确保消息被传递，如果失败会尝试重传
            history=QoSHistoryPolicy.KEEP_LAST,            # 历史策略： 只保留最新的N条消息
            depth=10                                       # N=10
        )

        # 创建静态变换广播器
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_transform_stamped = TransformStamped()

        # 使用雷达的时间戳
        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = 'base'  # 目标frame_id
        self.static_transform_stamped.child_frame_id = 'ridar'  # 原始frame_id rslidar
        self.static_transform_stamped.transform.translation.x = 0.171
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.0908
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0
        
        # 发布静态变换
        self.static_broadcaster.sendTransform(self.static_transform_stamped)
        

        # 创建 odom 话题订阅者，使用传感器数据的 Qos
        self.odom_subscribe_ = self.create_subscription(
            Odometry,
            ODOMTOPIC,
            self.odom_callback,
            qos_profile
        )

        # 创建一个TransformBroadcaster用于广播动态坐标变换
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()  # 初始化 transform

        # 将定时器间隔设置为0.05秒（即20Hz）
        self.dt = 0.05 
        self.timer = self.create_timer(self.dt , self.publish_transform)        # 动态tf发布的频率为20hz(不可动参数)
        


    # 回调函数，处理接收到的odom消息，并发布tf
    def odom_callback(self, msg):

        self.transform.header.stamp = msg.header.stamp     # 设置坐标变换消息的时间戳

        # self.transform.header.frame_id = msg.header.frame_id                           # 设置一个坐标变换的源坐标系
        self.transform.header.frame_id = 'odom_slamtoolbox'                          # 设置一个坐标变换的源坐标系
        # self.transform.child_frame_id = msg.child_frame_id                    # 设置一个坐标变换的目标坐标系
        self.transform.child_frame_id = 'base'                   # 设置一个坐标变换的目标坐标系
        self.transform.transform.translation.x = msg.pose.pose.position.x
        self.transform.transform.translation.y = msg.pose.pose.position.y
        self.transform.transform.translation.z = msg.pose.pose.position.z
        self.transform.transform.rotation.x = msg.pose.pose.orientation.x
        self.transform.transform.rotation.y = msg.pose.pose.orientation.y
        self.transform.transform.rotation.z = msg.pose.pose.orientation.z
        self.transform.transform.rotation.w = msg.pose.pose.orientation.w

    def publish_transform(self):
        # 广播坐标变换信息
        self.tf_broadcaster.sendTransform(self.transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTopic2TF('odom2tf')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()