import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

ODOMTOPIC = 'utlidar/robot_odom'


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
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_link'  # 目标frame_id
        static_transform_stamped.child_frame_id = 'utlidar_lidar'  # 原始frame_id
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform_stamped)

        # 创建 odom 话题订阅者，使用传感器数据的 Qos
        self.odom_subscribe_ = self.create_subscription(
            Odometry,
            ODOMTOPIC,
            self.odom_callback_,
            qos_profile
        )
        # 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
        self.tf_broadcaster = TransformBroadcaster(self)

    # 回调函数，处理接收到的odom消息，并发布tf
    def odom_callback_(self, msg):
        transform = TransformStamped()                                # 创建一个坐标变换的消息对象

        transform.header.stamp = msg.header.stamp     # 设置坐标变换消息的时间戳

        transform.header.frame_id = msg.header.frame_id                           # 设置一个坐标变换的源坐标系
        transform.child_frame_id = msg.child_frame_id                    # 设置一个坐标变换的目标坐标系
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w


        # 广播坐标变换信息
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTopic2TF('odom2tf')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()