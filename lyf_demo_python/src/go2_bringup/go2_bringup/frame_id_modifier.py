import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

class FrameIdModifier(Node):
    def __init__(self):
        super().__init__('frame_id_modifier')
        # 将frame_id进行重映射
        self.declare_parameter('new_frame_id', 'ridar')
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

        # 设置 QoS 策略
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 或者使用 ReliabilityPolicy.BEST_EFFORT
            depth=10
        )

        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 或者使用 ReliabilityPolicy.BEST_EFFORT
            depth=10
        )



        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_old',
            self.listener_callback,
            qos_profile_sub)
        
        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_pub)

        # 用于存储接收到的消息
        self.latest_msg = None

        # 将定时器间隔设置为0.2秒（即10Hz）
        self.timer = self.create_timer(0.2, self.publish_new_topic)

    def listener_callback(self, msg):
        self.latest_msg = msg
        self.latest_msg.header.frame_id = self.new_frame_id

        # # 调整时间戳，延迟 0.1 秒
        # adjusted_time = Time(seconds=self.latest_msg.header.stamp.sec, nanoseconds=self.latest_msg.header.stamp.nanosec) + rclpy.duration.Duration(seconds=0.5)
        # self.latest_msg.header.stamp = adjusted_time.to_msg()

    def publish_new_topic(self):
         if self.latest_msg is not None:
            self.publisher.publish(self.latest_msg)
            # self.get_logger().info(f'Publishing modified message with frame_id: "{self.latest_msg.header.frame_id}"')

def main(args=None):
    rclpy.init(args=args)
    node = FrameIdModifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()