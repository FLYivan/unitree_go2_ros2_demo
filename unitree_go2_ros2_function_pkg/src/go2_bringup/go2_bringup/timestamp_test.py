import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformListener

# ANSI 转义序列，定义打印颜色
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'  # 重置颜色


class YourNode(Node):
    def __init__(self):
        super().__init__('timestamp_test')


        # 自定义QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,     # 可靠性策略： 确保消息被传递，如果失败会尝试重传
            history=QoSHistoryPolicy.KEEP_LAST,            # 历史策略： 只保留最新的N条消息
            depth=10                                       # N=10
        )


        # 订阅雷达数据
        # self.create_subscription(
        #     PointCloud2,
        #     'rslidar_points',  # 替换为实际的雷达话题
        #     self.lidar_callback,
        #     qos_profile
        # )

        self.create_subscription(
            LaserScan,
            'scan_old',  # 替换为实际的雷达话题
            self.lidar_callback,
            qos_profile
        )


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建一个定时器，定期检查变换
        self.create_timer(1.0, self.check_transform)


    def lidar_callback(self, msg):
        # 获取雷达数据的时间戳
        radar_timestamp = msg.header.stamp
        
        # 获取当前系统时间
        current_time = self.get_clock().now().to_msg()

        # 打印雷达时间戳和系统时间
        self.get_logger().info(f'{YELLOW}Radar Timestamp: {radar_timestamp.sec}.{radar_timestamp.nanosec}{RESET}')
        self.get_logger().info(f'System Time: {current_time.sec}.{current_time.nanosec}')

    def check_transform(self):
        try:
            # 获取从 odom 到 base 的变换
            transform = self.tf_buffer.lookup_transform('base', 'odom_slamtoolbox', rclpy.time.Time())
            # 打印变换信息和时间戳
            self.get_logger().info(f'Transform from odom to base: {transform}')
            self.get_logger().info(f'{RED}Transform Timestamp: {transform.header.stamp.sec}.{transform.header.stamp.nanosec}{RESET}')
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()