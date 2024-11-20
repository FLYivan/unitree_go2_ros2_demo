import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        # 用于存储接收到的消息
        self.latest_msg = None

        # 将frame_id进行重映射
        self.declare_parameter('new_frame_id', 'map_slamtoolbox')
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

        self.sub =self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 将定时器间隔设置为0.2秒（即5Hz）
        self.timer = self.create_timer(0.2, self.publish_initial_pose)             # /scan话题的发布频率保证为5hz(不可动参数)


    def listener_callback(self,msg):

        self.latest_msg = msg
        self.latest_msg.header.frame_id = self.new_frame_id
     

    def publish_initial_pose(self):
        if self.latest_msg is not None:
            self.get_logger().info(f'Publishing initial pose:{self.latest_msg.pose.pose}')
            self.publisher.publish(self.latest_msg)
            
            self.get_logger().info('任务已执行，正在关闭节点。')
            rclpy.shutdown()  # 执行任务后关闭节点


def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



