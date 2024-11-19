import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.publish_initial_pose()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        
        # 设置 header
        msg.header.stamp.sec = 33644786  # 设置秒
        msg.header.stamp.nanosec = 69688244  # 设置纳秒
        msg.header.frame_id = 'map_slamtoolbox'  # 设置 frame_id

        # 设置位姿
        msg.pose.pose.position.x = 0.7671076655387878
        msg.pose.pose.position.y = -1.2045036554336548
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.7071080860634099
        msg.pose.pose.orientation.w = 0.7071054763072772

        # 设置协方差
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 
            0.06853891909122467  # 确保这是一个浮点数
        ]

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing initial pose:{msg.header.stamp}')

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



