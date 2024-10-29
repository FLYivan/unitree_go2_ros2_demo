import rclpy  # 导入rclpy库
from rclpy.node import Node  # 从rclpy中导入Node类
from sensor_msgs.msg import LaserScan, Image, Imu  # 导入传感器消息类型
from cv_bridge import CvBridge  # 导入cv_bridge用于图像转换
import cv2  # 导入OpenCV库

class PerceptionNode(Node):  # 定义感知节点类，继承自Node
    def __init__(self):
        super().__init__('perception_node')  # 初始化节点，命名为'perception_node'
        
        # 初始化每个传感器的发布者
        self.lidar_publisher = self.create_publisher(LaserScan, 'scan', 10)  # 创建激光雷达数据发布者
        self.camera_publisher = self.create_publisher(Image, 'camera/image', 10)  # 创建相机图像发布者
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)  # 创建IMU数据发布者
        
        # 初始化每个传感器的定时器
        self.lidar_timer = self.create_timer(0.1, self.publish_lidar_data)  # 设置激光雷达数据发布定时器
        self.camera_timer = self.create_timer(0.1, self.publish_camera_data)  # 设置相机数据发布定时器
        self.imu_timer = self.create_timer(0.1, self.publish_imu_data)  # 设置IMU数据发布定时器
        
        # 初始化CV Bridge用于图像转换
        self.bridge = CvBridge()  # 创建CV Bridge实例
        
        # 预留用于添加其他传感器的占位符
        self.additional_sensors = []  # 初始化额外传感器列表

    def publish_lidar_data(self):  # 定义发布激光雷达数据的方法
        scan = LaserScan()  # 创建LaserScan消息实例
        # 填充scan数据
        self.lidar_publisher.publish(scan)  # 发布激光雷达数据
        self.get_logger().info('Published LIDAR data')  # 记录日志信息

    def publish_camera_data(self):  # 定义发布相机数据的方法
        ret, frame = cv2.VideoCapture(0).read()  # 从摄像头读取图像
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')  # 转换图像格式
            self.camera_publisher.publish(image_message)  # 发布相机图像
            self.get_logger().info('Published camera image')  # 记录日志信息

    def publish_imu_data(self):  # 定义发布IMU数据的方法
        imu_data = Imu()  # 创建Imu消息实例
        # 填充imu_data
        self.imu_publisher.publish(imu_data)  # 发布IMU数据
        self.get_logger().info('Published IMU data')  # 记录日志信息

    def add_sensor(self, sensor_callback, topic_name, msg_type, timer_period=0.1):  # 定义添加额外传感器的方法
        """
        动态添加额外传感器的方法。
        :param sensor_callback: 用于发布传感器数据的回调函数
        :param topic_name: 传感器的ROS主题名称
        :param msg_type: 传感器的ROS消息类型
        :param timer_period: 发布数据的定时器周期
        """
        publisher = self.create_publisher(msg_type, topic_name, 10)  # 创建发布者
        timer = self.create_timer(timer_period, lambda: sensor_callback(publisher))  # 创建定时器
        self.additional_sensors.append((publisher, timer))  # 将发布者和定时器添加到列表
        self.get_logger().info(f'Added new sensor on topic {topic_name}')  # 记录日志信息

def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化rclpy
    perception_node = PerceptionNode()  # 创建感知节点实例
    
    # 示例：添加一个新传感器
    def dummy_sensor_callback(publisher):  # 定义虚拟传感器的回调函数
        # 创建并发布虚拟数据
        msg = Imu()  # 替换为适当的消息类型
        publisher.publish(msg)  # 发布消息
    
    perception_node.add_sensor(dummy_sensor_callback, 'dummy/sensor', Imu, 0.5)  # 添加虚拟传感器
    
    rclpy.spin(perception_node)  # 运行节点
    perception_node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

if __name__ == '__main__':  # 如果是主程序
    main()  # 调用主函数