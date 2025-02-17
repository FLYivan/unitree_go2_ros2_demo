import rclpy  # 导入ROS2 Python客户端库
from rclpy.node import Node  # 导入Node类
from sensor_msgs.msg import Image  # 导入Image消息类型
from cv_bridge import CvBridge  # 导入OpenCV与ROS图像转换工具
import cv2  # 导入OpenCV库

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream')  # 初始化节点
        self.image_publisher_ = self.create_publisher(Image, "/camera/image/raw", 10)

        multicast_iface = "wlp0s20f3"
        self.declare_parameter("multicast_iface", multicast_iface)
        multicast_iface = self.get_parameter("multicast_iface").get_parameter_value().string_value
  
        gstreamer_str = "udpsrc address=230.1.1.1 port=1720 multicast-iface=wlp0s20f3 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
        """
        注意！要确保opencv版本支持gstreamer,该开关打开，通过“print(cv2.getBuildInformation())”查看
        """       

        self.cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)


        # self.get_logger().info("cap is {self.cap}")

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video capture")
        else:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if ret:
                    ros_image = CvBridge().cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.frame_id = "camera"
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    self.image_publisher_.publish(ros_image)

                    self.get_logger().info("published image frame")
                cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    node = VideoStreamNode()  # 创建节点实例
    try:
        rclpy.spin(node)  # 运行节点
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()  # 释放视频捕获对象
        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口
        rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':
    main()