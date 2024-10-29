import rclpy  # 导入rclpy库，用于ROS 2节点编程
from rclpy.node import Node  # 从rclpy中导入Node类
from sensor_msgs.msg import Image  # 导入Image消息类型，用于处理图像数据
from std_msgs.msg import String  # 导入String消息类型，用于发布识别结果
from cv_bridge import CvBridge  # 导入CvBridge，用于OpenCV与ROS图像消息之间的转换
import cv2  # 导入OpenCV库，用于图像处理
import torch  # 导入PyTorch库，用于深度学习模型
import torchvision.transforms as transforms  # 导入torchvision.transforms，用于图像预处理
from torchvision import models  # 导入torchvision.models，用于加载预训练模型

class ImageRecognitionNode(Node):  # 定义图像识别节点类，继承自Node
    def __init__(self):
        super().__init__('image_recognition_node')  # 初始化节点，命名为'image_recognition_node'
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10)  # 创建订阅者，订阅'camera/image'主题
        self.publisher_ = self.create_publisher(String, 'image/recognition', 10)  # 创建发布者，发布'image/recognition'主题
        self.bridge = CvBridge()  # 创建CvBridge实例，用于图像转换

        # 加载预训练的模型并进行迁移学习
        self.model = models.resnet18(pretrained=True)  # 加载预训练的ResNet18模型
        num_ftrs = self.model.fc.in_features  # 获取全连接层的输入特征数
        self.model.fc = torch.nn.Linear(num_ftrs, 2)  # 修改全连接层，假设有两个类别
        self.model.load_state_dict(torch.load('path_to_finetuned_model.pth'))  # 加载微调后的模型权重
        self.model.eval()  # 设置模型为评估模式

        # 定义图像预处理
        self.preprocess = transforms.Compose([
            transforms.ToPILImage(),  # 转换为PIL图像
            transforms.Resize(256),  # 调整图像大小为256x256
            transforms.CenterCrop(224),  # 中心裁剪为224x224
            transforms.ToTensor(),  # 转换为张量
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # 归一化
        ])

    def image_callback(self, msg):  # 图像回调函数
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # 将ROS图像消息转换为OpenCV图像
        input_tensor = self.preprocess(cv_image)  # 对图像进行预处理
        input_batch = input_tensor.unsqueeze(0)  # 添加批次维度

        with torch.no_grad():  # 禁用梯度计算
            output = self.model(input_batch)  # 进行前向传播，获取输出
            _, predicted = torch.max(output, 1)  # 获取预测的类别
            result = f'Predicted class: {predicted.item()}'  # 格式化预测结果
            self.publisher_.publish(String(data=result))  # 发布预测结果
            self.get_logger().info(result)  # 记录日志信息

def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化rclpy
    image_recognition_node = ImageRecognitionNode()  # 创建图像识别节点实例
    rclpy.spin(image_recognition_node)  # 运行节点
    image_recognition_node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

if __name__ == '__main__':  # 如果是主程序
    main()  # 调用主函数