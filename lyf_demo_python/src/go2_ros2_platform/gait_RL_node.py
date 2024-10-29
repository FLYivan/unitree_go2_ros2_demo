import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np

class QuadrupedGaitController(Node):
    def __init__(self):
        super().__init__('quadruped_gait_controller')
        
        # 加载训练好的步态模型
        self.model = torch.load('path_to_trained_model.pth')
        self.model.eval()  # 设置模型为评估模式

        # 订阅传感器数据
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # 发布步态控制命令
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10)

        # 初始化传感器数据
        self.imu_data = None
        self.joint_states = None

    def imu_callback(self, msg):
        # 处理IMU数据
        self.imu_data = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def joint_state_callback(self, msg):
        # 处理关节状态数据
        self.joint_states = np.array(msg.position)

    def compute_gait(self):
        if self.imu_data is not None and self.joint_states is not None:
            # 将传感器数据转换为模型输入
            input_data = np.concatenate((self.imu_data, self.joint_states))
            input_tensor = torch.tensor(input_data, dtype=torch.float32).unsqueeze(0)

            # 使用模型计算步态动作
            with torch.no_grad():
                action = self.model(input_tensor).squeeze().numpy()

            # 发布步态控制命令
            command_msg = Float64MultiArray(data=action)
            self.command_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    gait_controller = QuadrupedGaitController()
    rclpy.spin(gait_controller)
    gait_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()