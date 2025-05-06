#include <unitree/robot/channel/channel_publisher.hpp> // 导入通道发布器头文件
#include <unitree/ros2_idl/QtCommand_.hpp> // 导入QtCommand_头文件

#define TOPIC "rt/qt_command" // 定义发布的主题
using namespace unitree::robot; // 使用unitree::robot命名空间

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; // 打印使用方法
        exit(-1);
    }
    ChannelFactory::Instance()->Init(0, argv[1]); // 初始化通道工厂，argv[1]：网络卡的名称，带有123的网络段
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_> publisher = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtCommand_>(TOPIC)); // 创建QtCommand_类型的通道发布器
    publisher->InitChannel(); // 初始化通道
    unitree_interfaces::msg::dds_::QtCommand_ send_msg; // 定义要发送的消息

    // 设置消息内容
    send_msg.seq_().data() = "index:123;"; // 用户设置的值123（1-10000）。
    send_msg.command_() = 7;               // 7是姿态初始化指令

    // 设置四元数
    send_msg.quaternion_x_() = 0; 
    send_msg.quaternion_y_() = 0;
    send_msg.quaternion_z_() = 0;
    send_msg.quaternion_w_() = 1;
    // 设置平移
    send_msg.translation_x_() = 0; 
    send_msg.translation_y_() = 0;
    send_msg.translation_z_() = 0;

    publisher->Write(send_msg); // 发布消息
    std::cout << "send pose init command" << std::endl; // 打印发送姿态初始化命令
    sleep(2); // 休眠2秒

    return 0;
}
