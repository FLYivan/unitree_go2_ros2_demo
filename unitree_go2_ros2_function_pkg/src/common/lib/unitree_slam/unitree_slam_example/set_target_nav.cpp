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

    // 导航到节点名为1的位置
    send_msg.seq_().data() = "index:123;";   // 用户设置的值123（1-10000）。
    send_msg.command_() = 9;                 // 9是单个节点导航命令
    send_msg.attribute_() = 1;
    send_msg.translation_x_() = 1.0;	      //X坐标
    send_msg.translation_y_() = 0.1;	      //Y坐标
    send_msg.translation_z_() = 0.2;         //Z坐标
    send_msg.euler_yaw_() = 0.3;             //偏航角
        
    send_msg.node_edge_name_().push_back(1); // 目标节点名

    publisher->Write(send_msg); // 发布消息
    std::cout << "send single nav command" << std::endl; // 打印发送单个导航命令
    sleep(2); // 休眠2秒

    return 0;
}
