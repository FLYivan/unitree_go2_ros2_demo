#include <unitree/robot/channel/channel_publisher.hpp> // 包含ChannelPublisher头文件
#include <unitree/ros2_idl/QtNode_.hpp> // 包含QtNode_头文件

#define TOPIC "rt/qt_add_node" // 定义发布的主题名称
using namespace unitree::robot; // 使用unitree::robot命名空间

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; // 打印使用方法
        exit(-1); // 如果参数不足，退出程序
    }

    ChannelFactory::Instance()->Init(0, argv[1]); // 初始化通道工厂，argv[1]：网络接口名，例如eth0
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_> publisher = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtNode_>(TOPIC)); // 创建一个发布者，发布QtNode_类型的消息到TOPIC主题
    publisher->InitChannel(); // 初始化发布者通道
    unitree_interfaces::msg::dds_::QtNode_ send_msg; // 定义要发送的消息

    send_msg.seq_().data() = "index:123;";              // 设置序列号，用户指定的值（1-10000）
    send_msg.node_().node_name_().push_back(1);         // 设置节点名称
    send_msg.node_().node_position_x_().push_back(1.0); // 设置节点X坐标信息
    send_msg.node_().node_position_y_().push_back(1.0); // 设置节点Y坐标信息
    send_msg.node_().node_position_z_().push_back(0);   // 设置节点Z坐标信息
    send_msg.node_().node_yaw_().push_back(1.57);       // 设置节点偏航角信息

    send_msg.node_().node_attribute_().push_back(0); // 设置节点属性，不打开属性，必须赋值为0，注意：不能为空！！！不能为空！！！不能为空！！！
    send_msg.node_().undefined_().push_back(0);
    send_msg.node_().node_state_2_().push_back(0);
    send_msg.node_().node_state_3_().push_back(0);

    publisher->Write(send_msg); // 发布消息
    std::cout << "send add node command" << std::endl; // 打印发送添加节点命令的信息
    sleep(2); // 休眠2秒

    return 0;
}
