#include <unitree/robot/channel/channel_publisher.hpp> // 包含通道发布器头文件
#include <unitree/ros2_idl/QtCommand_.hpp> // 包含QtCommand消息头文件

#define TOPIC "rt/qt_command" // 定义发布的主题名称
using namespace unitree::robot; // 使用unitree::robot命名空间

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; // 打印使用方法
        exit(-1); // 如果参数不足2个，则退出程序
    }
    ChannelFactory::Instance()->Init(0, argv[1]); // 初始化通道工厂，argv[1]为网络接口的名称，例如网络段为123的网卡
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_> publisher = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtCommand_>(TOPIC)); // 创建QtCommand消息的发布器
    publisher->InitChannel(); // 初始化通道
    unitree_interfaces::msg::dds_::QtCommand_ send_msg; // 定义要发送的QtCommand消息

    // 查询所有点信息
    send_msg.seq_().data() = "index:123;";     // 用户设置的值123（1-10000）。
    send_msg.command_() = 2;                   // 2是查询指令
    send_msg.attribute_() = 1;                 // 1是选择查询节点
    send_msg.floor_index_().push_back(999);    // 楼层区分，请给定固定值999
    send_msg.node_edge_name_().push_back(999); // 查询节点的名称列表。当node_edge_name[0]=999时，将查询所有边信息。

    publisher->Write(send_msg); // 发布消息
    std::cout << "send query node command" << std::endl; // 打印发送查询节点命令的信息
    sleep(2); // 休眠2秒

    return 0;
}
