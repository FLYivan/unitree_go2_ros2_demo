#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/ros2_idl/QtEdge_.hpp>

#define TOPIC "rt/qt_add_edge" // 定义发布的主题名称
using namespace unitree::robot;

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; // 打印使用方法
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]); // 初始化通道工厂，argv[1]：网络接口的名称，带有123的网络段
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_> publisher = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtEdge_>(TOPIC)); // 创建发布者
    publisher->InitChannel(); // 初始化通道
    unitree_interfaces::msg::dds_::QtEdge_ send_msg; // 定义要发送的消息

    // 构建连接拓扑节点0和拓扑节点1的边。添加边之前，确保起始和结束节点存在
    send_msg.seq_().data() = "index:123;";            // 用户设置的值（1-10000）
    send_msg.edge_().edge_name_().push_back(1);       // 边的名称
    send_msg.edge_().start_node_name_().push_back(0); // 起始节点的名称
    send_msg.edge_().end_node_name_().push_back(1);   // 结束节点的名称
    send_msg.edge_().dog_speed_().push_back(1);       // 狗在这条边上行走的速度（0-1），建议赋值为1
    send_msg.edge_().edge_state_2_().push_back(0);    // 遇到障碍时的状态，0：停止 1：避免 3：重规划。建议赋值为0。

    send_msg.edge_().dog_stats_().push_back(0); // 未开放的属性，请赋值为0，注意：不能为空!!! 不能为空!!! 不能为空!!!
    send_msg.edge_().edge_length_().push_back(0);
    send_msg.edge_().dog_back_stats_().push_back(0);
    send_msg.edge_().edge_state_().push_back(0);
    send_msg.edge_().edge_state_1_().push_back(0);
    send_msg.edge_().edge_state_3_().push_back(0);
    send_msg.edge_().edge_state_4_().push_back(0);

    publisher->Write(send_msg); // 发布消息
    std::cout << "send add edge command" << std::endl; // 打印发送添加边的命令
    sleep(2); // 休眠2秒

    return 0;
}
