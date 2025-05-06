#include "unitree/common/dds/dds_easy_model.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/ros2_idl/QtCommand_.hpp>
#include <unitree/ros2_idl/String_.hpp>
#include <unitree/ros2_idl/Odometry_.hpp>
#include <unitree/ros2_idl/QtNode_.hpp>
#include <unitree/ros2_idl/QtEdge_.hpp>

#include <stdio.h>
#include <termio.h>
#include <cmath>
#include <ctime>

// 定义主题名称
#define COMMANDTOPIC "rt/qt_command"           // Qt命令主题
#define NOTICETOPIC "rt/qt_notice"            // Qt通知主题
#define ODOMTOPIC "rt/lio_sam_ros2/mapping/re_location_odometry"  // 里程计主题
#define ADDNODETOPIC "rt/qt_add_node"         // 添加节点主题
#define ADDEDGETOPIC "rt/qt_add_edge"         // 添加边主题

using namespace unitree::robot;
using namespace std;

// 节点属性结构体
struct nodeAttribute
{
    u_int16_t nodeName;  // 节点名称
    float nodeX;         // 节点X坐标
    float nodeY;         // 节点Y坐标
    float nodeZ;         // 节点Z坐标
    float nodeYaw;       // 节点偏航角
};

// 边属性结构体
struct edgeAttribute
{
    u_int16_t edgeName;    // 边名称
    u_int16_t edgeStart;   // 起始节点
    u_int16_t edgeEnd;     // 终止节点
};

// SLAM演示类
class slamDemo
{
private:
    int index = 0;  // 命令索引
    const nav_msgs::msg::dds_::Odometry_ *currentOdom;  // 当前里程计数据
    u_int16_t node_name = 0;  // 节点名称计数器

    vector<nodeAttribute> nodeAttributeList;    // 节点属性列表
    vector<edgeAttribute> edgeAttributeList;    // 边属性列表

    // 发布者
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_> pubQtCommand = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtCommand_>(COMMANDTOPIC));
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_> pubQtNode = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtNode_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtNode_>(ADDNODETOPIC));
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_> pubQtEdge = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtEdge_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtEdge_>(ADDEDGETOPIC));
    // 订阅者
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subQtNotice = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(NOTICETOPIC));
    ChannelSubscriberPtr<nav_msgs::msg::dds_::Odometry_> subOdommetry = ChannelSubscriberPtr<nav_msgs::msg::dds_::Odometry_>(new ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>(ODOMTOPIC));

public:
    slamDemo(const char *networkInterface);  // 构造函数

    void qtNoticeHandler(const void *message);    // Qt通知处理函数
    void odometryHandler(const void *message);    // 里程计数据处理函数

    void keyExecute();                            // 按键执行函数
    unsigned char keyDetection();                 // 按键检测函数

    void startMapping();                          // 开始建图
    void endMapping();                            // 结束建图
    void startRelocation();                       // 开始重定位
    void initPose();                             // 初始化位姿
    void startNavigation();                       // 开始导航
    void defaultNavigation();                     // 默认导航
    void addNodeAndEdge();                        // 添加节点和边
    void addEdge(u_int16_t edge_name, u_int16_t start_node, u_int16_t end_node);  // 添加边
    void saveNodeAndEdge();                       // 保存节点和边
    void closeAllNode();                          // 关闭所有节点
    void deleteAllNode();                         // 删除所有节点
    void deleteAllEdge();                         // 删除所有边
    void pauseNavigation();                       // 暂停导航
    void recoverNavigation();                     // 恢复导航
};

// 构造函数实现
slamDemo::slamDemo(const char *networkInterface)
{
    ChannelFactory::Instance()->Init(0, networkInterface); // 初始化网络接口

    // 初始化发布者
    pubQtCommand->InitChannel();
    pubQtNode->InitChannel();
    pubQtEdge->InitChannel();

    // 初始化订阅者
    subQtNotice->InitChannel(std::bind(&slamDemo::qtNoticeHandler, this, std::placeholders::_1), 10);
    subOdommetry->InitChannel(std::bind(&slamDemo::odometryHandler, this, std::placeholders::_1), 1);

    // 打印使用说明
    cout << "***********************  Unitree SLAM Demo ***********************\n";
    cout << "------------------         q    w    e         -------------------\n";
    cout << "------------------         a    s    d         -------------------\n";
    cout << "------------------         z    x    c    v    -------------------\n";
    cout << "------------------------------------------------------------------\n";
    cout << "------------------ q: 关闭ROS节点              -------------------\n";
    cout << "------------------ w: 开始建图                 -------------------\n";
    cout << "------------------ e: 结束建图                 -------------------\n";
    cout << "------------------ a: 开始导航                 -------------------\n";
    cout << "------------------ s: 暂停导航                 -------------------\n";
    cout << "------------------ d: 恢复导航                 -------------------\n";
    cout << "------------------ z: 重定位和初始化位姿       -------------------\n";
    cout << "------------------ x: 添加节点和边             -------------------\n";
    cout << "------------------ c: 保存节点和边             -------------------\n";
    cout << "------------------ v: 删除所有节点和边         -------------------\n";
    cout << "--------------- 按Ctrl + C退出程序             ---------------\n";
    cout << "------------------------------------------------------------------" << endl;
}

// 里程计数据处理函数
void slamDemo::odometryHandler(const void *message)
{
    currentOdom = (const nav_msgs::msg::dds_::Odometry_ *)message;
}

// Qt通知处理函数
void slamDemo::qtNoticeHandler(const void *message)
{
    int index_, begin_, end_, feedback_, arrive_;
    const std_msgs::msg::dds_::String_ *seq = (const std_msgs::msg::dds_::String_ *)message;
    string str_, notice_;

    begin_ = seq->data().find("index:", 0);  // 指令唯一标识符
    end_ = seq->data().find(";", begin_);
    str_ = seq->data().substr(begin_ + 6, end_ - begin_ - 6);
    index_ = atoi(str_.c_str());

    begin_ = seq->data().find("notice:", 0);  // 提示信息
    end_ = seq->data().find(";", begin_);
    notice_ = seq->data().substr(begin_ + 7, end_ - begin_ - 7);

    if (index_ <= 10000)
    { // 命令执行反馈
        begin_ = seq->data().find("feedback:", 0);
        end_ = seq->data().find(";", begin_);
        str_ = seq->data().substr(begin_ + 9, end_ - begin_ - 9);
        feedback_ = atoi(str_.c_str());
        if (feedback_ == 0 || feedback_ == -1)
            cout << "\033[1;31m"
                 << "命令执行失败，索引 = " << index_ << "."
                 << "\033[0m";
        cout << notice_ << endl;
    }
    else if (index_ == 10001)
    { // 导航反馈
        begin_ = seq->data().find("arrive:", 0);
        end_ = seq->data().find(";", begin_);
        str_ = seq->data().substr(begin_ + 7, end_ - begin_ - 7);
        arrive_ = atoi(str_.c_str());
        cout << " 已到达节点 " << arrive_ << ". " << notice_ << endl;
    }
}

// 开始建图
void slamDemo::startMapping()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 3;  // 3是开始建图命令
    send_msg.attribute_() = 1;  // 当此值为1时启用XT16激光雷达节点，为2时启用MID360激光雷达节点
    pubQtCommand->Write(send_msg);
}

// 结束建图
void slamDemo::endMapping()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.command_() = 4;  // 4是结束建图命令
    send_msg.attribute_() = 0;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.floor_index_().push_back(0);   // 楼层号，固定值0
    send_msg.pcdmap_index_().push_back(0);  // PCD地图号，固定值0
    pubQtCommand->Write(send_msg);
}

// 开始重定位
void slamDemo::startRelocation()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 6;  // 6是开始重定位命令
    send_msg.attribute_() = 1;  // 当此值为1时启用XT16激光雷达节点，为2时启用MID360激光雷达节点
    pubQtCommand->Write(send_msg);
}

// 初始化位姿
void slamDemo::initPose()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 7;       // 7是位姿初始化指令
    send_msg.quaternion_x_() = 0;  // 四元数
    send_msg.quaternion_y_() = 0;
    send_msg.quaternion_z_() = 0;
    send_msg.quaternion_w_() = 1;
    send_msg.translation_x_() = 0;  // 平移
    send_msg.translation_y_() = 0;
    send_msg.translation_z_() = 0;
    pubQtCommand->Write(send_msg);
}

// 开始导航
void slamDemo::startNavigation()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 8;  // 8是开始导航命令
    pubQtCommand->Write(send_msg);
}

// 默认导航
void slamDemo::defaultNavigation()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 10;  // 10是多节点循环导航(默认)命令
    pubQtCommand->Write(send_msg);
}

// 关闭所有节点
void slamDemo::closeAllNode()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 99;  // 99是关闭所有节点命令
    pubQtCommand->Write(send_msg);
}

// 添加节点和边
void slamDemo::addNodeAndEdge()
{
    nodeAttribute nodeTemp;
    float siny_cosp, cosy_cosp;

    // 计算偏航角
    siny_cosp = 2 * (currentOdom->pose().pose().orientation().w() * currentOdom->pose().pose().orientation().z() + currentOdom->pose().pose().orientation().x() * currentOdom->pose().pose().orientation().y());
    cosy_cosp = 1 - 2 * (currentOdom->pose().pose().orientation().y() * currentOdom->pose().pose().orientation().y() + currentOdom->pose().pose().orientation().z() * currentOdom->pose().pose().orientation().z());
    node_name++;

    // 设置节点属性
    nodeTemp.nodeX = currentOdom->pose().pose().position().x();
    nodeTemp.nodeY = currentOdom->pose().pose().position().y();
    nodeTemp.nodeZ = currentOdom->pose().pose().position().z();;  # 增加z轴属性
    nodeTemp.nodeYaw = std::atan2(siny_cosp, cosy_cosp);
    nodeTemp.nodeName = node_name;

    nodeAttributeList.push_back(nodeTemp);

    cout << "添加节点.   名称: " << node_name << "  X:" << nodeTemp.nodeX << "  Y:" << nodeTemp.nodeY << "  Z:" << nodeTemp.nodeZ << "  偏航角:" << nodeTemp.nodeYaw << endl;
    if (node_name >= 2)
        addEdge(node_name - 1, node_name - 1, node_name);  // 顺序连接节点
}

// 添加边
void slamDemo::addEdge(u_int16_t edge_name, u_int16_t start_node, u_int16_t end_node)
{
    edgeAttribute edgeTemp;

    edgeTemp.edgeName = edge_name;
    edgeTemp.edgeStart = start_node;
    edgeTemp.edgeEnd = end_node;
    edgeAttributeList.push_back(edgeTemp);

    cout << "添加边.   名称: " << edge_name << "  起始节点:" << start_node << "  终止节点:" << end_node << endl;
    cout << "--------------------------------------------------------------------------" << endl;
}

// 保存节点和边
void slamDemo::saveNodeAndEdge()
{
    unitree_interfaces::msg::dds_::QtNode_ nodeMsg;
    unitree_interfaces::msg::dds_::QtEdge_ edgeMsg;
    if (edgeAttributeList.size() == 0)
    {
        cout << "\033[1;31m"
             << "拓扑图中边的数量为0."
             << "\033[0m" << endl;
        return;
    }

    // 保存节点信息
    index++;
    nodeMsg.seq_().data() = "index:" + to_string(index) + ";";
    for (int i = 0; i < nodeAttributeList.size(); i++)
    {
        nodeMsg.node_().node_name_().push_back(nodeAttributeList[i].nodeName);     // 节点名称
        nodeMsg.node_().node_position_x_().push_back(nodeAttributeList[i].nodeX);  // 点X坐标信息
        nodeMsg.node_().node_position_y_().push_back(nodeAttributeList[i].nodeY);  // 点Y坐标信息
        nodeMsg.node_().node_position_z_().push_back(nodeAttributeList[i].nodeZ);  // 点Z坐标信息
        nodeMsg.node_().node_yaw_().push_back(nodeAttributeList[i].nodeYaw);       // 点偏航角信息
        nodeMsg.node_().node_attribute_().push_back(0);                            // 未开放属性，请赋值0
        nodeMsg.node_().undefined_().push_back(0);
        nodeMsg.node_().node_state_2_().push_back(0);
        nodeMsg.node_().node_state_3_().push_back(0);
    }
    pubQtNode->Write(nodeMsg);

    // 保存边信息
    index++;
    edgeMsg.seq_().data() = "index:" + to_string(index) + ";";
    for (int i = 0; i < edgeAttributeList.size(); i++)
    {
        edgeMsg.edge_().edge_name_().push_back(edgeAttributeList[i].edgeName);         // 边名称
        edgeMsg.edge_().start_node_name_().push_back(edgeAttributeList[i].edgeStart);  // 起始节点名称
        edgeMsg.edge_().end_node_name_().push_back(edgeAttributeList[i].edgeEnd);      // 终止节点名称
        edgeMsg.edge_().edge_length_().push_back(0);
        edgeMsg.edge_().dog_speed_().push_back(0.5);     // 机器狗走这条边的速度(0-1)
        edgeMsg.edge_().edge_state_2_().push_back(1);    // 遇到障碍物时：0停止，1避障，3重规划。建议赋值0，修改为1

        edgeMsg.edge_().dog_stats_().push_back(0);       // 未开放属性，请赋值0
        edgeMsg.edge_().dog_back_stats_().push_back(0);
        edgeMsg.edge_().edge_state_().push_back(0);
        edgeMsg.edge_().edge_state_1_().push_back(0);
        edgeMsg.edge_().edge_state_3_().push_back(0);
        edgeMsg.edge_().edge_state_4_().push_back(0);
    }
    pubQtEdge->Write(edgeMsg);
}

// 删除所有节点
void slamDemo::deleteAllNode()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 1;    // 1是删除指令
    send_msg.attribute_() = 1;  // 1是选择删除节点
    send_msg.node_edge_name_().push_back(999);

    pubQtCommand->Write(send_msg);
}

// 删除所有边
void slamDemo::deleteAllEdge()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 1;    // 1是删除指令
    send_msg.attribute_() = 2;  // 2是选择删除边
    send_msg.node_edge_name_().push_back(999);
    pubQtCommand->Write(send_msg);
}

// 暂停导航
void slamDemo::pauseNavigation()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 13;  // 13是暂停导航命令
    pubQtCommand->Write(send_msg);
}

// 恢复导航
void slamDemo::recoverNavigation()
{
    index++;
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;
    send_msg.seq_().data() = "index:" + to_string(index) + ";";
    send_msg.command_() = 14;  // 14是恢复导航命令
    pubQtCommand->Write(send_msg);
}

// 按键检测
unsigned char slamDemo::keyDetection()
{
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    cout << "\033[1;32m"
         << "按键 " << ch << " 已按下."
         << "\033[0m" << endl;
    return ch;
}

// 执行按键命令
void slamDemo::keyExecute()
{
    unsigned char currentKey;
    while (true)
    {
        currentKey = keyDetection();
        switch (currentKey)
        {
        case 'q':  // 关闭所有节点
            closeAllNode();
            break;
        case 'w':  // 开始建图(默认清除节点/边信息)
            deleteAllNode();
            deleteAllEdge();
            nodeAttributeList.clear();
            edgeAttributeList.clear();
            startMapping();
            node_name = 0;
            break;
        case 'e':  // 结束建图
            endMapping();
            break;
        case 'a':  // 开始导航(默认)
            startRelocation();
            startNavigation();
            initPose();
            defaultNavigation();
            break;
        case 's':  // 暂停导航
            pauseNavigation();
            break;
        case 'd':  // 恢复导航
            recoverNavigation();
            break;
        case 'z':  // 开始重定位和导航，为采集节点/边信息做准备(默认清除节点/边信息)
            deleteAllNode();
            deleteAllEdge();
            startRelocation();
            startNavigation();
            initPose();
            break;
        case 'x':  // 采集节点/边信息
            addNodeAndEdge();
            break;
        case 'c':  // 保存节点/边信息
            saveNodeAndEdge();
            nodeAttributeList.clear();
            edgeAttributeList.clear();
            node_name = 0;
            break;
        case 'v':  // 清除节点/边信息
            deleteAllNode();
            deleteAllEdge();
            node_name = 0;
            break;
        default:
            break;
        }
    }
}

int main(int argc, const char **argv)
{
    slamDemo slamTest(argv[1]);  // argv[1]：网段为123的网卡名称
    slamTest.keyExecute();
    return 0;
}
