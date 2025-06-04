#include "pcl_main.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/pcl_base.hpp>

namespace pcl_main // PCL主命名空间
{
    PclStudy::PclStudy() // PCL学习类构造函数
        : Node("pcl_study") // 初始化ROS2节点名称为pcl_study
    {
        // 初始化体素滤波参数
        voxel_size_x_ = 0.1f; // 设置X轴体素大小为0.05
        voxel_size_y_ = 0.1f; // 设置Y轴体素大小为0.05
        voxel_size_z_ = 0.1f; // 设置Z轴体素大小为0.05

        pub_res = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_result", 50); // 创建点云结果发布器

        auto callback = [this](const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) -> void // 定义回调函数处理接收到的点云数据
        {
            pcl::PointCloud<hesai_ros::PointXYZIRT>::Ptr cloud(new pcl::PointCloud<hesai_ros::PointXYZIRT>); // 创建原始点云指针
            pcl::fromROSMsg(*cloud_msg, *cloud); // 将ROS消息转换为PCL点云格式

            pcl::PointCloud<hesai_ros::PointXYZIRT>::Ptr cloud_filtered(new pcl::PointCloud<hesai_ros::PointXYZIRT>); // 创建滤波后点云指针

            // 体素滤波
            pcl::VoxelGrid<hesai_ros::PointXYZIRT> sor; // 模板类实例化，创建体素滤波对象
            sor.setInputCloud(cloud); // 设置输入点云
            sor.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_); // 设置体素大小
            
            // 设置对所有字段进行下采样，这样可以保留时间戳信息
            sor.setDownsampleAllData(true);  // 对所有字段进行下采样
            
            sor.filter(*cloud_filtered); // 执行滤波操作

            sensor_msgs::msg::PointCloud2 ros2_msg; // 创建ROS2点云消息
            pcl::toROSMsg(*cloud_filtered, ros2_msg); // 将PCL点云转换回ROS2消息格式
            std::cout << "before filter: " << cloud_msg->width << std::endl; // 打印滤波前点云宽度
            std::cout << "after filter: " << ros2_msg.width << std::endl; // 打印滤波后点云宽度
            // 保持原始点云的header信息
            ros2_msg.header = cloud_msg->header; // 复制原始消息的头部信息

            publish_result(ros2_msg); // 发布处理后的点云消息
        };

        sub_origin = create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", rclcpp::SensorDataQoS(), callback); // 创建点云订阅器
    }

    PclStudy::~PclStudy() {} // PCL学习类析构函数

    void PclStudy::publish_result(sensor_msgs::msg::PointCloud2 res_msg) // 发布结果的成员函数
    {
        // res_msg.header.frame_id = "id"; // 设置消息帧ID
        // res_msg.header.stamp = now(); // 设置时间戳，注释掉以保留原始时间戳
        pub_res->publish(res_msg); // 发布点云处理结果
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pcl_main::PclStudy>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

