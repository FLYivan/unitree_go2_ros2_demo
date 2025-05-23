#include "pcl_main.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/pcl_base.hpp>

namespace pcl_main
{
    PclStudy::PclStudy()
        : Node("pcl_study")
    {
        // 初始化体素滤波参数
        voxel_size_x_ = 0.05f;
        voxel_size_y_ = 0.05f;
        voxel_size_z_ = 0.05f;

        pub_res = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_result", 10);

        auto callback = [this](const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) -> void
        {
            pcl::PointCloud<hesai_ros::PointXYZIRT>::Ptr cloud(new pcl::PointCloud<hesai_ros::PointXYZIRT>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            pcl::PointCloud<hesai_ros::PointXYZIRT>::Ptr cloud_filtered(new pcl::PointCloud<hesai_ros::PointXYZIRT>);

            // 体素滤波
            pcl::VoxelGrid<hesai_ros::PointXYZIRT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
            sor.filter(*cloud_filtered);

            sensor_msgs::msg::PointCloud2 ros2_msg;
            pcl::toROSMsg(*cloud_filtered, ros2_msg);
            std::cout << "before filter: " << cloud_msg->width << std::endl;
            std::cout << "after filter: " << ros2_msg.width << std::endl;
            // 保持原始点云的header信息
            ros2_msg.header = cloud_msg->header;

            publish_result(ros2_msg);
        };

        sub_origin = create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_points", rclcpp::SensorDataQoS(), callback);
    }

    PclStudy::~PclStudy() {}

    void PclStudy::publish_result(sensor_msgs::msg::PointCloud2 res_msg)
    {
        // res_msg.header.frame_id = "id";
        // res_msg.header.stamp = now();
        pub_res->publish(res_msg);
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

