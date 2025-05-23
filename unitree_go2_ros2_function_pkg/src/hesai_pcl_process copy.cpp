
#include "pcl_main.h"
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_main
{
    
    PclStudy::PclStudy()
        : Node("pcl_study")
    {
        pub_res = create_publisher<PointCloud2>("cloud_result", 10);

      
        auto callback = [this](const std::shared_ptr<PointCloud2> cloud_msg) -> void
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

            // 体素滤波
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.05f, 0.05f, 0.05f);
            sor.setDownsampleAllData(true);
            sor.filter(*cloud_filtered);

            sensor_msgs::msg::PointCloud2 ros2_msg;
            pcl::toROSMsg(*cloud_filtered, ros2_msg);
            std::cout << "before filter: " << cloud_msg->width << std::endl;
            std::cout << "after filter: " << ros2_msg.width << std::endl;
            publish_result(ros2_msg);
        };

        sub_origin = create_subscription<PointCloud2>("/lidar_points", rclcpp::SensorDataQoS(), callback);
    }

    PclStudy::~PclStudy() {}

    void PclStudy::publish_result(sensor_msgs::msg::PointCloud2 res_msg)
    {
        res_msg.header.frame_id = "id";
        res_msg.header.stamp = now();
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

