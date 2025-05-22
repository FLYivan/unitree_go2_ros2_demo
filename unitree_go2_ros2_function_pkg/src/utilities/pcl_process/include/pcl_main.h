#ifndef PCL_MAIN_H
#define PCL_MAIN_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_main
{
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    
    class PclStudy : public rclcpp::Node
    {
    public:
        PclStudy();
        ~PclStudy();

    private:
        void publish_result(sensor_msgs::msg::PointCloud2 res_msg);
        
        rclcpp::Publisher<PointCloud2>::SharedPtr pub_res;
        rclcpp::Subscription<PointCloud2>::SharedPtr sub_origin;
    };
}

#endif // PCL_MAIN_H