#ifndef PCL_MAIN_H
#define PCL_MAIN_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

// 定义 Hesai PointXYZIRT 类型
namespace hesai_ros {
struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros

// 注册点云类型
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

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

        // 添加点云处理参数
        float voxel_size_x_;
        float voxel_size_y_;
        float voxel_size_z_;
    };
}

#endif // PCL_MAIN_H