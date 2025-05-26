#ifndef PCL_MAIN_H  // PCL主头文件宏定义开始  （宏=文本替换）
#define PCL_MAIN_H  // PCL主头文件宏定义 （这个宏的作用就是防止头文件被重复包含）

#include <rclcpp/rclcpp.hpp>                    // 包含ROS2基础头文件
#include <sensor_msgs/msg/point_cloud2.hpp>  // 包含点云消息类型头文件
#include <pcl/point_types.h>                // 包含PCL点类型头文件
#include <pcl/point_cloud.h>                    // 包含PCL点云类型头文件
#include <pcl/impl/point_types.hpp>             // 包含PCL点类型实现头文件
#include <pcl/filters/impl/voxel_grid.hpp>          // 包含体素滤波器实现头文件

// 定义 Hesai PointXYZIRT 类型
namespace hesai_ros {                              // Hesai激光雷达命名空间开始
struct EIGEN_ALIGN16 PointXYZIRT {              // 定义16字节对齐的点类型结构体
                                                 // 在访问内存时，如果数据是按照特定字节对齐的（比如16字节），可以一次性读取更多数据，效率更高
    PCL_ADD_POINT4D;                        // 添加XYZ坐标和填充字段
    float intensity;                            // 点云强度
    uint16_t ring;                           // 激光线束编号
    double timestamp;                           // 时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW          // 用于保证用 new 关键字动态分配对象时，内存同样是按16字节对齐的
};
}  // namespace hesai_ros              

// 注册点云类型（POINT_CLOUD_REGISTER_POINT_STRUCT 是PCL（点云库）提供的一个宏，用于注册自定义点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::PointXYZIRT,  // 注册自定义点云结构
    (float, x, x)  // X坐标字段
    (float, y, y)  // Y坐标字段
    (float, z, z)  // Z坐标字段
    (float, intensity, intensity)  // 强度字段
    (uint16_t, ring, ring)  // 线束编号字段
    (double, timestamp, timestamp)  // 时间戳字段
)

namespace pcl_main  // PCL主处理命名空间开始
{
    using PointCloud2 = sensor_msgs::msg::PointCloud2;  // 定义点云消息类型别名
    
    class PclStudy : public rclcpp::Node  // 定义PCL学习类，继承自ROS2节点
    {
    public:
        PclStudy();  // 构造函数声明
        ~PclStudy();  // 析构函数声明

    private:
        void publish_result(sensor_msgs::msg::PointCloud2 res_msg);  // 发布结果的成员函数声明
        
        rclcpp::Publisher<PointCloud2>::SharedPtr pub_res;  // 点云发布器智能指针
        rclcpp::Subscription<PointCloud2>::SharedPtr sub_origin;  // 点云订阅器智能指针

        // 添加点云处理参数
        float voxel_size_x_;  // 体素滤波X轴分辨率
        float voxel_size_y_;  // 体素滤波Y轴分辨率
        float voxel_size_z_;  // 体素滤波Z轴分辨率
    };
}  // 命名空间结束

#endif // PCL_MAIN_H  // PCL主头文件宏定义结束