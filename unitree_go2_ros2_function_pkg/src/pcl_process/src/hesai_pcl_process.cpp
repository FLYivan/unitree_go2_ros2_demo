#include "pcl_main.h"
#include "rclcpp_components/register_node_macro.hpp"
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/statistical_outlier_removal.h>

namespace pcl_main
{
    PclStudy::PclStudy(rclcpp::NodeOptions options)
        : Node("pcl_study",
               options.automatically_declare_parameters_from_overrides(true))
    {
        pub_res = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_result", 10);

        auto callback = [this](const PointCloud2::UniquePtr cloud_msg) -> void
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

            // 直通滤波
            /*
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(1.0,3.0);
            // pass.setFilterLimitsNegative (true);   // 对设置的范围取反，本来是对设置的范围内的点云可以通过的，现在取反就是设置的范围内的点云不能通过
            pass.filter(*cloud_filtered);
            */

            // 体素滤波
           
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.05f, 0.05f, 0.05f);
            sor.setDownsampleAllData(true);
            sor.filter(*cloud_filtered);
          

            // 移除离群点：当搜索半径内的点的个数少于设置的阈值时将会被当作离群点移除
            /*
            // 尝试了几个参数，都没有滤波效果
            pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(0.8);   // 设置搜索半径
            outrem.setMinNeighborsInRadius(2);  // 设置搜索半径内的最少邻近点个数
            outrem.setKeepOrganized(true);
            // apply filter
            outrem.filter(*cloud_filtered);
            */

            /**
             * @brief 统计滤波
             *  对每一点的邻域进行统计分析，基于点到所有邻近点的距离分布特征，过滤掉一些不满足要求的离群点。
             * 该算法对整个输入进行 两次迭代：
             * 在第一次迭代中，
             * 对于点云中的每一个点，找到该点的K近邻；
             * 计算每个点与它K近邻点的平局距离，结果满足高斯分布；
             * 计算这些距离的均值μ和标准差σ；
             * 设置距离阈值 dthreshold = μ ± k⋅σ，k 为标准差乘数；
             * 在下一次迭代中，如果点的平均邻域距离在区间[μ−k⋅σ ,μ +k⋅σ]，则该点为内点，予以保留，区间外的点为噪声点，予以删除。
             */

            // 创建滤波对象
            // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            // sor.setInputCloud(cloud);
            // sor.setMeanK(50);               // k为50
            // sor.setStddevMulThresh(1.0);   // 标准差
            // sor.filter(*cloud_filtered);

            // sor.setNegative(true);      // 噪声点
            // sor.filter(*cloud_filtered);

            // 条件滤波
            // pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
            // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 2.0)));
            // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, 2.8)));
            // // build the filter
            // pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
            // condrem.setCondition(range_cond);
            // condrem.setInputCloud(cloud);
            // // 对于散乱点云，不需要执行此语句；若输入点云为有组织的点云，此语句可保持点云的原始组织结构，
            // // 不会改变行列数，点数也不会减少，被过滤掉的点用 NaN 填充。
            // // condrem.setKeepOrganized(true);
            // condrem.filter(*cloud_filtered);
            // // 去除 NaN 点（只针对有组织的点云。散乱点云不需要）
            // // vector<int> Idx;
            // // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, Idx);

            // 点云提取
            /**
             * @brief
             * pcl::ExtractIndices是基于某一个分割算法提取点云的子集.
             * 思路： ExtractIndices通过分割算法提取部分点云数据子集的下标索引
             * 代码步骤：
             * （1） 使用之前的体素栅格下采样方法进行下采样；
             * （2） SAC平面参数模型提取符合该几何模型的点云数据子集，再利用分割算法进行提取符合几何平面的点云数据子集；
             * （3） 利用negative变量可以提取相反的点云集剩余点云；
             * （4） 利用剩余点云作为待处理的点云，返回步骤2，进入下一轮循环，直到满足终止条件。
             */
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZI>), cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);

            // // pcl::PointCloud<pcl::PointXYZI>转为pcl::PCLPointCloud2
            // pcl::toPCLPointCloud2(*cloud, *cloud_blob);
            // // 创建滤波器对象进行体素滤波
            // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            // sor.setInputCloud(cloud_blob);
            // sor.setLeafSize(0.1f, 0.1f, 0.1f);
            // sor.filter(*cloud_filtered_blob);

            // // 转换为模板点云
            // pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
            // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

            // //创建分割时所需要的模型系数对象coefficients
            // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            // //创建存储内点的点索引对象inliers
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            // // 创建分割对象
            // pcl::SACSegmentation<pcl::PointXYZI> seg;
            // // 设置对估计的模型做优化处理（可选）
            // seg.setOptimizeCoefficients(true);
            // // Mandatory（强制性的，必选的）
            // seg.setModelType(pcl::SACMODEL_PLANE); //设置分割模型类别
            // seg.setMethodType(pcl::SAC_RANSAC);    //设置随机参数估计方法
            // seg.setMaxIterations(10);
            // seg.setDistanceThreshold(0.1);

            // // 创建滤波对象
            // pcl::ExtractIndices<pcl::PointXYZI> extract;

            // int i = 0, nr_points = (int)cloud_filtered->size();
            // // 循环条件是剩余30%的原始点云
            // while (cloud_filtered->size() > 0.3 * nr_points)
            // {
            //     // 从余下的点云中分割最大平面组成部分
            //     seg.setInputCloud(cloud_filtered);
            //     seg.segment(*inliers, *coefficients);
            //     if (inliers->indices.size() == 0)
            //     {
            //         std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            //         break;
            //     }

            //     // 提取内层
            //     extract.setInputCloud(cloud_filtered);
            //     extract.setIndices(inliers); // 设置分割后的内点为需要提取的点集
            //     extract.setNegative(false);  // 设置提取内点（不取反，即满足最大平面的条件）
            //     extract.filter(*cloud_p);
            //     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

            //     // 创建滤波对象
            //     extract.setNegative(true); // 设置提取外点（取反，即剩余的点云）
            //     extract.filter(*cloud_f);
            //     cloud_filtered.swap(cloud_f);

            //     sensor_msgs::msg::PointCloud2 ros2_msg;
            //     pcl::toROSMsg(*cloud_p, ros2_msg);
            //     publish_result(ros2_msg);
            //     i++;
            // }

            // size_t len = cloud_msg->width;
            // for (size_t i = 0; i < len; i++)
            // {
            //     double x = cloud->points[i].x;
            //     double y = cloud->points[i].y;
            //     double z = cloud->points[i].z;
            //     std::cout << "points: " << x << ", " << y << ", " << z << std::endl;
            // }

            sensor_msgs::msg::PointCloud2 ros2_msg;
            pcl::toROSMsg(*cloud_filtered, ros2_msg);
            std::cout << "before filter: " << cloud_msg->width << std::endl;
            std::cout << "after filter: " << ros2_msg.width << std::endl;
            publish_result(ros2_msg);
        };

        sub_origin = create_subscription<PointCloud2>("/topic", rclcpp::SensorDataQoS(), callback);
    }
    PclStudy::~PclStudy() {}

    // 处理结果发布
    void PclStudy::publish_result(sensor_msgs::msg::PointCloud2 res_msg)
    {
        res_msg.header.frame_id = "id";
        res_msg.header.stamp = now();
        pub_res->publish(res_msg);
    }
}
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_main::PclStudy)