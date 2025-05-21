// include/calibrate_imu/calibrate_imu_common.h
#pragma once

#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace calibrate_imu {

class IMUCalibrator : public rclcpp::Node {
public:
    // 构造函数
    IMUCalibrator(const std::string& node_name) 
        : Node(node_name)
        , state_(-1)
        , acc_bias_x_(0)
        , acc_bias_y_(0)
        , acc_bias_z_(0)
        , ang_bias_x_(0)
        , ang_bias_y_(0)
        , ang_bias_z_(0)
        , ang_z2x_proj_(0)
        , ang_z2y_proj_(0)
    {
    }

    // 虚析构函数
    virtual ~IMUCalibrator() {}

protected:
    // 成员变量
    int state_;                                      // 状态变量
    std::vector<sensor_msgs::msg::Imu> imu_static_; // 存储静止状态下的IMU数据
    std::vector<sensor_msgs::msg::Imu> imu_rotation_positive_z_; // 存储正向Z轴旋转时的IMU数据

    double acc_bias_x_;                             // 加速度计X轴偏差
    double acc_bias_y_;                             // 加速度计Y轴偏差
    double acc_bias_z_;                             // 加速度计Z轴偏差

    double ang_bias_x_;                             // 陀螺仪X轴偏差
    double ang_bias_y_;                             // 陀螺仪Y轴偏差
    double ang_bias_z_;                             // 陀螺仪Z轴偏差

    double ang_z2x_proj_;                           // Z轴到X轴的投影系数
    double ang_z2y_proj_;                           // Z轴到Y轴的投影系数

    // 公共方法
    void serialize_to_file() {
        const char* homeDir = getenv("HOME");
        std::string file_path = std::string(homeDir) + "/桌面/imu_calib_data.yaml";
        std::ofstream file;
        file.open(file_path, std::ios::out);
        file << "acc_bias_x: " << acc_bias_x_ << std::endl;
        file << "acc_bias_y: " << acc_bias_y_ << std::endl;
        file << "acc_bias_z: " << acc_bias_z_ << std::endl;
        file << "ang_bias_x: " << ang_bias_x_ << std::endl;
        file << "ang_bias_y: " << ang_bias_y_ << std::endl;
        file << "ang_bias_z: " << ang_bias_z_ << std::endl;
        file << "ang_z2x_proj: " << ang_z2x_proj_ << std::endl;
        file << "ang_z2y_proj: " << ang_z2y_proj_ << std::endl;
        file.close();
    }

    void estimate_bias() {
        int static_count = 0;
        double ang_rot_x_mean_positive = 0;
        double ang_rot_y_mean_positive = 0;
        double ang_rot_z_mean_positive = 0;

        RCLCPP_INFO(this->get_logger(), "static size: %ld", imu_static_.size());
        RCLCPP_INFO(this->get_logger(), "pos size: %ld", imu_rotation_positive_z_.size());

        // 处理静止状态数据
        for (const auto& imu_data : imu_static_) {
            static_count += 1;

            acc_bias_x_ += imu_data.linear_acceleration.x;
            acc_bias_y_ += imu_data.linear_acceleration.y;
            acc_bias_z_ += imu_data.linear_acceleration.z;

            ang_bias_x_ += imu_data.angular_velocity.x;
            ang_bias_y_ += imu_data.angular_velocity.y;
            ang_bias_z_ += imu_data.angular_velocity.z;
        }

        // 计算平均值
        if (!imu_static_.empty()) {
            acc_bias_x_ /= imu_static_.size();
            acc_bias_y_ /= imu_static_.size();
            acc_bias_z_ /= imu_static_.size();
            ang_bias_x_ /= imu_static_.size();
            ang_bias_y_ /= imu_static_.size();
            ang_bias_z_ /= imu_static_.size();
        }

        acc_bias_z_ -= 9.81;  // 减去重力加速度

        // 处理旋转数据
        for (const auto& imu_data : imu_rotation_positive_z_) {
            ang_rot_x_mean_positive += imu_data.angular_velocity.x;
            ang_rot_y_mean_positive += imu_data.angular_velocity.y;
            ang_rot_z_mean_positive += imu_data.angular_velocity.z;
        }

        if (!imu_rotation_positive_z_.empty()) {
            ang_rot_x_mean_positive = ang_rot_x_mean_positive / imu_rotation_positive_z_.size() - ang_bias_x_;
            ang_rot_y_mean_positive = ang_rot_y_mean_positive / imu_rotation_positive_z_.size() - ang_bias_y_;
            ang_rot_z_mean_positive = ang_rot_z_mean_positive / imu_rotation_positive_z_.size() - ang_bias_z_;

            ang_z2x_proj_ = -ang_rot_x_mean_positive / ang_rot_z_mean_positive;
            ang_z2y_proj_ = -ang_rot_y_mean_positive / ang_rot_z_mean_positive;
        }

        // 输出结果
        RCLCPP_INFO(this->get_logger(), "acc_bias_x: %f", acc_bias_x_);
        RCLCPP_INFO(this->get_logger(), "acc_bias_y: %f", acc_bias_y_);
        RCLCPP_INFO(this->get_logger(), "acc_bias_z: %f", acc_bias_z_);
        RCLCPP_INFO(this->get_logger(), "ang_bias_x: %f", ang_bias_x_);
        RCLCPP_INFO(this->get_logger(), "ang_bias_y: %f", ang_bias_y_);
        RCLCPP_INFO(this->get_logger(), "ang_bias_z: %f", ang_bias_z_);
        RCLCPP_INFO(this->get_logger(), "ang_z2x_proj: %f", ang_z2x_proj_);
        RCLCPP_INFO(this->get_logger(), "ang_z2y_proj: %f", ang_z2y_proj_);
    }

    // 获取状态
    int get_state() const { return state_; }
    
    // 设置状态
    void set_state(int state) { state_ = state; }
};

} // namespace calibrate_imu