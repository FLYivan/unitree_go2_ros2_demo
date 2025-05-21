// src/calibrate_imu.cpp
#include "calibrate_imu/calibrate_imu_common.h"

namespace calibrate_imu {

class StandardIMUCalibrator : public IMUCalibrator {
public:
    StandardIMUCalibrator() 
        : IMUCalibrator("calibrate_imu")
    {
        // 创建发布者
        pub_speed_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

        // 创建订阅者
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 300, 
            std::bind(&StandardIMUCalibrator::imu_handler, this, std::placeholders::_1));

        // 初始化定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&StandardIMUCalibrator::timer_callback, this));

        // 初始化开始时间
        beginning_ = std::chrono::system_clock::now();
    }

    void imu_handler(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
        if (get_state() == 1) {
            imu_static_.push_back(*msg_in);
        }
        else if (get_state() == 2) {
            imu_rotation_positive_z_.push_back(*msg_in);
        }
    }

    void timer_callback() {
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning_).count();

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "vehicle";
        const double ang_vel = 1.396;

        if (seconds < 2) {  // 前2秒：调整初始位置
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);
            
            set_state(0);
            RCLCPP_INFO(this->get_logger(), "Adjusting the robot to the initial position...");
        }
        else if (seconds >= 2 && seconds < 15) {  // 2-15秒：静止状态收集数据
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);

            if (seconds >= 5) {
                set_state(1);
                RCLCPP_INFO(this->get_logger(), "Collecting static data...");
            }
        }
        else if (seconds >= 15 && seconds < 35) {  // 15-35秒：执行Z轴正向旋转
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = ang_vel;
            pub_speed_->publish(cmd_vel);

            set_state(2);
            RCLCPP_INFO(this->get_logger(), "Collecting positive z-axis rotation data...");
        }
        else if (seconds >= 35 && seconds < 37) {  // 35-37秒：停止运动并保存数据
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);

            set_state(3);
            if (!file_written_) {
                RCLCPP_INFO(this->get_logger(), "Writing to file...");
                estimate_bias();
                serialize_to_file();
                file_written_ = true;
                RCLCPP_INFO(this->get_logger(), "Calibration finished!");
                rclcpp::shutdown();
            }
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_speed_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::system_clock::time_point beginning_;
    bool file_written_{false};
};

} // namespace calibrate_imu

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<calibrate_imu::StandardIMUCalibrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}