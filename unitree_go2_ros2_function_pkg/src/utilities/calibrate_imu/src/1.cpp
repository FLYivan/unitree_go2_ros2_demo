// src/calibrate_go2_imu.cpp
#include "calibrate_imu/calibrate_imu_common.h"

namespace calibrate_imu {

class Go2IMUCalibrator : public IMUCalibrator {
public:
    Go2IMUCalibrator() 
        : IMUCalibrator("calibrate_go2_imu")
    {
        // 创建发布者
        pub_go2_request_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        pub_speed_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

        // 创建订阅者
        sub_imu_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 300, 
            std::bind(&Go2IMUCalibrator::imu_handler, this, std::placeholders::_1));

        // 初始化定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Go2IMUCalibrator::timer_callback, this));

        // 初始化开始时间
        beginning_ = std::chrono::system_clock::now();
    }

    void imu_handler(const unitree_go::msg::SportModeState::SharedPtr msg_in) {
        double theta = 15.1 / 180 * 3.1415926;  // 将角度转换为弧度，调整为Z轴向上

        double x = msg_in->imu_state.gyroscope[0];
        double y = msg_in->imu_state.gyroscope[1];
        double z = msg_in->imu_state.gyroscope[2];

        double acc_x = msg_in->imu_state.accelerometer[0];
        double acc_y = msg_in->imu_state.accelerometer[1];
        double acc_z = msg_in->imu_state.accelerometer[2];

        sensor_msgs::msg::Imu msg_store;
        msg_store.header.frame_id = "body";
        msg_store.header.stamp = this->now();
        msg_store.orientation.x = msg_in->imu_state.quaternion[1];
        msg_store.orientation.y = msg_in->imu_state.quaternion[2];
        msg_store.orientation.z = msg_in->imu_state.quaternion[3];
        msg_store.orientation.w = msg_in->imu_state.quaternion[0];

        msg_store.angular_velocity.x = x;
        msg_store.angular_velocity.y = y;
        msg_store.angular_velocity.z = z;
        msg_store.linear_acceleration.x = acc_x;
        msg_store.linear_acceleration.y = acc_y;
        msg_store.linear_acceleration.z = acc_z;

        if (get_state() == 1) {
            imu_static_.push_back(msg_store);
        }
        else if (get_state() == 2) {
            imu_rotation_positive_z_.push_back(msg_store);
        }
    }

    void timer_callback() {
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning_).count();

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "vehicle";
        unitree_api::msg::Request req;
        SportClient sport_req;

        const double ang_vel = 1.396;

        if (seconds < 2) {  // 前2秒：调整初始位置
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);

            sport_req.Move(req, 0, 0, 0);
            pub_go2_request_->publish(req);
            
            set_state(0);
            RCLCPP_INFO(this->get_logger(), "Adjusting the robot to the initial position...");
        }
        else if (seconds >= 2 && seconds < 15) {  // 2-15秒：静止状态收集数据
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);

            sport_req.StopMove(req);
            pub_go2_request_->publish(req);

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

            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
            pub_go2_request_->publish(req);

            set_state(2);
            RCLCPP_INFO(this->get_logger(), "Collecting positive z-axis rotation data...");
        }
        else if (seconds >= 35 && seconds < 37) {  // 35-37秒：停止运动并保存数据
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pub_speed_->publish(cmd_vel);

            sport_req.StopMove(req);
            pub_go2_request_->publish(req);

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
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_go2_request_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_speed_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::system_clock::time_point beginning_;
    bool file_written_{false};
};

} // namespace calibrate_imu

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<calibrate_imu::Go2IMUCalibrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}