/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <explore/explore.h> // 导入explore头文件

#include <thread> // 导入线程库

// 定义一个内联静态函数，用于判断两个点是否相同
inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x; // 计算x坐标差
  double dy = one.y - two.y; // 计算y坐标差
  double dist = sqrt(dx * dx + dy * dy); // 计算两点之间的距离
  return dist < 0.01; // 如果距离小于0.01，则认为两点相同
}

namespace explore
{
// Explore类的构造函数
Explore::Explore()
  : Node("explore_node") // 初始化节点名称为"explore_node"
  , tf_buffer_(this->get_clock()) // 初始化tf_buffer_
  , tf_listener_(tf_buffer_) // 初始化tf_listener_
  , costmap_client_(*this, &tf_buffer_) // 初始化costmap_client_
  , prev_distance_(0) // 初始化prev_distance_为0
  , last_markers_count_(0) // 初始化last_markers_count_为0
{
  double timeout; // 定义超时时间变量
  double min_frontier_size; // 定义最小前沿大小变量
  // 声明参数并设置默认值
  this->declare_parameter<float>("planner_frequency", 1.0); // 规划器频率
  this->declare_parameter<float>("progress_timeout", 30.0); // 进度超时时间
  this->declare_parameter<bool>("visualize", false); // 是否可视化
  this->declare_parameter<float>("potential_scale", 1e-3); // 用于对边界进行加权。这个乘法参数影响边界权重的边界势分量（到边界的距离）。
  this->declare_parameter<float>("orientation_scale", 0.0); // 方向缩放
  this->declare_parameter<float>("gain_scale", 1.0); // 用于对边界进行加权。该乘法参数影响边界权重的边界增益分量（边界大小）。
  this->declare_parameter<float>("min_frontier_size", 0.5); // 将边界视为勘探目标的最小边界尺寸。数值单位为米。
  this->declare_parameter<bool>("return_to_init", false); // 是否返回初始位置

  // 获取参数值
  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout; // 设置进度超时时间
  // 创建move_base客户端
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  // 初始化前沿搜索
  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  // 如果需要可视化，创建MarkerArray发布者
  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // 订阅resume（恢复探索）和stop（停止探索）的主题
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,                                                       //向explore/resume主题发布False可以暂停探索，发布True可以恢复探索
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server"); // 打印等待连接move_base nav2服务器的信息
  move_base_client_->wait_for_action_server(); // 等待move_base服务器
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server"); // 打印已连接move_base nav2服务器的信息

  // 如果需要返回初始位置，获取机器人的初始位姿
  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot"); // 打印获取机器人初始位姿的信息
    geometry_msgs::msg::TransformStamped transformStamped; // 定义TransformStamped变量
    std::string map_frame = costmap_client_.getGlobalFrameID(); // 获取全局坐标系ID
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero); // 查找变换
      initial_pose_.position.x = transformStamped.transform.translation.x; // 设置初始位置x坐标
      initial_pose_.position.y = transformStamped.transform.translation.y; // 设置初始位置y坐标
      initial_pose_.orientation = transformStamped.transform.rotation; // 设置初始方向
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what()); // 打印错误信息
      return_to_init_ = false; // 设置return_to_init_为false
    }
  }

  // 创建定时器，定期调用makePlan函数
  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { restart(); });
  // 立即开始探索
  makePlan();
}


// 定义定时器回调函数restart

void Explore::restart()
{

  RCLCPP_INFO(logger_, "定时器被触发，调用makePlan函数"); // 打印定时器触发信息
  makePlan();

}

// Explore类的析构函数
Explore::~Explore()
{
  stop(); // 停止探索
}

// resumeCallback函数，用于处理resume主题的回调
void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume(); // 如果消息数据为true，恢复探索
  } else {
    stop(); // 否则，停止探索
  }
}

// visualizeFrontiers函数，用于可视化前沿
void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  // 定义颜色
  // 定义蓝色，表示前沿不在黑名单中
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;   // 红色分量为0
  blue.g = 0;   // 绿色分量为0
  blue.b = 1.0; // 蓝色分量为1.0
  blue.a = 1.0; // 透明度为1.0（不透明）

  // 定义红色，表示前沿在黑名单中
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;  // 红色分量为1.0
  red.g = 0;    // 绿色分量为0
  red.b = 0;    // 蓝色分量为0
  red.a = 1.0;  // 透明度为1.0（不透明）

  // 定义绿色，备用颜色
  std_msgs::msg::ColorRGBA green;
  green.r = 0;   // 红色分量为0
  green.g = 1.0; // 绿色分量为1.0
  green.b = 0;   // 蓝色分量为0
  green.a = 1.0; // 透明度为1.0（不透明）

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size()); // 打印可视化前沿的数量
  visualization_msgs::msg::MarkerArray markers_msg; // 定义MarkerArray消息
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers; // 获取Marker数组
  visualization_msgs::msg::Marker m; // 定义Marker

  m.header.frame_id = costmap_client_.getGlobalFrameID(); // 设置Marker的坐标系ID
  m.header.stamp = this->now(); // 设置Marker的时间戳
  m.ns = "frontiers"; // 设置Marker的命名空间
  m.scale.x = 1.0; // 设置Marker的x轴缩放
  m.scale.y = 1.0; // 设置Marker的y轴缩放
  m.scale.z = 1.0; // 设置Marker的z轴缩放
  m.color.r = 0; // 设置Marker的颜色
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // 设置Marker的生命周期
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // 在ELOQUENT版本中
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // 在DASHING版本中
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // 在FOXY及以后的版本中
#endif
  m.frame_locked = true; // 设置Marker的帧锁定

  // 前沿按权重排序
  //  frontiers 向量不为空时，获取其第一个元素的 cost 值作为 min_cost，否则将 min_cost 设置为 0
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost; 

  m.action = visualization_msgs::msg::Marker::ADD; // 设置Marker的动作为添加
  size_t id = 0; // 初始化Marker的ID
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS; // 设置Marker的类型为点
    m.id = int(id); // 设置Marker的ID
    m.scale.x = 0.1; // 设置Marker的x轴缩放
    m.scale.y = 0.1; // 设置Marker的y轴缩放
    m.scale.z = 0.1; // 设置Marker的z轴缩放
    m.points = frontier.points; // 设置Marker的点
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red; // 如果目标在黑名单中，设置颜色为红色
    } else {
      m.color = blue; // 否则，设置颜色为蓝色
    }
    markers.push_back(m); // 将Marker添加到数组中
    ++id; // 增加ID
    m.type = visualization_msgs::msg::Marker::SPHERE; // 设置Marker的类型为球体
    m.id = int(id); // 设置Marker的ID
    m.pose.position = frontier.initial; // 设置Marker的位置

    // 绿色球体是前沿的初始位置，可以作为目标点，一般为前沿质心
    // 前沿成本越高，球体越小
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5); // 计算缩放比例
    m.scale.x = scale; // 设置Marker的x轴缩放
    m.scale.y = scale; // 设置Marker的y轴缩放
    m.scale.z = scale; // 设置Marker的z轴缩放
    m.points = {}; // 清空Marker的点
    m.color = green; // 设置Marker的颜色为绿色
    markers.push_back(m); // 将Marker添加到数组中
    ++id; // 增加ID
  }
  size_t current_markers_count = markers.size(); // 获取当前Marker的数量

  // 删除之前未使用的Marker
  m.action = visualization_msgs::msg::Marker::DELETE; // 设置Marker的动作为删除
  for (; id < last_markers_count_; ++id) {
    m.id = int(id); // 设置Marker的ID
    markers.push_back(m); // 将Marker添加到数组中
  }

  last_markers_count_ = current_markers_count; // 更新最后的Marker数量
  marker_array_publisher_->publish(markers_msg); // 发布MarkerArray消息
}

// makePlan函数，用于生成探索计划
void Explore::makePlan()
{
  // 获取机器人的位姿
  auto pose = costmap_client_.getRobotPose();
  // 获取按成本排序的前沿
  auto frontiers = search_.searchFrom(pose.position);     //调用FrontierSearch类实现，search_是FrontierSearch类的实例化
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size()); // 打印找到的前沿数量
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost); // 打印每个前沿的成本
  }

// frontiers是一个存储前沿信息的容器，通常是一个std::vector。如果这个容器为空，意味着当前没有可供探索的前沿。
  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping."); // 如果没有找到前沿，打印警告信息并停止
    stop(true);
    return;
  }

  // 如果需要可视化，发布前沿作为可视化标记
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // 查找不在黑名单中的前沿
  auto frontier = // 查找不在黑名单中的前沿
      std::find_if_not(frontiers.begin(), frontiers.end(), // 从前沿开始查找
                       [this](const frontier_exploration::Frontier& f) { // 使用lambda函数
                         return goalOnBlacklist(f.centroid); // 判断目标是否在黑名单中
                       });
                       
// std::find_if_not 函数用于从 frontiers 列表中查找第一个不在黑名单中的前沿。

// 如果 std::find_if_not 返回的迭代器等于 frontiers.end()，这意味着没有找到任何不在黑名单中的前沿
  if (frontier == frontiers.end()) { // 如果所有前沿都在黑名单中
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping."); // 如果所有前沿都在黑名单中，打印警告信息并停止
    stop(true); // 停止
    return; // 返回
  }
  geometry_msgs::msg::Point target_position = frontier->centroid; // 将centroid属性，即前沿中心，赋给目标点变量

  // 如果没有进展，则超时
  bool same_goal = same_point(prev_goal_, target_position); // 判断目标是否相同

  prev_goal_ = target_position; // 更新上一个目标
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // 如果机器人有了新的目标位置，或者在向目标前进（即距离在缩短）
    //  则更新最后进展时间和上一个距离
    last_progress_ = this->now();         // 上一次有进展的时间
    prev_distance_ = frontier->min_distance;
  }

  // 如果长时间没有进展，将目标加入黑名单（完全根据进展时间是否超时，来决定是否加黑名单）
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position); // 将目标加入黑名单
    RCLCPP_DEBUG(logger_, "Adding current goal to black list"); // 打印将目标加入黑名单的信息
    makePlan(); // 重新生成计划
    return;
  }

  // 确保只有第一次调用makePlan时将resuming_设置为true
  if (resuming_) {
    resuming_ = false;
  }

  // 如果仍在追求相同的目标，则不需要做任何事情
  if (same_goal) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2"); // 打印发送目标到move base nav2的信息

  // 如果有新目标，发送目标到move_base
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position; // 设置目标位置
  goal.pose.pose.orientation.w = 1.; // 设置到达目标后的方向
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID(); // 设置目标的坐标系ID
  goal.pose.header.stamp = this->now(); // 设置目标的时间戳（这里可能会存在时间戳不一致）

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // 设置结果回调函数
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options); // 异步发送目标
}

// returnToInitialPose函数，用于返回初始位姿
void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose."); // 打印返回初始位姿的信息
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position; // 设置目标位置为初始位置，这个值，是节点初始化时，从那个时点的tf变换中获取的
  goal.pose.pose.orientation = initial_pose_.orientation; // 设置目标方向为初始方向
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID(); // 设置目标的坐标系ID
  goal.pose.header.stamp = this->now(); // 设置目标的时间戳（这里可能会存在时间戳不一致）

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options); // 异步发送目标
}

// goalOnBlacklist函数，用于判断目标是否在黑名单中
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 2; // 定义容差是分辨率的几倍，容差约大，约容易误判目标点在黑名单 （自定义）
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap(); // 获取costmap

  // 检查目标是否在黑名单中
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x); // 计算x坐标差
    double y_diff = fabs(goal.y - frontier_goal.y); // 计算y坐标差

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true; // 如果在容差范围内，返回true
  }
  return false; // 否则，返回false
}

// reachedGoal函数，用于处理到达目标的回调
void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:        // nav2返回到达目标点成功
      RCLCPP_DEBUG(logger_, "Goal was successful"); // 如果目标成功，打印调试信息
      break;
    case rclcpp_action::ResultCode::ABORTED:        // nav2返回目标点不可行，或有障碍
      RCLCPP_DEBUG(logger_, "Goal was aborted"); // 如果目标被中止，打印调试信息
      frontier_blacklist_.push_back(frontier_goal); // 将目标加入黑名单
      RCLCPP_DEBUG(logger_, "Adding current goal to black list"); // 打印将目标加入黑名单的信息
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled"); // 如果目标被取消，打印调试信息
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2"); // 如果结果代码未知，打印警告信息
      break;
  }
  makePlan(); // 立即生成新目标
}

// start函数，用于开始探索
void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started."); // 打印探索开始的信息
}

// stop函数，用于停止探索
void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped."); // 打印探索停止的信息
  move_base_client_->async_cancel_all_goals(); // 异步取消所有目标
  exploring_timer_->cancel(); // 取消定时器

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose(); // 如果需要返回初始位置并且探索完成，返回初始位姿
  }
}

// resume函数，用于恢复探索
void Explore::resume()
{
  resuming_ = true; // 设置resuming_为true
  RCLCPP_INFO(logger_, "Exploration resuming."); // 打印探索恢复的信息
  exploring_timer_->reset(); // 重置定时器
  makePlan(); // 立即生成新目标
}

}  // namespace explore

// main函数，程序入口
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // 初始化rclcpp
  rclcpp::spin(
      std::make_shared<explore::Explore>()); // 创建Explore对象并运行
  rclcpp::shutdown(); // 关闭rclcpp
  return 0; // 返回0
}
