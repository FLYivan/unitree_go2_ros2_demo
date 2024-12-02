#include <explore/costmap_client.h>
#include <unistd.h>

#include <functional>
#include <mutex>
#include <string>

namespace explore
{
// 初始化转换表以加快速度
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> cost_translation_table__ =
    init_translation_table();

// Costmap2DClient类的构造函数
Costmap2DClient::Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf)
  : tf_(tf), node_(node)
{
  std::string costmap_topic; // 定义costmap主题
  std::string costmap_updates_topic; // 定义costmap更新主题

  // 声明参数并设置默认值
  node_.declare_parameter<std::string>("costmap_topic", std::string("costmap"));
  node_.declare_parameter<std::string>("costmap_updates_topic",
                                       std::string("costmap_updates"));
  node_.declare_parameter<std::string>("robot_base_frame", std::string("base_"
                                                                       "link"));
  // transform_tolerance用于所有tf变换
  node_.declare_parameter<double>("transform_tolerance", 0.3);

  // 获取参数值
  node_.get_parameter("costmap_topic", costmap_topic);
  node_.get_parameter("costmap_updates_topic", costmap_updates_topic);
  node_.get_parameter("robot_base_frame", robot_base_frame_);
  node_.get_parameter("transform_tolerance", transform_tolerance_);       

  /* 初始化costmap */
  costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 1000,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_received_ = true;
        updateFullMap(msg);
      });

  // ROS1代码
  // auto costmap_msg =
  // ros::topic::waitForMessage<nav_msgs::msg::OccupancyGrid>(
  //     costmap_topic, subscription_nh);

  // 旋转一些直到回调被调用以复制
  // ros::topic::waitForMessage
  RCLCPP_INFO(node_.get_logger(),
              "Waiting for costmap to become available, topic: %s",
              costmap_topic.c_str());
  while (!costmap_received_) {
    rclcpp::spin_some(node_.get_node_base_interface());
    // 等待一秒
    usleep(1000000);
  }
  // updateFullMap(costmap_msg); // 这已经在costmap_sub_的回调中调用

  /* 订阅地图更新 */
  costmap_updates_sub_ =
      node_.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
          costmap_updates_topic, 1000,
          [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
            updatePartialMap(msg);
          });

  // ROS1代码
  // TODO: 我们需要这个吗？
  /* 解析tf前缀以获取robot_base_frame */
  // std::string tf_prefix = tf::getPrefixParam(node_);
  // robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  // 我们需要确保机器人基座框架和全局框架之间的变换是可用的

  // 全局框架在costmap回调中设置。这就是为什么我们需要
  // 确保接收到costmap

  /* tf变换对于getRobotPose是必要的 */
  auto last_error = node_.now();
  std::string tf_error;
  while (rclcpp::ok() &&
         !tf_->canTransform(global_frame_, robot_base_frame_,
                            tf2::TimePointZero, tf2::durationFromSec(0.1),
                            &tf_error)) {
    rclcpp::spin_some(node_.get_node_base_interface());
    if (last_error + tf2::durationFromSec(5.0) < node_.now()) {
      RCLCPP_WARN(node_.get_logger(),
                  "Timed out waiting for transform from %s to %s to become "
                  "available "
                  "before subscribing to costmap, tf error: %s",
                  robot_base_frame_.c_str(), global_frame_.c_str(),
                  tf_error.c_str());
      last_error = node_.now();
      ;
    }
    // 错误字符串将累积，错误通常是相同的，
    // 所以最后一个警告就够了。重置字符串以避免累积。
    tf_error.clear();
  }
}

// 更新完整地图
void Costmap2DClient::updateFullMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_frame_ = msg->header.frame_id; // 设置全局框架

  unsigned int size_in_cells_x = msg->info.width; // 获取地图宽度
  unsigned int size_in_cells_y = msg->info.height; // 获取地图高度
  double resolution = msg->info.resolution; // 获取地图分辨率
  double origin_x = msg->info.origin.position.x; // 获取地图原点x坐标
  double origin_y = msg->info.origin.position.y; // 获取地图原点y坐标

  RCLCPP_DEBUG(node_.get_logger(), "received full new map, resizing to: %d, %d",
               size_in_cells_x, size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                     origin_y); // 调整地图大小

  // 锁定以访问底层地图
  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // 用数据填充地图
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  RCLCPP_DEBUG(node_.get_logger(), "full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = cost_translation_table__[cell_cost];
  }
  RCLCPP_DEBUG(node_.get_logger(), "map updated, written %lu values",
               costmap_size);
}

// 更新部分地图
void Costmap2DClient::updatePartialMap(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  RCLCPP_DEBUG(node_.get_logger(), "received partial map update");
  global_frame_ = msg->header.frame_id; // 设置全局框架

  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_DEBUG(node_.get_logger(),
                 "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                 msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x); // 获取更新区域的起始x坐标
  size_t y0 = static_cast<size_t>(msg->y); // 获取更新区域的起始y坐标
  size_t xn = msg->width + x0; // 获取更新区域的结束x坐标
  size_t yn = msg->height + y0; // 获取更新区域的结束y坐标

  // 锁定以访问底层地图
  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX(); // 获取地图宽度
  size_t costmap_yn = costmap_.getSizeInCellsY(); // 获取地图高度

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn) {
    RCLCPP_WARN(node_.get_logger(),
                "received update doesn't fully fit into existing map, "
                "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                "map is: [0, %lu], [0, %lu]",
                x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  // 用数据更新地图
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      ++i;
    }
  }
}

// 获取机器人位姿
geometry_msgs::msg::Pose Costmap2DClient::getRobotPose() const
{
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Pose empty_pose;
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = node_.now();

  auto& clk = *node_.get_clock();

  // 获取机器人的全局位姿
  try {
    robot_pose = tf_->transform(robot_pose, global_frame_,
                                tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "No Transform available Error looking up robot pose: "
                          "%s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Connectivity Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Extrapolation Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000, "Other error: %s\n",
                          ex.what());
    return empty_pose;
  }

  return robot_pose.pose;
}

// 初始化转换表
std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // 线性映射从[0..100]到[0..255]
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // 特殊值:
  cost_translation_table[0] = 0;      // 无障碍物
  cost_translation_table[99] = 253;   // 内部障碍物
  cost_translation_table[100] = 254;  // 致命障碍物
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // 未知

  return cost_translation_table;
}

}  // namespace explore
