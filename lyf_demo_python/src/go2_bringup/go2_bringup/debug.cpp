// 1、首先需要添加头文件，在 dwb_plugin.cpp 文件顶部添加：


#include <execinfo.h>  // 用于backtrace


void DWBLocalPlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, 
  nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, 
  bool publish_plan)
{
  // 在函数开始处添加
  RCLCPP_INFO(logger_, "Stack trace for goal pose transform:");
  void* buffer[100];                              // 创建缓冲区存储调用栈地址
  int nptrs = backtrace(buffer, 100);            // 获取调用栈
  char** strings = backtrace_symbols(buffer, nptrs);  // 将地址转换为符号名称
  for (int i = 0; i < nptrs; i++) {
    RCLCPP_INFO(logger_, "%s", strings[i]);      // 打印每一层调用
  }
  free(strings);                                 // 释放内存

  // 原有代码继续
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan);
  }
  // ...
}



# 添加调试信息
add_compile_options(-g)
# 如果需要更详细的符号信息
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -rdynamic")




  RCLCPP_INFO(
    get_logger(),
    "Costmap transform tolerance: %f",
    costmap_ros_->getTransformTolerance());