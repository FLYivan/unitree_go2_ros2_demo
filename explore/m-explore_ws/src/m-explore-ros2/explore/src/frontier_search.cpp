#include <explore/costmap_tools.h> // 包含costmap工具头文件
#include <explore/frontier_search.h> // 包含前沿搜索头文件

#include <geometry_msgs/msg/point.hpp> // 包含几何消息点头文件
#include <mutex> // 包含互斥锁头文件

#include "nav2_costmap_2d/cost_values.hpp" // 包含costmap值头文件

namespace frontier_exploration // 定义前沿探索命名空间
{
using nav2_costmap_2d::FREE_SPACE; // 使用nav2_costmap_2d中的FREE_SPACE
using nav2_costmap_2d::LETHAL_OBSTACLE; // 使用nav2_costmap_2d中的LETHAL_OBSTACLE
using nav2_costmap_2d::NO_INFORMATION; // 使用nav2_costmap_2d中的NO_INFORMATION

// FrontierSearch类的构造函数
FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap) // 初始化costmap_
  , potential_scale_(potential_scale) // 初始化potential_scale_
  , gain_scale_(gain_scale) // 初始化gain_scale_
  , min_frontier_size_(min_frontier_size) // 初始化min_frontier_size_
{
}

// searchFrom方法——从指定pose.position开始搜索前沿
std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list; // 定义前沿列表

  // 检查机器人是否在costmap范围内
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(rclcpp::get_logger("FrontierSearch"), "Robot out of costmap "
                                                       "bounds, cannot search "
                                                       "for frontiers");
    return frontier_list; // 如果机器人不在costmap范围内，返回空的前沿列表
  }

  // 确保地图在搜索期间是一致且锁定的
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap(); // 获取地图数据
  size_x_ = costmap_->getSizeInCellsX(); // 获取地图的宽度（以单元格为单位）
  size_y_ = costmap_->getSizeInCellsY(); // 获取地图的高度（以单元格为单位）

  // 初始化标志数组以跟踪已访问和前沿单元格
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // 初始化广度优先搜索
  std::queue<unsigned int> bfs;

  // 找到最近的清晰单元格以开始搜索
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(rclcpp::get_logger("FrontierSearch"), "Could not find nearby "
                                                      "clear cell to start "
                                                      "search");
  }
  visited_flag[bfs.front()] = true; // 标记起始单元格为已访问

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // 遍历4连通邻域
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // 将所有自由且未访问的单元格添加到队列中
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // 检查单元格是否为新的前沿单元格（未访问、无信息、自由邻居）
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier); // 如果前沿大小大于最小前沿大小，将其添加到前沿列表中
        }
      }
    }
  }

  // 设置前沿的成本
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; }); // 按成本对前沿进行排序

  return frontier_list; // 返回前沿列表
}

// 构建新的前沿
Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // 初始化前沿结构
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // 记录前沿的初始接触点
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // 将初始网格单元推入队列
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // 缓存参考位置的世界坐标
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // 尝试将8连通邻域中的单元格添加到前沿
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // 检查邻居是否为潜在的前沿单元格
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // 标记单元格为前沿
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point); // 将点添加到前沿点列表中

        // 更新前沿大小
        output.size++;

        // 更新前沿的质心
        output.centroid.x += wx;
        output.centroid.y += wy;

        // 确定前沿块中最近的前沿点到机器人的距离，按离机器人最近的网格单元计算
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // 将单元格添加到广度优先搜索队列中
        bfs.push(nbr);
      }
    }
  }

  // 平均前沿质心
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output; // 返回前沿
}

// 检查单元格是否为新的前沿单元格
bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // 检查单元格是否为未知且未标记为前沿
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // 前沿单元格应至少有一个4连通邻域单元格是自由的
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

// 计算前沿的成本
double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration
