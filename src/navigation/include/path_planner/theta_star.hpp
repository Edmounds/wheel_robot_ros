#ifndef PATH_PLANNER__THETA_STAR_HPP_
#define PATH_PLANNER__THETA_STAR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace path_planner {

/// 2D 栅格坐标。
struct GridCell {
  int x;
  int y;

  bool operator==(const GridCell& other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const GridCell& other) const { return !(*this == other); }
};

/// 世界坐标点。
struct WorldPoint {
  double x;
  double y;
};

/// 2D 栅格地图封装，提供障碍物查询和视线检查。
class GridMap {
 public:
  /// 从 OccupancyGrid 构建栅格地图。
  ///
  /// Args:
  ///   grid: ROS 占用栅格消息。
  ///   lethal_cost_threshold: 代价值大于等于此阈值视为障碍物。
  ///   allow_unknown: 是否将未知区域(-1)视为可通行。
  GridMap(const nav_msgs::msg::OccupancyGrid& grid,
          int8_t lethal_cost_threshold,
          bool allow_unknown);

  /// 从 PGM 文件构建栅格地图。
  ///
  /// Args:
  ///   pgm_path: PGM 图像文件路径。
  ///   resolution: 每像素对应的米数。
  ///   origin_x: 地图原点 x (m)。
  ///   origin_y: 地图原点 y (m)。
  ///   occupied_thresh: 占用概率高于此阈值视为障碍物。
  ///   free_thresh: 占用概率低于此阈值视为可通行。
  GridMap(const std::string& pgm_path,
          double resolution,
          double origin_x,
          double origin_y,
          double occupied_thresh,
          double free_thresh,
          int8_t lethal_cost_threshold = 65,
          bool allow_unknown = false);

  /// 查询某栅格是否被占用。
  bool is_occupied(int gx, int gy) const;

  /// @brief 从地图 YAML 文件和 PGM 图像构建栅格地图，并自动进行膨胀。
  /// @param yaml_path YAML 配置文件路径。
  /// @param inflation_radius_m 膨胀半径 (m)。
  /// @return 构建好的 GridMap。
  static GridMap from_map_yaml(const std::string& yaml_path,
                               double inflation_radius_m,
                               int8_t lethal_cost_threshold = 65,
                               bool allow_unknown = false);

  /// @brief 使用 BFS 对所有障碍物向外膨胀给定的半径。
  /// @param inflation_radius_m 膨胀半径 (m)。
  void inflate(double inflation_radius_m);

  /// @brief 将 GridMap 转为 ROS 2 的 OccupancyGrid 消息以供 RViz 显示。
  /// @return /nav_msgs/msg/OccupancyGrid 消息（需调用方补全 header）。
  nav_msgs::msg::OccupancyGrid to_occupancy_grid() const;

  /// 查询某栅格是否在地图范围内。
  bool in_bounds(int gx, int gy) const;

  /// Bresenham 直线视线检查，两栅格间是否无障碍。
  bool line_of_sight(const GridCell& a, const GridCell& b) const;

  /// 使用新的 OccupancyGrid 数据更新当前地图，尽量复用内部存储。
  void update(const nav_msgs::msg::OccupancyGrid& grid,
              int8_t lethal_cost_threshold,
              bool allow_unknown);

  /// 判断地图几何布局是否一致，可用于决定是否复用内存。
  bool has_same_layout(const nav_msgs::msg::OccupancyGrid& grid) const;

  /// 世界坐标转栅格坐标。
  GridCell world_to_grid(double wx, double wy) const;

  /// 栅格坐标转世界坐标（取栅格中心）。
  WorldPoint grid_to_world(int gx, int gy) const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }

 private:
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<int8_t> data_;
  int8_t lethal_threshold_;
  bool allow_unknown_;
};

/// Theta* 路径搜索算法。
class ThetaStar {
 public:
  /// 构造 Theta* 搜索器。
  ///
  /// Args:
  ///   weight: 启发式加权因子，>1 偏向贪心搜索。
  explicit ThetaStar(double weight = 1.0,
                     std::size_t max_search_iterations = 250000);

  /// 在栅格地图上搜索从起点到终点的路径。
  ///
  /// Args:
  ///   map: 2D 栅格地图。
  ///   start: 起点世界坐标。
  ///   goal: 终点世界坐标。
  ///
  /// Returns:
  ///   世界坐标路径点序列，空表示搜索失败。
  std::vector<WorldPoint> search(const GridMap& map,
                                 const WorldPoint& start,
                                 const WorldPoint& goal) const;

  const std::string& last_failure_reason() const { return last_failure_reason_; }

 private:
  double weight_;
  std::size_t max_search_iterations_;
  mutable std::string last_failure_reason_;
};

/// 对路径进行线性插值，确保相邻点间距不超过指定值。
///
/// Args:
///   path: 原始路径点序列。
///   max_spacing: 最大点间距 (m)。
///
/// Returns:
///   插值后的路径。
std::vector<WorldPoint> interpolate_path(const std::vector<WorldPoint>& path,
                                         double max_spacing);

}  // namespace path_planner

#endif  // PATH_PLANNER__THETA_STAR_HPP_
