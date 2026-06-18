#include "path_planner/theta_star.hpp"
#include "path_planner/math_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

namespace path_planner {

namespace {

bool diagonal_corner_blocked(const GridMap& map,
                             const GridCell& from,
                             const GridCell& to) {
  const int step_x = to.x - from.x;
  const int step_y = to.y - from.y;
  if (std::abs(step_x) != 1 || std::abs(step_y) != 1) {
    return false;
  }

  const bool side_x_blocked = map.is_occupied(from.x + step_x, from.y);
  const bool side_y_blocked = map.is_occupied(from.x, from.y + step_y);
  return side_x_blocked && side_y_blocked;
}

int8_t occupancy_from_pgm_value(int pixel_value,
                                int max_value,
                                double occupied_thresh,
                                double free_thresh) {
  const double normalized_intensity = static_cast<double>(pixel_value) / max_value;
  const double occupancy_probability = 1.0 - normalized_intensity;
  if (occupancy_probability >= occupied_thresh) {
    return 100;
  }
  if (occupancy_probability <= free_thresh) {
    return 0;
  }
  return -1;
}

}  // namespace

// ============================================================================
// GridMap
// ============================================================================

GridMap::GridMap(const nav_msgs::msg::OccupancyGrid& grid,
                 int8_t lethal_cost_threshold,
                 bool allow_unknown)
    : width_(0),
      height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      lethal_threshold_(lethal_cost_threshold),
      allow_unknown_(allow_unknown) {
  update(grid, lethal_cost_threshold, allow_unknown);
}

GridMap::GridMap(const std::string& pgm_path,
                 double resolution,
                 double origin_x,
                 double origin_y,
                 double occupied_thresh,
                 double free_thresh,
                 int8_t lethal_cost_threshold,
                 bool allow_unknown)
    : resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      lethal_threshold_(lethal_cost_threshold),
      allow_unknown_(allow_unknown) {
  std::ifstream file(pgm_path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("无法打开 PGM 文件: " + pgm_path);
  }

  std::string magic;
  file >> magic;
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("不支持的 PGM 格式: " + magic);
  }

  // 跳过注释行
  auto skip_comments = [&file]() {
    while (file.peek() == '#' || file.peek() == '\n' || file.peek() == ' ') {
      if (file.peek() == '#') {
        std::string line;
        std::getline(file, line);
      } else {
        file.get();
      }
    }
  };

  skip_comments();
  file >> width_ >> height_;
  if (!file.good()) {
    throw std::runtime_error("PGM 头部尺寸字段不完整: " + pgm_path);
  }
  skip_comments();
  int max_val;
  file >> max_val;
  if (!file.good()) {
    throw std::runtime_error("PGM 最大像素值字段不完整: " + pgm_path);
  }
  const int separator = file.get();
  if (separator == EOF) {
    throw std::runtime_error("PGM 像素数据缺少分隔符: " + pgm_path);
  }
  if (magic == "P5" && !std::isspace(separator)) {
    throw std::runtime_error("PGM 二进制像素数据前缺少空白分隔: " + pgm_path);
  }

  // 读取像素数据并转换为占用值
  data_.resize(width_ * height_);
  if (magic == "P5") {
    std::vector<uint8_t> raw(width_ * height_);
    file.read(reinterpret_cast<char*>(raw.data()), raw.size());
    if (file.gcount() != static_cast<std::streamsize>(raw.size())) {
      throw std::runtime_error("PGM 二进制像素数据长度不足: " + pgm_path);
    }

    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        // PGM 行序与地图行序相反（PGM 从上到下，地图从下到上）
        int pgm_idx = row * width_ + col;
        int map_idx = (height_ - 1 - row) * width_ + col;
        data_[map_idx] = occupancy_from_pgm_value(raw[pgm_idx], max_val, occupied_thresh,
                                                  free_thresh);
      }
    }
  } else {
    // P2 文本格式
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        int val;
        file >> val;
        if (!file.good()) {
          throw std::runtime_error("PGM 文本像素数据不完整: " + pgm_path);
        }
        int map_idx = (height_ - 1 - row) * width_ + col;
        data_[map_idx] =
            occupancy_from_pgm_value(val, max_val, occupied_thresh, free_thresh);
      }
    }
  }
}

GridMap GridMap::from_map_yaml(const std::string& yaml_path,
                               double inflation_radius_m,
                               int8_t lethal_cost_threshold,
                               bool allow_unknown) {
  std::filesystem::path resolved_yaml_path(yaml_path);
  if (!resolved_yaml_path.is_absolute()) {
    resolved_yaml_path =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("navigation")) /
        "maps" / resolved_yaml_path;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(resolved_yaml_path.string());
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("读取地图 YAML 失败: " + std::string(e.what()));
  }

  std::string image_name = config["image"].as<std::string>();
  double resolution = config["resolution"].as<double>();
  auto origin_node = config["origin"];
  double origin_x = origin_node[0].as<double>();
  double origin_y = origin_node[1].as<double>();
  double occupied_thresh = config["occupied_thresh"].as<double>();
  double free_thresh = config["free_thresh"].as<double>();

  const std::filesystem::path yaml_dir = resolved_yaml_path.parent_path();
  const std::filesystem::path image_path(image_name);
  const std::string pgm_path =
      (image_path.is_absolute() ? image_path : (yaml_dir / image_path)).string();

  GridMap map(pgm_path, resolution, origin_x, origin_y, occupied_thresh, free_thresh,
              lethal_cost_threshold, allow_unknown);
  if (inflation_radius_m > 0.0) {
    map.inflate(inflation_radius_m);
  }
  return map;
}

void GridMap::inflate(double inflation_radius_m) {
  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_m / resolution_));
  if (inflation_cells <= 0) return;

  // 使用欧几里得距离平方阈值，确保膨胀区域为圆形而非正方形
  const int inflation_cells_sq = inflation_cells * inflation_cells;

  // 收集所有原始障碍物栅格作为 BFS 种子
  struct InflationSeed {
    int x;
    int y;
    int origin_x;
    int origin_y;
  };
  std::vector<InflationSeed> queue;
  std::vector<bool> visited(width_ * height_, false);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (data_[y * width_ + x] >= lethal_threshold_) {
        queue.push_back({x, y, x, y});
        visited[y * width_ + x] = true;
      }
    }
  }

  size_t head = 0;
  const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

  while (head < queue.size()) {
    const InflationSeed current = queue[head++];

    for (int i = 0; i < 8; ++i) {
      const int nx = current.x + dx[i];
      const int ny = current.y + dy[i];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

      const int n_idx = ny * width_ + nx;
      if (visited[n_idx]) continue;

      // 检查到源障碍物的欧几里得距离平方
      const int ddx = nx - current.origin_x;
      const int ddy = ny - current.origin_y;
      if (ddx * ddx + ddy * ddy > inflation_cells_sq) continue;

      visited[n_idx] = true;
      if (data_[n_idx] < lethal_threshold_ && data_[n_idx] >= 0) {
        data_[n_idx] = lethal_threshold_;
      } else if (data_[n_idx] < 0 && allow_unknown_) {
        data_[n_idx] = lethal_threshold_;
      }
      queue.push_back({nx, ny, current.origin_x, current.origin_y});
    }
  }
}

nav_msgs::msg::OccupancyGrid GridMap::to_occupancy_grid() const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data = data_;
  return grid;
}

void GridMap::update(const nav_msgs::msg::OccupancyGrid& grid,
                     int8_t lethal_cost_threshold,
                     bool allow_unknown) {
  width_ = static_cast<int>(grid.info.width);
  height_ = static_cast<int>(grid.info.height);
  resolution_ = grid.info.resolution;
  origin_x_ = grid.info.origin.position.x;
  origin_y_ = grid.info.origin.position.y;
  lethal_threshold_ = lethal_cost_threshold;
  allow_unknown_ = allow_unknown;
  data_.assign(grid.data.begin(), grid.data.end());
}

bool GridMap::has_same_layout(const nav_msgs::msg::OccupancyGrid& grid) const {
  return width_ == static_cast<int>(grid.info.width) &&
         height_ == static_cast<int>(grid.info.height) &&
         approx_equal(resolution_, grid.info.resolution) &&
         approx_equal(origin_x_, grid.info.origin.position.x) &&
         approx_equal(origin_y_, grid.info.origin.position.y);
}

bool GridMap::is_occupied(int gx, int gy) const {
  if (!in_bounds(gx, gy)) {
    return true;
  }
  int8_t val = data_[gy * width_ + gx];
  if (val < 0) {
    return !allow_unknown_;
  }
  return val >= lethal_threshold_;
}

bool GridMap::in_bounds(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

bool GridMap::line_of_sight(const GridCell& a, const GridCell& b) const {
  // Bresenham 直线检查
  int dx = std::abs(b.x - a.x);
  int dy = std::abs(b.y - a.y);
  int sx = (a.x < b.x) ? 1 : -1;
  int sy = (a.y < b.y) ? 1 : -1;
  int err = dx - dy;
  int cx = a.x;
  int cy = a.y;

  while (true) {
    if (is_occupied(cx, cy)) {
      return false;
    }
    if (cx == b.x && cy == b.y) {
      break;
    }

    const int prev_x = cx;
    const int prev_y = cy;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      cx += sx;
    }
    if (e2 < dx) {
      err += dx;
      cy += sy;
    }
    if (diagonal_corner_blocked(*this, {prev_x, prev_y}, {cx, cy})) {
      return false;
    }
  }
  return true;
}

GridCell GridMap::world_to_grid(double wx, double wy) const {
  return {
      static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
      static_cast<int>(std::floor((wy - origin_y_) / resolution_))};
}

WorldPoint GridMap::grid_to_world(int gx, int gy) const {
  return {
      origin_x_ + (gx + 0.5) * resolution_,
      origin_y_ + (gy + 0.5) * resolution_};
}

// ============================================================================
// ThetaStar
// ============================================================================

/// 搜索节点内部状态。
struct SearchNode {
  GridCell cell;
  double g_cost;
  double f_cost;

  bool operator>(const SearchNode& other) const {
    return f_cost > other.f_cost;
  }
};

/// 将 GridCell 哈希为整数键。
static inline int64_t cell_key(const GridCell& c, int width) {
  return static_cast<int64_t>(c.y) * width + c.x;
}

/// 欧几里得距离启发函数。
static inline double heuristic(const GridCell& a, const GridCell& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

ThetaStar::ThetaStar(double weight, std::size_t max_search_iterations)
    : weight_(weight),
      max_search_iterations_(std::max<std::size_t>(1, max_search_iterations)) {}

std::vector<WorldPoint> ThetaStar::search(const GridMap& map,
                                          const WorldPoint& start,
                                          const WorldPoint& goal) const {
  last_failure_reason_.clear();
  GridCell start_cell = map.world_to_grid(start.x, start.y);
  GridCell goal_cell = map.world_to_grid(goal.x, goal.y);

  if (map.is_occupied(start_cell.x, start_cell.y)) {
    last_failure_reason_ = "起点落在障碍物内";
    return {};
  }
  if (map.is_occupied(goal_cell.x, goal_cell.y)) {
    last_failure_reason_ = "终点落在障碍物内";
    return {};
  }

  int w = map.width();
  auto key = [w](const GridCell& c) { return cell_key(c, w); };

  // g 值表
  std::unordered_map<int64_t, double> g_score;
  // 父节点表
  std::unordered_map<int64_t, GridCell> parent;
  // open 集合标记
  std::unordered_map<int64_t, bool> closed;

  // 优先队列
  std::priority_queue<SearchNode, std::vector<SearchNode>,
                      std::greater<SearchNode>>
      open;

  g_score[key(start_cell)] = 0.0;
  parent[key(start_cell)] = start_cell;
  open.push({start_cell, 0.0, weight_ * heuristic(start_cell, goal_cell)});

  // 8 连通邻居偏移
  static const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const double dcost[] = {
      1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

  std::size_t iterations = 0;
  while (!open.empty()) {
    ++iterations;
    if (iterations > max_search_iterations_) {
      std::ostringstream oss;
      oss << "Theta* 搜索达到最大迭代次数限制(" << max_search_iterations_
          << "): start=(" << start.x << ", " << start.y
          << "), goal=(" << goal.x << ", " << goal.y << ")";
      last_failure_reason_ = oss.str();
      return {};
    }
    SearchNode current = open.top();
    open.pop();

    if (current.cell == goal_cell) {
      // 回溯路径
      std::vector<WorldPoint> path;
      GridCell c = goal_cell;
      while (!(c == start_cell)) {
        WorldPoint wp = map.grid_to_world(c.x, c.y);
        path.push_back(wp);
        c = parent[key(c)];
      }
      // 使用精确的起点和终点坐标
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      if (path.size() >= 2) {
        path.back() = goal;
      }
      return path;
    }

    int64_t ck = key(current.cell);
    if (closed.count(ck)) {
      continue;
    }
    closed[ck] = true;

    for (int i = 0; i < 8; ++i) {
      GridCell neighbor = {current.cell.x + dx[i],
                           current.cell.y + dy[i]};

      if (!map.in_bounds(neighbor.x, neighbor.y) ||
          map.is_occupied(neighbor.x, neighbor.y) ||
          diagonal_corner_blocked(map, current.cell, neighbor)) {
        continue;
      }

      int64_t nk = key(neighbor);
      if (closed.count(nk)) {
        continue;
      }

      // Theta* 核心：尝试从 current 的父节点直接连接到 neighbor
      GridCell& p = parent[ck];
      double tentative_g;
      GridCell best_parent;

      if (map.line_of_sight(p, neighbor)) {
        // Path 2: 从父节点直连
        double dist = heuristic(p, neighbor);
        tentative_g = g_score[key(p)] + dist;
        best_parent = p;
      } else {
        // Path 1: 常规 A* 步进
        tentative_g = g_score[ck] + dcost[i];
        best_parent = current.cell;
      }

      auto it = g_score.find(nk);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nk] = tentative_g;
        parent[nk] = best_parent;
        double f = tentative_g + weight_ * heuristic(neighbor, goal_cell);
        open.push({neighbor, tentative_g, f});
      }
    }
  }

  last_failure_reason_ = "无可达路径";
  return {};  // 搜索失败
}

// ============================================================================
// 路径插值
// ============================================================================

std::vector<WorldPoint> interpolate_path(const std::vector<WorldPoint>& path,
                                         double max_spacing) {
  if (path.size() < 2 || max_spacing <= 0.0) {
    return path;
  }

  std::vector<WorldPoint> result;
  result.push_back(path.front());

  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    int segments = std::max(1, static_cast<int>(std::ceil(dist / max_spacing)));
    for (int s = 1; s <= segments; ++s) {
      double t = static_cast<double>(s) / segments;
      result.push_back(
          {path[i - 1].x + t * dx, path[i - 1].y + t * dy});
    }
  }
  return result;
}

}  // namespace path_planner
