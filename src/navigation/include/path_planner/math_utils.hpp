#ifndef PATH_PLANNER__MATH_UTILS_HPP_
#define PATH_PLANNER__MATH_UTILS_HPP_

#include <cmath>

namespace path_planner {

/// 将角度归一化到 [-π, π) 范围。
///
/// Args:
///   angle: 任意弧度值。
///
/// Returns:
///   归一化后的弧度值。
inline double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// 浮点值近似相等判断。
///
/// Args:
///   a: 第一个浮点数。
///   b: 第二个浮点数。
///   epsilon: 容差。
///
/// Returns:
///   两值之差的绝对值小于容差时返回 true。
inline bool approx_equal(double a, double b, double epsilon = 1e-9) {
  return std::abs(a - b) < epsilon;
}

}  // namespace path_planner

#endif  // PATH_PLANNER__MATH_UTILS_HPP_
