// 与 steps/l1ang_planner.py 中算法等价的纯 C++ 实现（不依赖 ROS，便于单独测试）。
//
// 对应关系：
//   douglasPeucker()  <- dp_path() 内部的 dp()
//   nearestIndex()    <- 折线路径上找最近拐点（用于重规划后的索引重锚定）
//   computeVelocity() <- follow_path() 的“先转向、再直行”
#ifndef YDLIDAR_ROS_DRIVER_L1ANG_PATH_UTILS_H_
#define YDLIDAR_ROS_DRIVER_L1ANG_PATH_UTILS_H_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace l1ang_planner {

const double kPi = 3.14159265358979323846;

struct Point {
  double x;
  double y;

  Point() : x(0.0), y(0.0) {}
  Point(double px, double py) : x(px), y(py) {}
};

struct PlannerParams {
  double dp_threshold;  // Douglas-Peucker 抽稀阈值 (m)
  double yaw_kp;        // 转向比例系数
  double dist_kp;       // 前进比例系数
  double max_v;         // 最大线速度 (m/s)
  double max_w;         // 最大角速度 (rad/s)
  double yaw_tol;       // 朝向容差 (rad)
  double dist_tol;      // 到达拐点容差 (m)

  PlannerParams()
      : dp_threshold(0.15),
        yaw_kp(1.5),
        dist_kp(1.0),
        max_v(0.25),
        max_w(1.5),
        yaw_tol(0.25),
        dist_tol(0.10) {}
};

struct VelocityCmd {
  double v;
  double w;

  VelocityCmd() : v(0.0), w(0.0) {}
  VelocityCmd(double linear, double angular) : v(linear), w(angular) {}
};

inline double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

inline double clampValue(double v, double lim) {
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

// 点到首尾连线的距离，与 Python 版 get_point_dis() 完全一致
inline double pointLineDistance(const Point& p, const Point& head, const Point& tail) {
  const double dx = tail.x - head.x;
  const double dy = tail.y - head.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) {
    return std::hypot(p.x - head.x, p.y - head.y);
  }
  const double area2 = std::fabs(dx * (p.y - head.y) - dy * (p.x - head.x));
  return area2 / len;
}

namespace detail {

inline void dpRecursive(const std::vector<Point>& pts, std::size_t first, std::size_t last,
                        double epsilon, std::vector<char>& keep) {
  if (last <= first + 1) {
    return;
  }

  double max_dist = -1.0;
  std::size_t index = first;
  for (std::size_t i = first + 1; i < last; ++i) {
    const double d = pointLineDistance(pts[i], pts[first], pts[last]);
    if (d > max_dist) {
      max_dist = d;
      index = i;
    }
  }

  if (max_dist > epsilon) {
    keep[index] = 1;
    dpRecursive(pts, first, index, epsilon, keep);
    dpRecursive(pts, index, last, epsilon, keep);
  }
}

}  // namespace detail

// Douglas-Peucker 抽稀：把平滑的全局路径变成少量拐点组成的折线
inline void douglasPeucker(const std::vector<Point>& points, double epsilon,
                           std::vector<Point>& simplified) {
  simplified.clear();
  if (points.empty()) {
    return;
  }
  if (points.size() <= 2) {
    simplified = points;
    return;
  }

  std::vector<char> keep(points.size(), 0);
  keep.front() = 1;
  keep.back() = 1;
  detail::dpRecursive(points, 0, points.size() - 1, epsilon, keep);

  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) {
      simplified.push_back(points[i]);
    }
  }
}

inline std::size_t nearestIndex(const std::vector<Point>& points, const Point& p) {
  if (points.empty()) {
    return 0;
  }
  std::size_t best = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double d = std::hypot(points[i].x - p.x, points[i].y - p.y);
    if (d < best_dist) {
      best_dist = d;
      best = i;
    }
  }
  return best;
}

// 朝向没对 → 原地转；朝向对 → 直走
inline VelocityCmd computeVelocity(const Point& robot, double robot_yaw, const Point& target,
                                   const PlannerParams& prm) {
  VelocityCmd cmd;
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_err = normalizeAngle(std::atan2(dy, dx) - robot_yaw);

  if (std::fabs(yaw_err) > prm.yaw_tol) {
    cmd.w = clampValue(prm.yaw_kp * yaw_err, prm.max_w);
  } else {
    cmd.v = clampValue(prm.dist_kp * dist, prm.max_v);
  }
  return cmd;
}

// 路径走完后原地对准目标朝向
inline VelocityCmd computeYawAlign(double yaw_err, const PlannerParams& prm) {
  VelocityCmd cmd;
  cmd.w = clampValue(prm.yaw_kp * yaw_err, prm.max_w);
  return cmd;
}

}  // namespace l1ang_planner

#endif  // YDLIDAR_ROS_DRIVER_L1ANG_PATH_UTILS_H_
