#include "ydlidar_ros_driver/l1ang_local_planner.h"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace l1ang_planner {

L1angLocalPlanner::L1angLocalPlanner()
    : is_initialized_(false),
      tf_(NULL),
      costmap_ros_(NULL),
      stop_dist_(0.15),
      scan_span_rad_(20.0 * kPi / 180.0),
      xy_goal_tol_(0.20),
      yaw_goal_tol_(0.25),
      latch_xy_goal_tol_(true),
      scan_topic_("scan"),
      path_index_(0),
      goal_yaw_(0.0),
      has_goal_(false),
      has_goal_yaw_(false),
      xy_reached_(false),
      obstacle_dist_(std::numeric_limits<double>::infinity()) {}

L1angLocalPlanner::~L1angLocalPlanner() {}

void L1angLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  if (is_initialized_) {
    ROS_WARN("L1angLocalPlanner 已经初始化过了，忽略重复调用");
    return;
  }
  if (tf == NULL || costmap_ros == NULL) {
    ROS_ERROR("L1angLocalPlanner: tf 或 costmap 为空，初始化失败");
    return;
  }
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // move_base 传入的 name 可能是 "L1angLocalPlanner"，也可能是 "包名/L1angLocalPlanner"，
  // 统一取最后一段，保证参数命名空间恒为 ~/<类名>。
  std::string short_name = name;
  const std::size_t slash = short_name.rfind('/');
  if (slash != std::string::npos) {
    short_name = short_name.substr(slash + 1);
  }
  ros::NodeHandle pn("~/" + short_name);
  ros::NodeHandle nh;

  pn.param("dp_threshold", prm_.dp_threshold, prm_.dp_threshold);
  pn.param("yaw_kp", prm_.yaw_kp, prm_.yaw_kp);
  pn.param("dist_kp", prm_.dist_kp, prm_.dist_kp);
  pn.param("max_v", prm_.max_v, prm_.max_v);
  pn.param("max_w", prm_.max_w, prm_.max_w);
  pn.param("yaw_tol", prm_.yaw_tol, prm_.yaw_tol);
  pn.param("dist_tol", prm_.dist_tol, prm_.dist_tol);

  pn.param("stop_dist", stop_dist_, stop_dist_);
  pn.param("scan_topic", scan_topic_, scan_topic_);
  double scan_span_deg = scan_span_rad_ * 180.0 / kPi;
  pn.param("scan_span_deg", scan_span_deg, scan_span_deg);
  scan_span_rad_ = scan_span_deg * kPi / 180.0;

  pn.param("xy_goal_tolerance", xy_goal_tol_, xy_goal_tol_);
  pn.param("yaw_goal_tolerance", yaw_goal_tol_, yaw_goal_tol_);
  pn.param("latch_xy_goal_tolerance", latch_xy_goal_tol_, latch_xy_goal_tol_);

  path_pub_ = pn.advertise<nav_msgs::Path>("simplified_path", 1, true);
  scan_sub_ = nh.subscribe(scan_topic_, 1, &L1angLocalPlanner::scanCallback, this);

  is_initialized_ = true;
  ROS_INFO(
      "L1angLocalPlanner 初始化完成: dp_threshold=%.3f max_v=%.2f max_w=%.2f "
      "yaw_tol=%.2f dist_tol=%.2f stop_dist=%.2f scan=%s",
      prm_.dp_threshold, prm_.max_v, prm_.max_w, prm_.yaw_tol, prm_.dist_tol, stop_dist_,
      scan_topic_.c_str());
}

bool L1angLocalPlanner::isInitialized() { return is_initialized_; }

void L1angLocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (msg->ranges.empty()) {
    return;
  }

  const double range_max =
      (std::isfinite(msg->range_max) && msg->range_max > msg->range_min)
          ? msg->range_max
          : std::numeric_limits<double>::infinity();
  double nearest = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle =
        normalizeAngle(msg->angle_min + static_cast<double>(i) * msg->angle_increment);
    if (std::fabs(angle) > scan_span_rad_) {
      continue;
    }
    const double r = msg->ranges[i];
    if (!std::isfinite(r) || r <= msg->range_min || r >= range_max) {
      continue;
    }
    nearest = std::min(nearest, r);
  }

  boost::mutex::scoped_lock lock(scan_mutex_);
  obstacle_dist_ = nearest;
  last_scan_time_ = ros::Time::now();
}

double L1angLocalPlanner::obstacleDistance() {
  boost::mutex::scoped_lock lock(scan_mutex_);
  if (last_scan_time_.isZero() ||
      (ros::Time::now() - last_scan_time_).toSec() > 1.0) {
    return std::numeric_limits<double>::infinity();  // 没有激光数据时不做急停
  }
  return obstacle_dist_;
}

bool L1angLocalPlanner::getRobotPose(Point& robot, double& robot_yaw) {
  geometry_msgs::PoseStamped global_pose;
  if (costmap_ros_ == NULL || !costmap_ros_->getRobotPose(global_pose)) {
    return false;
  }

  std::string target_frame;
  {
    boost::mutex::scoped_lock lock(mutex_);
    target_frame = plan_frame_;
  }
  if (target_frame.empty()) {
    target_frame = "map";
  }

  if (global_pose.header.frame_id != target_frame) {
    try {
      geometry_msgs::PoseStamped transformed;
      tf_->transform(global_pose, transformed, target_frame, ros::Duration(0.1));
      global_pose = transformed;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "L1angLocalPlanner: 位姿变换到 %s 失败: %s",
                        target_frame.c_str(), ex.what());
      return false;
    }
  }

  robot.x = global_pose.pose.position.x;
  robot.y = global_pose.pose.position.y;
  robot_yaw = tf2::getYaw(global_pose.pose.orientation);
  return true;
}

void L1angLocalPlanner::publishPath(const std::vector<Point>& points) {
  if (!path_pub_) {
    return;
  }

  nav_msgs::Path msg;
  {
    boost::mutex::scoped_lock lock(mutex_);
    msg.header.frame_id = plan_frame_.empty() ? std::string("map") : plan_frame_;
  }
  msg.header.stamp = ros::Time::now();
  msg.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::PoseStamped ps;
    ps.header = msg.header;
    ps.pose.position.x = points[i].x;
    ps.pose.position.y = points[i].y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    msg.poses.push_back(ps);
  }
  path_pub_.publish(msg);
}

bool L1angLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!is_initialized_) {
    ROS_ERROR("L1angLocalPlanner: 未初始化就收到了全局路径");
    return false;
  }
  if (plan.empty()) {
    ROS_WARN("L1angLocalPlanner: 收到空路径");
    return false;
  }

  std::vector<Point> raw;
  raw.reserve(plan.size());
  for (std::size_t i = 0; i < plan.size(); ++i) {
    raw.push_back(Point(plan[i].pose.position.x, plan[i].pose.position.y));
  }

  std::vector<Point> simplified;
  douglasPeucker(raw, prm_.dp_threshold, simplified);
  if (simplified.empty()) {
    simplified = raw;
  }

  // 全局路径会被高频重规划（planner_frequency），新路径的起点就是车当前位置。
  // 让索引重新锚定到离车最近的拐点，避免索引漂移导致来回摆动。
  std::size_t new_index = 0;
  Point robot;
  double robot_yaw = 0.0;
  if (getRobotPose(robot, robot_yaw)) {
    new_index = nearestIndex(simplified, robot);
  }

  const std::string frame =
      plan.front().header.frame_id.empty() ? std::string("map") : plan.front().header.frame_id;
  const geometry_msgs::Quaternion& q = plan.back().pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

  {
    boost::mutex::scoped_lock lock(mutex_);
    path_ = simplified;
    path_index_ = new_index;
    plan_frame_ = frame;
    goal_ = simplified.back();
    has_goal_ = true;
    has_goal_yaw_ = q_norm > 1e-6;
    goal_yaw_ = has_goal_yaw_ ? tf2::getYaw(q) : 0.0;
    xy_reached_ = false;
  }

  publishPath(simplified);
  ROS_INFO("L1angLocalPlanner: 原始 %zu 点 -> 抽稀 %zu 点, 起点索引 %zu", plan.size(),
           simplified.size(), new_index);
  return true;
}

bool L1angLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  cmd_vel = geometry_msgs::Twist();
  if (!is_initialized_) {
    ROS_ERROR("L1angLocalPlanner: 未初始化");
    return false;
  }

  Point robot;
  double robot_yaw = 0.0;
  if (!getRobotPose(robot, robot_yaw)) {
    ROS_WARN_THROTTLE(1.0, "L1angLocalPlanner: 暂时拿不到机器人位姿");
    return false;
  }

  std::vector<Point> path;
  std::size_t index;
  double goal_yaw;
  bool has_goal;
  bool has_goal_yaw;
  {
    boost::mutex::scoped_lock lock(mutex_);
    path = path_;
    index = path_index_;
    goal_yaw = goal_yaw_;
    has_goal = has_goal_;
    has_goal_yaw = has_goal_yaw_;
  }

  if (path.empty()) {
    ROS_WARN_THROTTLE(2.0, "L1angLocalPlanner: 还没有收到全局路径");
    return true;  // 零速度等待
  }

  // 折线已走完：原地对准目标朝向，便于 isGoalReached() 判定成功
  if (index >= path.size()) {
    if (has_goal && has_goal_yaw) {
      const double yaw_err = normalizeAngle(goal_yaw - robot_yaw);
      if (std::fabs(yaw_err) > yaw_goal_tol_) {
        cmd_vel.angular.z = computeYawAlign(yaw_err, prm_).w;
      }
    }
    return true;
  }

  const Point target = path[index];
  const double dist = std::hypot(target.x - robot.x, target.y - robot.y);

  if (dist < prm_.dist_tol) {
    boost::mutex::scoped_lock lock(mutex_);
    if (path_index_ == index && path_index_ < path_.size()) {
      ++path_index_;
      ROS_INFO("L1angLocalPlanner: 到达拐点 %zu/%zu", path_index_ - 1, path_.size() - 1);
    }
    return true;  // 本周期停车，下个周期朝下一个拐点走
  }

  const double obstacle = obstacleDistance();
  if (obstacle < stop_dist_) {
    ROS_WARN_THROTTLE(1.0, "L1angLocalPlanner: 急停，前方障碍 %.3f m", obstacle);
    return true;
  }

  const VelocityCmd cmd = computeVelocity(robot, robot_yaw, target, prm_);
  cmd_vel.linear.x = cmd.v;
  cmd_vel.angular.z = cmd.w;
  return true;
}

bool L1angLocalPlanner::isGoalReached() {
  if (!is_initialized_) {
    return false;
  }

  bool has_goal;
  bool has_goal_yaw;
  Point goal;
  double goal_yaw;
  {
    boost::mutex::scoped_lock lock(mutex_);
    has_goal = has_goal_;
    has_goal_yaw = has_goal_yaw_;
    goal = goal_;
    goal_yaw = goal_yaw_;
  }
  if (!has_goal) {
    return false;
  }

  Point robot;
  double robot_yaw = 0.0;
  if (!getRobotPose(robot, robot_yaw)) {
    return false;
  }

  const double dist = std::hypot(goal.x - robot.x, goal.y - robot.y);
  if (latch_xy_goal_tol_) {
    boost::mutex::scoped_lock lock(mutex_);
    if (dist <= xy_goal_tol_) {
      xy_reached_ = true;
    }
    if (!xy_reached_) {
      return false;
    }
  } else if (dist > xy_goal_tol_) {
    return false;
  }

  if (!has_goal_yaw) {
    return true;  // 目标姿态未指定，只看位置
  }
  return std::fabs(normalizeAngle(goal_yaw - robot_yaw)) <= yaw_goal_tol_;
}

}  // namespace l1ang_planner

PLUGINLIB_EXPORT_CLASS(l1ang_planner::L1angLocalPlanner, nav_core::BaseLocalPlanner)
