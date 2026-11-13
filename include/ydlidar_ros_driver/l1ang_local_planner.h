#ifndef YDLIDAR_ROS_DRIVER_L1ANG_LOCAL_PLANNER_H_
#define YDLIDAR_ROS_DRIVER_L1ANG_LOCAL_PLANNER_H_

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>

#include <boost/thread/mutex.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include "ydlidar_ros_driver/l1ang_path_utils.h"

namespace l1ang_planner {

// move_base 的局部规划插件：跟踪 global_planner 给出的折线路径。
// 算法与原 scripts/l1ang_planner.py 一致（DP 抽稀 + 先转向再直行），
// 区别是速度由 move_base 通过 computeVelocityCommands() 同步索取，不再自己发布 /cmd_vel。
class L1angLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  L1angLocalPlanner();
  ~L1angLocalPlanner() override;

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

  bool isInitialized();

 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool getRobotPose(Point& robot, double& robot_yaw);
  double obstacleDistance();
  void publishPath(const std::vector<Point>& points);

  bool is_initialized_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  ros::Publisher path_pub_;
  ros::Subscriber scan_sub_;

  PlannerParams prm_;
  double stop_dist_;
  double scan_span_rad_;
  double xy_goal_tol_;
  double yaw_goal_tol_;
  bool latch_xy_goal_tol_;
  std::string scan_topic_;

  // setPlan() 在 move_base 的规划线程里调用，computeVelocityCommands() 在控制线程里调用
  boost::mutex mutex_;
  std::vector<Point> path_;  // 抽稀后的折线
  std::size_t path_index_;   // 当前跟踪的拐点索引
  std::string plan_frame_;   // 全局路径所在坐标系
  Point goal_;
  double goal_yaw_;
  bool has_goal_;
  bool has_goal_yaw_;
  bool xy_reached_;  // latch_xy_goal_tolerance 用

  boost::mutex scan_mutex_;
  double obstacle_dist_;
  ros::Time last_scan_time_;
};

}  // namespace l1ang_planner

#endif  // YDLIDAR_ROS_DRIVER_L1ANG_LOCAL_PLANNER_H_
