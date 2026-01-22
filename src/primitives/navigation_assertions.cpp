// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/navigation_assertions.hpp"

#include <cmath>
#include <sstream>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace sim_harness::primitives
{

namespace
{

double distance2d(double x1, double y1, double x2, double y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2 * M_PI;}
  while (angle < -M_PI) {angle += 2 * M_PI;}
  return angle;
}

}  // namespace

NavigationResult assertReachesGoal(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const geometry_msgs::msg::PoseStamped & goal,
  double position_tolerance,
  double orientation_tolerance,
  std::chrono::seconds timeout)
{
  NavigationResult result;
  result.success = false;
  result.final_distance_to_goal = std::numeric_limits<double>::max();
  result.time_taken = std::chrono::milliseconds(0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&latest_odom](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom = msg;
    });

  double goal_yaw = getYawFromQuaternion(goal.pose.orientation);
  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(50));

    if (latest_odom) {
      double dist = distance2d(
        latest_odom->pose.pose.position.x, latest_odom->pose.pose.position.y,
        goal.pose.position.x, goal.pose.position.y);

      result.final_distance_to_goal = dist;

      double current_yaw = getYawFromQuaternion(latest_odom->pose.pose.orientation);
      double yaw_diff = std::abs(normalizeAngle(current_yaw - goal_yaw));

      if (dist <= position_tolerance && yaw_diff <= orientation_tolerance) {
        result.success = true;
        result.time_taken = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start);
        break;
      }
    }
  }

  std::stringstream ss;
  ss << "Final distance to goal: " << result.final_distance_to_goal << "m";
  ss << " (tolerance: " << position_tolerance << "m)";
  if (result.success) {
    ss << " - reached in " << result.time_taken.count() << "ms";
  }
  result.details = ss.str();

  return result;
}

bool assertFollowsPath(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const nav_msgs::msg::Path & expected_path,
  double max_deviation,
  std::chrono::seconds timeout)
{
  if (expected_path.poses.empty()) {
    return true;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&latest_odom](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom = msg;
    });

  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(50));

    if (latest_odom) {
      // Find minimum distance to any point on path
      double min_dist = std::numeric_limits<double>::max();
      for (const auto & pose : expected_path.poses) {
        double dist = distance2d(
          latest_odom->pose.pose.position.x, latest_odom->pose.pose.position.y,
          pose.pose.position.x, pose.pose.position.y);
        min_dist = std::min(min_dist, dist);
      }

      if (min_dist > max_deviation) {
        return false;
      }
    }
  }

  return true;
}

NavigationResult assertNavigationActionSucceeds(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal,
  std::chrono::seconds timeout)
{
  NavigationResult result;
  result.success = false;
  result.final_distance_to_goal = std::numeric_limits<double>::max();

  // This is a simplified implementation
  // Full implementation would use action client to navigate_to_pose
  (void)node;
  (void)action_name;
  (void)goal;
  (void)timeout;

  result.details = "Navigation action client not yet implemented";
  return result;
}

bool assertCostmapContainsObstacle(
  rclcpp::Node::SharedPtr node,
  const std::string & costmap_topic,
  const geometry_msgs::msg::Point & obstacle_position,
  double search_radius)
{
  // Simplified implementation
  (void)node;
  (void)costmap_topic;
  (void)obstacle_position;
  (void)search_radius;

  // Full implementation would subscribe to costmap and check cell values
  return false;
}

}  // namespace sim_harness::primitives
