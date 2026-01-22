// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__NAVIGATION_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__NAVIGATION_ASSERTIONS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a navigation assertion.
 */
struct NavigationResult
{
  /// Whether navigation succeeded
  bool success;

  /// Final distance to goal (meters)
  double final_distance_to_goal;

  /// Time taken to reach goal
  std::chrono::milliseconds time_taken;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a vehicle reaches a goal pose.
 *
 * Monitors odometry and checks if vehicle arrives within tolerance.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param goal Goal pose
 * @param position_tolerance Position tolerance (meters)
 * @param orientation_tolerance Orientation tolerance (radians)
 * @param timeout Maximum time to wait
 * @return NavigationResult
 */
NavigationResult assertReachesGoal(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const geometry_msgs::msg::PoseStamped & goal,
  double position_tolerance = 0.5,
  double orientation_tolerance = 0.1,
  std::chrono::seconds timeout = std::chrono::seconds(60));

/**
 * @brief Assert that a vehicle follows a path within a corridor.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param expected_path Expected path
 * @param max_deviation Maximum allowed deviation from path (meters)
 * @param timeout Maximum time to monitor
 * @return true if vehicle stayed within corridor
 */
bool assertFollowsPath(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const nav_msgs::msg::Path & expected_path,
  double max_deviation = 1.0,
  std::chrono::seconds timeout = std::chrono::seconds(60));

/**
 * @brief Assert that a Nav2 navigate_to_pose action succeeds.
 *
 * @param node ROS 2 node
 * @param action_name Navigation action name
 * @param goal Goal pose
 * @param timeout Maximum time to wait
 * @return NavigationResult
 */
NavigationResult assertNavigationActionSucceeds(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal,
  std::chrono::seconds timeout = std::chrono::seconds(120));

/**
 * @brief Assert that a costmap contains obstacles at expected positions.
 *
 * @param node ROS 2 node
 * @param costmap_topic Costmap topic
 * @param obstacle_position Expected obstacle position
 * @param search_radius Search radius around position
 * @return true if high-cost cells found near position
 */
bool assertCostmapContainsObstacle(
  rclcpp::Node::SharedPtr node,
  const std::string & costmap_topic,
  const geometry_msgs::msg::Point & obstacle_position,
  double search_radius = 0.5);

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__NAVIGATION_ASSERTIONS_HPP_
