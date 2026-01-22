// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__VEHICLE_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__VEHICLE_ASSERTIONS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a vehicle movement assertion.
 */
struct MovementResult
{
  /// Whether the vehicle moved the required distance
  bool success;

  /// Actual distance moved (meters)
  double distance_moved;

  /// Starting position
  geometry_msgs::msg::Point start_position;

  /// Ending position
  geometry_msgs::msg::Point end_position;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Result of a vehicle velocity assertion.
 */
struct VelocityResult
{
  /// Whether the target velocity was reached
  bool success;

  /// Measured velocity (m/s)
  double measured_velocity;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a vehicle moved at least min_distance meters.
 *
 * Monitors the vehicle's odometry topic, sends a forward velocity command,
 * and verifies that the vehicle moved the required distance.
 *
 * @param node ROS 2 node for subscriptions/publishers
 * @param vehicle_id Vehicle namespace (e.g., "yt222_01")
 * @param min_distance Minimum distance to move (meters)
 * @param velocity Forward velocity to command (m/s, default 1.0)
 * @param timeout Maximum time to wait
 * @return MovementResult with success status and details
 */
MovementResult assertVehicleMoved(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double min_distance,
  double velocity = 1.0,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that a vehicle moved at least min_distance meters (custom topics).
 *
 * Overload that accepts custom odometry and cmd_vel topic names.
 *
 * @param node ROS 2 node for subscriptions/publishers
 * @param vehicle_id Vehicle namespace (empty string for no namespace)
 * @param min_distance Minimum distance to move (meters)
 * @param timeout Maximum time to wait
 * @param odom_topic Custom odometry topic name
 * @param cmd_vel_topic Custom velocity command topic name
 * @param velocity Forward velocity to command (m/s)
 * @return MovementResult with success status and details
 */
MovementResult assertVehicleMoved(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double min_distance,
  std::chrono::seconds timeout,
  const std::string & odom_topic,
  const std::string & cmd_vel_topic,
  double velocity = 1.0);

/**
 * @brief Assert that a vehicle is stationary.
 *
 * Monitors odometry and verifies velocity stays below threshold
 * for the specified duration.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param velocity_threshold Maximum velocity to consider stationary (m/s)
 * @param duration How long the vehicle must remain stationary
 * @return true if vehicle remained stationary
 */
bool assertVehicleStationary(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double velocity_threshold = 0.01,
  std::chrono::seconds duration = std::chrono::seconds(2));

/**
 * @brief Assert that a vehicle is stationary (custom topic).
 *
 * Overload that accepts a custom odometry topic name.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace (empty for no namespace)
 * @param velocity_threshold Maximum velocity to consider stationary (m/s)
 * @param duration How long the vehicle must remain stationary
 * @param odom_topic Custom odometry topic name
 * @return true if vehicle remained stationary
 */
bool assertVehicleStationary(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double velocity_threshold,
  std::chrono::seconds duration,
  const std::string & odom_topic);

/**
 * @brief Assert that a vehicle reaches the target velocity.
 *
 * Monitors odometry and verifies the vehicle reaches the commanded
 * velocity within tolerance.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param target_velocity Target velocity (m/s)
 * @param tolerance Acceptable deviation from target (m/s)
 * @param timeout Maximum time to wait
 * @return VelocityResult with success status and measured velocity
 */
VelocityResult assertVehicleVelocity(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double target_velocity,
  double tolerance = 0.1,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that a vehicle is within a bounding region.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param min_bounds Minimum corner of bounding box
 * @param max_bounds Maximum corner of bounding box
 * @param timeout Time to wait for odometry
 * @return true if vehicle is within bounds
 */
bool assertVehicleInRegion(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const geometry_msgs::msg::Point & min_bounds,
  const geometry_msgs::msg::Point & max_bounds,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that a vehicle's yaw orientation is within tolerance.
 *
 * @param node ROS 2 node
 * @param vehicle_id Vehicle namespace
 * @param expected_yaw Expected yaw angle (radians)
 * @param tolerance_rad Acceptable deviation (radians)
 * @param timeout Time to wait for odometry
 * @return true if orientation is within tolerance
 */
bool assertVehicleOrientation(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double expected_yaw,
  double tolerance_rad = 0.1,
  std::chrono::seconds timeout = std::chrono::seconds(5));

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__VEHICLE_ASSERTIONS_HPP_
