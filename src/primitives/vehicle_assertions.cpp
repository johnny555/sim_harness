// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/vehicle_assertions.hpp"

#include <cmath>
#include <sstream>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace sim_harness::primitives
{

namespace
{

double distance3d(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::sqrt(
    std::pow(a.x - b.x, 2) +
    std::pow(a.y - b.y, 2) +
    std::pow(a.z - b.z, 2));
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

MovementResult assertVehicleMoved(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double min_distance,
  double velocity,
  std::chrono::seconds timeout)
{
  MovementResult result;
  result.success = false;
  result.distance_moved = 0.0;

  // Create executor for spinning
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to odometry
  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&latest_odom](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom = msg;
    });

  // Create velocity publisher
  std::string cmd_topic = "/" + vehicle_id + "/steering_controller/reference";
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);

  // Wait for initial odometry
  auto start_time = std::chrono::steady_clock::now();
  while (!latest_odom &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5))
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_odom) {
    result.details = "No odometry received on " + odom_topic;
    return result;
  }

  result.start_position = latest_odom->pose.pose.position;

  // Send velocity command
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = velocity;
  cmd.twist.angular.z = 0.0;

  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < timeout) {
    // Publish command
    cmd.header.stamp = node->now();
    cmd_pub->publish(cmd);

    // Spin to receive odometry
    executor.spin_some(std::chrono::milliseconds(50));

    if (latest_odom) {
      result.end_position = latest_odom->pose.pose.position;
      result.distance_moved = distance3d(result.start_position, result.end_position);

      if (result.distance_moved >= min_distance) {
        result.success = true;
        break;
      }
    }
  }

  // Stop the vehicle
  cmd.twist.linear.x = 0.0;
  cmd.header.stamp = node->now();
  cmd_pub->publish(cmd);

  std::stringstream ss;
  ss << "Vehicle moved " << result.distance_moved << "m";
  if (result.success) {
    ss << " (required: " << min_distance << "m)";
  } else {
    ss << " but required " << min_distance << "m";
  }
  result.details = ss.str();

  return result;
}

bool assertVehicleStationary(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double velocity_threshold,
  std::chrono::seconds duration)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  double max_velocity_seen = 0.0;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&max_velocity_seen](const nav_msgs::msg::Odometry::SharedPtr msg) {
      double vel = std::sqrt(
        std::pow(msg->twist.twist.linear.x, 2) +
        std::pow(msg->twist.twist.linear.y, 2));
      max_velocity_seen = std::max(max_velocity_seen, vel);
    });

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < duration) {
    executor.spin_some(std::chrono::milliseconds(10));

    if (max_velocity_seen > velocity_threshold) {
      return false;
    }
  }

  return max_velocity_seen <= velocity_threshold;
}

VelocityResult assertVehicleVelocity(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double target_velocity,
  double tolerance,
  std::chrono::seconds timeout)
{
  VelocityResult result;
  result.success = false;
  result.measured_velocity = 0.0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  double latest_velocity = 0.0;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&latest_velocity](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_velocity = std::sqrt(
        std::pow(msg->twist.twist.linear.x, 2) +
        std::pow(msg->twist.twist.linear.y, 2));
    });

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));

    result.measured_velocity = latest_velocity;
    if (std::abs(latest_velocity - target_velocity) <= tolerance) {
      result.success = true;
      break;
    }
  }

  std::stringstream ss;
  ss << "Measured velocity: " << result.measured_velocity << " m/s";
  ss << " (target: " << target_velocity << " +/- " << tolerance << " m/s)";
  result.details = ss.str();

  return result;
}

bool assertVehicleInRegion(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  const geometry_msgs::msg::Point & min_bounds,
  const geometry_msgs::msg::Point & max_bounds,
  std::chrono::seconds timeout)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  geometry_msgs::msg::Point position;
  bool received = false;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&position, &received](const nav_msgs::msg::Odometry::SharedPtr msg) {
      position = msg->pose.pose.position;
      received = true;
    });

  auto start_time = std::chrono::steady_clock::now();
  while (!received && std::chrono::steady_clock::now() - start_time < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!received) {
    return false;
  }

  return position.x >= min_bounds.x && position.x <= max_bounds.x &&
         position.y >= min_bounds.y && position.y <= max_bounds.y &&
         position.z >= min_bounds.z && position.z <= max_bounds.z;
}

bool assertVehicleOrientation(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double expected_yaw,
  double tolerance_rad,
  std::chrono::seconds timeout)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::string odom_topic = "/" + vehicle_id + "/steering_controller/odometry";
  double current_yaw = 0.0;
  bool received = false;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&current_yaw, &received](const nav_msgs::msg::Odometry::SharedPtr msg) {
      current_yaw = getYawFromQuaternion(msg->pose.pose.orientation);
      received = true;
    });

  auto start_time = std::chrono::steady_clock::now();
  while (!received && std::chrono::steady_clock::now() - start_time < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!received) {
    return false;
  }

  // Normalize angle difference to [-pi, pi]
  double diff = current_yaw - expected_yaw;
  while (diff > M_PI) {diff -= 2 * M_PI;}
  while (diff < -M_PI) {diff += 2 * M_PI;}

  return std::abs(diff) <= tolerance_rad;
}

MovementResult assertVehicleMoved(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double min_distance,
  std::chrono::seconds timeout,
  const std::string & odom_topic,
  const std::string & cmd_vel_topic,
  double velocity)
{
  (void)vehicle_id;  // Not used in this overload

  MovementResult result;
  result.success = false;
  result.distance_moved = 0.0;

  // Create executor for spinning
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to odometry
  nav_msgs::msg::Odometry::SharedPtr latest_odom;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&latest_odom](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom = msg;
    });

  // Create velocity publisher (use Twist for TurtleBot3 compatibility)
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  // Wait for initial odometry
  auto start_time = std::chrono::steady_clock::now();
  while (!latest_odom &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5))
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_odom) {
    result.details = "No odometry received on " + odom_topic;
    return result;
  }

  result.start_position = latest_odom->pose.pose.position;

  // Send velocity command
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = velocity;
  cmd.angular.z = 0.0;

  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < timeout) {
    // Publish command
    cmd_pub->publish(cmd);

    // Spin to receive odometry
    executor.spin_some(std::chrono::milliseconds(50));

    if (latest_odom) {
      result.end_position = latest_odom->pose.pose.position;
      result.distance_moved = distance3d(result.start_position, result.end_position);

      if (result.distance_moved >= min_distance) {
        result.success = true;
        break;
      }
    }
  }

  // Stop the vehicle
  cmd.linear.x = 0.0;
  cmd_pub->publish(cmd);

  std::stringstream ss;
  ss << "Vehicle moved " << result.distance_moved << "m";
  if (result.success) {
    ss << " (required: " << min_distance << "m)";
  } else {
    ss << " but required " << min_distance << "m";
  }
  result.details = ss.str();

  return result;
}

bool assertVehicleStationary(
  rclcpp::Node::SharedPtr node,
  const std::string & vehicle_id,
  double velocity_threshold,
  std::chrono::seconds duration,
  const std::string & odom_topic)
{
  (void)vehicle_id;  // Not used in this overload

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  double max_velocity_seen = 0.0;

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    [&max_velocity_seen](const nav_msgs::msg::Odometry::SharedPtr msg) {
      double vel = std::sqrt(
        std::pow(msg->twist.twist.linear.x, 2) +
        std::pow(msg->twist.twist.linear.y, 2));
      max_velocity_seen = std::max(max_velocity_seen, vel);
    });

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < duration) {
    executor.spin_some(std::chrono::milliseconds(10));

    if (max_velocity_seen > velocity_threshold) {
      return false;
    }
  }

  return max_velocity_seen <= velocity_threshold;
}

}  // namespace sim_harness::primitives
