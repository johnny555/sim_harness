// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__SENSOR_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__SENSOR_ASSERTIONS_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a sensor data validation.
 */
struct SensorDataResult
{
  /// Whether the sensor data is valid
  bool valid;

  /// Number of messages received during sampling
  size_t message_count;

  /// Measured publish rate (Hz)
  double publish_rate_hz;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a sensor is publishing at the expected rate.
 *
 * @param node ROS 2 node
 * @param topic Topic to monitor
 * @param expected_rate_hz Expected publish rate
 * @param tolerance_percent Acceptable deviation (percent)
 * @param sample_duration How long to sample
 * @return SensorDataResult with publish rate info
 */
SensorDataResult assertSensorPublishing(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double expected_rate_hz,
  double tolerance_percent = 10.0,
  std::chrono::seconds sample_duration = std::chrono::seconds(5));

/**
 * @brief Assert that LIDAR data is valid.
 *
 * Checks:
 * - Messages are being received
 * - Point cloud has minimum number of points
 * - Range values are within bounds (no NaN, within sensor limits)
 *
 * @param node ROS 2 node
 * @param topic LIDAR topic
 * @param min_range Minimum valid range (meters)
 * @param max_range Maximum valid range (meters)
 * @param min_points Minimum number of points expected
 * @param timeout Time to wait for data
 * @return SensorDataResult
 */
SensorDataResult assertLidarValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double min_range = 0.1,
  double max_range = 100.0,
  size_t min_points = 100,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that GPS data is valid and within a region.
 *
 * Checks:
 * - Messages are being received
 * - Coordinates are within specified bounds
 * - Fix status is valid
 * - No NaN values in coordinates
 *
 * @param node ROS 2 node
 * @param topic GPS topic
 * @param min_lat Minimum latitude (degrees)
 * @param max_lat Maximum latitude (degrees)
 * @param min_lon Minimum longitude (degrees)
 * @param max_lon Maximum longitude (degrees)
 * @param timeout Time to wait for data
 * @return SensorDataResult
 */
SensorDataResult assertGpsValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double min_lat,
  double max_lat,
  double min_lon,
  double max_lon,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that IMU data is valid.
 *
 * Checks:
 * - Messages are being received
 * - Acceleration values are reasonable (not extreme)
 * - Angular velocity values are reasonable
 * - No NaN values
 *
 * @param node ROS 2 node
 * @param topic IMU topic
 * @param max_acceleration Maximum expected acceleration (m/s^2)
 * @param max_angular_velocity Maximum expected angular velocity (rad/s)
 * @param timeout Time to wait for data
 * @return SensorDataResult
 */
SensorDataResult assertImuValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double max_acceleration = 50.0,
  double max_angular_velocity = 10.0,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that camera images are valid.
 *
 * Checks:
 * - Messages are being received
 * - Image dimensions match expected
 * - Encoding matches (if specified)
 * - Image data is non-empty
 *
 * @param node ROS 2 node
 * @param topic Camera image topic
 * @param expected_width Expected image width (0 = any)
 * @param expected_height Expected image height (0 = any)
 * @param expected_encoding Expected encoding (empty = any)
 * @param timeout Time to wait for data
 * @return SensorDataResult
 */
SensorDataResult assertCameraValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  uint32_t expected_width = 0,
  uint32_t expected_height = 0,
  const std::string & expected_encoding = "",
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Assert that joint states contain expected joints.
 *
 * Checks:
 * - Messages are being received
 * - All expected joints are present
 * - Position/velocity/effort values are not NaN
 *
 * @param node ROS 2 node
 * @param topic Joint states topic
 * @param expected_joints List of joint names that must be present
 * @param timeout Time to wait for data
 * @return SensorDataResult
 */
SensorDataResult assertJointStatesValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  const std::vector<std::string> & expected_joints,
  std::chrono::seconds timeout = std::chrono::seconds(5));

/**
 * @brief Check if a value is NaN.
 *
 * @tparam T Numeric type
 * @param value Value to check
 * @return true if NaN
 */
template<typename T>
bool isNan(T value)
{
  return std::isnan(static_cast<double>(value));
}

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__SENSOR_ASSERTIONS_HPP_
