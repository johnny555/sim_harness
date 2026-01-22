// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/sensor_assertions.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sim_harness::primitives
{

SensorDataResult assertSensorPublishing(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double expected_rate_hz,
  double tolerance_percent,
  std::chrono::seconds sample_duration)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;
  result.publish_rate_hz = 0.0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Generic subscription that just counts messages
  size_t count = 0;
  auto sub = node->create_generic_subscription(
    topic, "std_msgs/msg/Header", 10,
    [&count](std::shared_ptr<rclcpp::SerializedMessage>) {
      ++count;
    });

  // Sample for the duration
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < sample_duration) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  result.message_count = count;
  double duration_sec = std::chrono::duration<double>(sample_duration).count();
  result.publish_rate_hz = static_cast<double>(count) / duration_sec;

  double deviation_percent = std::abs(result.publish_rate_hz - expected_rate_hz) /
    expected_rate_hz * 100.0;
  result.valid = deviation_percent <= tolerance_percent;

  std::stringstream ss;
  ss << "Rate: " << result.publish_rate_hz << " Hz";
  ss << " (expected: " << expected_rate_hz << " +/- " << tolerance_percent << "%)";
  ss << " - " << count << " messages in " << duration_sec << "s";
  result.details = ss.str();

  return result;
}

SensorDataResult assertLidarValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double min_range,
  double max_range,
  size_t min_points,
  std::chrono::seconds timeout)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::PointCloud2::SharedPtr latest_msg;

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, 10,
    [&latest_msg, &result](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      latest_msg = msg;
      result.message_count++;
    });

  // Wait for a message
  auto start = std::chrono::steady_clock::now();
  while (!latest_msg && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_msg) {
    result.details = "No PointCloud2 messages received on " + topic;
    return result;
  }

  // Count valid points
  size_t total_points = latest_msg->width * latest_msg->height;
  size_t valid_points = 0;
  bool has_nan = false;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x, y = *iter_y, z = *iter_z;

      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        has_nan = true;
        continue;
      }

      double range = std::sqrt(x * x + y * y + z * z);
      if (range >= min_range && range <= max_range) {
        valid_points++;
      }
    }
  } catch (const std::exception & e) {
    result.details = "Error reading point cloud: " + std::string(e.what());
    return result;
  }

  result.valid = valid_points >= min_points && !has_nan;

  std::stringstream ss;
  ss << "Points: " << valid_points << "/" << total_points << " valid";
  ss << " (range: " << min_range << "-" << max_range << "m)";
  if (has_nan) {
    ss << " [WARNING: NaN values detected]";
  }
  result.details = ss.str();

  return result;
}

SensorDataResult assertGpsValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double min_lat,
  double max_lat,
  double min_lon,
  double max_lon,
  std::chrono::seconds timeout)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::NavSatFix::SharedPtr latest_msg;

  auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    topic, 10,
    [&latest_msg, &result](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      latest_msg = msg;
      result.message_count++;
    });

  auto start = std::chrono::steady_clock::now();
  while (!latest_msg && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_msg) {
    result.details = "No NavSatFix messages received on " + topic;
    return result;
  }

  // Check for NaN
  bool has_nan = std::isnan(latest_msg->latitude) ||
    std::isnan(latest_msg->longitude) ||
    std::isnan(latest_msg->altitude);

  // Check bounds
  bool in_bounds =
    latest_msg->latitude >= min_lat && latest_msg->latitude <= max_lat &&
    latest_msg->longitude >= min_lon && latest_msg->longitude <= max_lon;

  // Check fix status
  bool has_fix = latest_msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  result.valid = !has_nan && in_bounds && has_fix;

  std::stringstream ss;
  ss << "GPS: lat=" << latest_msg->latitude << ", lon=" << latest_msg->longitude;
  ss << ", alt=" << latest_msg->altitude;
  ss << ", status=" << static_cast<int>(latest_msg->status.status);
  if (has_nan) {ss << " [ERROR: NaN values]";}
  if (!in_bounds) {ss << " [ERROR: Out of bounds]";}
  if (!has_fix) {ss << " [WARNING: No fix]";}
  result.details = ss.str();

  return result;
}

SensorDataResult assertImuValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double max_acceleration,
  double max_angular_velocity,
  std::chrono::seconds timeout)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::Imu::SharedPtr latest_msg;

  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    topic, 10,
    [&latest_msg, &result](const sensor_msgs::msg::Imu::SharedPtr msg) {
      latest_msg = msg;
      result.message_count++;
    });

  auto start = std::chrono::steady_clock::now();
  while (!latest_msg && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_msg) {
    result.details = "No IMU messages received on " + topic;
    return result;
  }

  // Check for NaN
  bool has_nan =
    std::isnan(latest_msg->linear_acceleration.x) ||
    std::isnan(latest_msg->linear_acceleration.y) ||
    std::isnan(latest_msg->linear_acceleration.z) ||
    std::isnan(latest_msg->angular_velocity.x) ||
    std::isnan(latest_msg->angular_velocity.y) ||
    std::isnan(latest_msg->angular_velocity.z);

  // Check acceleration magnitude
  double accel_mag = std::sqrt(
    std::pow(latest_msg->linear_acceleration.x, 2) +
    std::pow(latest_msg->linear_acceleration.y, 2) +
    std::pow(latest_msg->linear_acceleration.z, 2));

  // Check angular velocity magnitude
  double ang_vel_mag = std::sqrt(
    std::pow(latest_msg->angular_velocity.x, 2) +
    std::pow(latest_msg->angular_velocity.y, 2) +
    std::pow(latest_msg->angular_velocity.z, 2));

  bool accel_ok = accel_mag <= max_acceleration;
  bool ang_vel_ok = ang_vel_mag <= max_angular_velocity;

  result.valid = !has_nan && accel_ok && ang_vel_ok;

  std::stringstream ss;
  ss << "IMU: accel=" << accel_mag << " m/s^2, ang_vel=" << ang_vel_mag << " rad/s";
  if (has_nan) {ss << " [ERROR: NaN values]";}
  if (!accel_ok) {ss << " [ERROR: Acceleration too high]";}
  if (!ang_vel_ok) {ss << " [ERROR: Angular velocity too high]";}
  result.details = ss.str();

  return result;
}

SensorDataResult assertCameraValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  uint32_t expected_width,
  uint32_t expected_height,
  const std::string & expected_encoding,
  std::chrono::seconds timeout)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::Image::SharedPtr latest_msg;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, 10,
    [&latest_msg, &result](const sensor_msgs::msg::Image::SharedPtr msg) {
      latest_msg = msg;
      result.message_count++;
    });

  auto start = std::chrono::steady_clock::now();
  while (!latest_msg && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_msg) {
    result.details = "No Image messages received on " + topic;
    return result;
  }

  bool size_ok = true;
  if (expected_width > 0 && latest_msg->width != expected_width) {
    size_ok = false;
  }
  if (expected_height > 0 && latest_msg->height != expected_height) {
    size_ok = false;
  }

  bool encoding_ok = expected_encoding.empty() ||
    latest_msg->encoding == expected_encoding;

  bool data_ok = !latest_msg->data.empty();

  result.valid = size_ok && encoding_ok && data_ok;

  std::stringstream ss;
  ss << "Image: " << latest_msg->width << "x" << latest_msg->height;
  ss << ", encoding=" << latest_msg->encoding;
  ss << ", size=" << latest_msg->data.size() << " bytes";
  if (!size_ok) {ss << " [ERROR: Size mismatch]";}
  if (!encoding_ok) {ss << " [ERROR: Encoding mismatch]";}
  if (!data_ok) {ss << " [ERROR: Empty data]";}
  result.details = ss.str();

  return result;
}

SensorDataResult assertJointStatesValid(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  const std::vector<std::string> & expected_joints,
  std::chrono::seconds timeout)
{
  SensorDataResult result;
  result.valid = false;
  result.message_count = 0;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  sensor_msgs::msg::JointState::SharedPtr latest_msg;

  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    topic, 10,
    [&latest_msg, &result](const sensor_msgs::msg::JointState::SharedPtr msg) {
      latest_msg = msg;
      result.message_count++;
    });

  auto start = std::chrono::steady_clock::now();
  while (!latest_msg && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latest_msg) {
    result.details = "No JointState messages received on " + topic;
    return result;
  }

  // Check for expected joints
  std::vector<std::string> missing_joints;
  for (const auto & expected : expected_joints) {
    auto it = std::find(
      latest_msg->name.begin(), latest_msg->name.end(), expected);
    if (it == latest_msg->name.end()) {
      missing_joints.push_back(expected);
    }
  }

  // Check for NaN in values
  bool has_nan = false;
  for (double v : latest_msg->position) {
    if (std::isnan(v)) {has_nan = true; break;}
  }
  if (!has_nan) {
    for (double v : latest_msg->velocity) {
      if (std::isnan(v)) {has_nan = true; break;}
    }
  }
  if (!has_nan) {
    for (double v : latest_msg->effort) {
      if (std::isnan(v)) {has_nan = true; break;}
    }
  }

  result.valid = missing_joints.empty() && !has_nan;

  std::stringstream ss;
  ss << "JointStates: " << latest_msg->name.size() << " joints";
  if (!missing_joints.empty()) {
    ss << " [MISSING: ";
    for (size_t i = 0; i < missing_joints.size(); ++i) {
      ss << missing_joints[i];
      if (i < missing_joints.size() - 1) {ss << ", ";}
    }
    ss << "]";
  }
  if (has_nan) {ss << " [ERROR: NaN values]";}
  result.details = ss.str();

  return result;
}

}  // namespace sim_harness::primitives
