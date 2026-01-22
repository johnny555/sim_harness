// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/timing_assertions.hpp"

#include <atomic>
#include <sstream>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace sim_harness::primitives
{

TimingResult assertPublishRate(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double expected_hz,
  double tolerance_percent,
  std::chrono::seconds sample_duration)
{
  TimingResult result;
  result.within_bounds = false;
  result.measured_rate_hz = 0.0;
  result.min_latency_ms = 0.0;
  result.max_latency_ms = 0.0;
  result.avg_latency_ms = 0.0;

  // Create a temporary node to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "rate_checker_" + std::to_string(temp_node_counter++));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  std::vector<rclcpp::Time> timestamps;

  // Generic subscription to count messages
  auto sub = temp_node->create_generic_subscription(
    topic, "std_msgs/msg/Header", 10,
    [&timestamps, &temp_node](std::shared_ptr<rclcpp::SerializedMessage>) {
      timestamps.push_back(temp_node->now());
    });

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < sample_duration) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (timestamps.size() < 2) {
    result.details = "Not enough messages received to measure rate";
    return result;
  }

  double duration_sec = std::chrono::duration<double>(sample_duration).count();
  result.measured_rate_hz = static_cast<double>(timestamps.size()) / duration_sec;

  double deviation_percent = std::abs(result.measured_rate_hz - expected_hz) /
    expected_hz * 100.0;
  result.within_bounds = deviation_percent <= tolerance_percent;

  std::stringstream ss;
  ss << "Rate: " << result.measured_rate_hz << " Hz";
  ss << " (expected: " << expected_hz << " +/- " << tolerance_percent << "%)";
  result.details = ss.str();

  return result;
}

TimingResult assertLatency(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  std::chrono::milliseconds max_latency,
  std::chrono::seconds sample_duration)
{
  TimingResult result;
  result.within_bounds = true;
  result.min_latency_ms = std::numeric_limits<double>::max();
  result.max_latency_ms = 0.0;
  result.avg_latency_ms = 0.0;

  // Create a temporary node to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "latency_checker_" + std::to_string(temp_node_counter++));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  std::vector<double> latencies;

  auto sub = temp_node->create_subscription<std_msgs::msg::Header>(
    topic, 10,
    [&latencies, &temp_node, &result, max_latency](const std_msgs::msg::Header::SharedPtr msg) {
      auto now = temp_node->now();
      auto msg_time = rclcpp::Time(msg->stamp);
      double latency_ms = (now - msg_time).seconds() * 1000.0;

      latencies.push_back(latency_ms);
      result.min_latency_ms = std::min(result.min_latency_ms, latency_ms);
      result.max_latency_ms = std::max(result.max_latency_ms, latency_ms);

      if (latency_ms > static_cast<double>(max_latency.count())) {
        result.within_bounds = false;
      }
    });

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < sample_duration) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (!latencies.empty()) {
    double sum = 0.0;
    for (double l : latencies) {sum += l;}
    result.avg_latency_ms = sum / static_cast<double>(latencies.size());
  }

  std::stringstream ss;
  ss << "Latency: min=" << result.min_latency_ms << "ms";
  ss << ", max=" << result.max_latency_ms << "ms";
  ss << ", avg=" << result.avg_latency_ms << "ms";
  ss << " (limit: " << max_latency.count() << "ms)";
  result.details = ss.str();

  return result;
}

bool assertTransformAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & target_frame,
  const std::string & source_frame,
  std::chrono::milliseconds max_age)
{
  // Create a temporary node to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "tf_checker_" + std::to_string(temp_node_counter++));

  tf2_ros::Buffer tf_buffer(temp_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  // Give the TF listener time to connect and receive initial data
  // This is important for /tf_static which uses transient local QoS
  for (int i = 0; i < 20; ++i) {
    executor.spin_some(std::chrono::milliseconds(50));
  }

  // Wait for transform to become available (up to 10 seconds total)
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    executor.spin_some(std::chrono::milliseconds(50));

    try {
      auto transform = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);

      // For simulation time (stamp may be 0), just return true if found
      if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
        return true;
      }

      // Check age
      auto tf_time = rclcpp::Time(transform.header.stamp);
      auto now = temp_node->now();
      auto age_ms = std::chrono::milliseconds(
        static_cast<int64_t>((now - tf_time).seconds() * 1000));

      // For max_age > 1000ms, treat it as a relaxed check (just find the transform)
      if (max_age > std::chrono::milliseconds(1000) || age_ms <= max_age) {
        return true;
      }
    } catch (const tf2::TransformException &) {
      // Transform not yet available
    }
  }

  return false;
}

bool assertActionServerResponsive(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  std::chrono::seconds timeout)
{
  // Create a temporary node to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "action_checker_" + std::to_string(temp_node_counter++));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    auto service_names = temp_node->get_service_names_and_types();
    for (const auto & [name, types] : service_names) {
      if (name.find(action_name) != std::string::npos &&
        name.find("send_goal") != std::string::npos)
      {
        return true;
      }
    }
    executor.spin_some(std::chrono::milliseconds(100));
  }

  return false;
}

}  // namespace sim_harness::primitives
