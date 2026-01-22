// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__TIMING_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__TIMING_ASSERTIONS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a timing assertion.
 */
struct TimingResult
{
  /// Whether timing is within bounds
  bool within_bounds;

  /// Measured publish rate (Hz)
  double measured_rate_hz;

  /// Minimum latency observed (ms)
  double min_latency_ms;

  /// Maximum latency observed (ms)
  double max_latency_ms;

  /// Average latency (ms)
  double avg_latency_ms;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a topic publishes at the expected rate.
 *
 * @param node ROS 2 node
 * @param topic Topic to monitor
 * @param expected_hz Expected publish rate
 * @param tolerance_percent Acceptable deviation (percent)
 * @param sample_duration How long to sample
 * @return TimingResult
 */
TimingResult assertPublishRate(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  double expected_hz,
  double tolerance_percent = 10.0,
  std::chrono::seconds sample_duration = std::chrono::seconds(5));

/**
 * @brief Assert that message latency is within bounds.
 *
 * Measures time between message stamp and receive time.
 *
 * @param node ROS 2 node
 * @param topic Topic to monitor
 * @param max_latency Maximum acceptable latency
 * @param sample_duration How long to sample
 * @return TimingResult
 */
TimingResult assertLatency(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  std::chrono::milliseconds max_latency,
  std::chrono::seconds sample_duration = std::chrono::seconds(5));

/**
 * @brief Assert that a transform is available and fresh.
 *
 * @param node ROS 2 node
 * @param target_frame Target frame
 * @param source_frame Source frame
 * @param max_age Maximum age of transform
 * @return true if transform available and fresh
 */
bool assertTransformAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & target_frame,
  const std::string & source_frame,
  std::chrono::milliseconds max_age = std::chrono::milliseconds(100));

/**
 * @brief Assert that an action server responds within timeout.
 *
 * @param node ROS 2 node
 * @param action_name Action name
 * @param timeout Maximum time to wait for response
 * @return true if server responds
 */
bool assertActionServerResponsive(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  std::chrono::seconds timeout = std::chrono::seconds(5));

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__TIMING_ASSERTIONS_HPP_
