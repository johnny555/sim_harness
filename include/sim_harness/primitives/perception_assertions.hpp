// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__PERCEPTION_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__PERCEPTION_ASSERTIONS_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a detection assertion.
 */
struct DetectionResult
{
  /// Whether object was detected
  bool detected;

  /// Number of detections
  size_t detection_count;

  /// Closest detected position (if any)
  std::optional<geometry_msgs::msg::Point> closest_position;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that an object is detected near an expected position.
 *
 * @param node ROS 2 node
 * @param detection_topic Topic with detection messages
 * @param expected_position Expected object position
 * @param search_radius Search radius (meters)
 * @param timeout Maximum time to wait
 * @return DetectionResult
 */
DetectionResult assertObjectDetected(
  rclcpp::Node::SharedPtr node,
  const std::string & detection_topic,
  const geometry_msgs::msg::Point & expected_position,
  double search_radius = 1.0,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that an object of a specific class is detected.
 *
 * @param node ROS 2 node
 * @param detection_topic Topic with detection messages
 * @param object_class Class/label to search for
 * @param min_confidence Minimum confidence threshold
 * @param timeout Maximum time to wait
 * @return DetectionResult
 */
DetectionResult assertObjectDetectedByClass(
  rclcpp::Node::SharedPtr node,
  const std::string & detection_topic,
  const std::string & object_class,
  double min_confidence = 0.5,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that a minimum number of objects are detected.
 *
 * @param node ROS 2 node
 * @param detection_topic Topic with detection messages
 * @param min_count Minimum number of detections
 * @param timeout Maximum time to wait
 * @return DetectionResult
 */
DetectionResult assertMinObjectsDetected(
  rclcpp::Node::SharedPtr node,
  const std::string & detection_topic,
  size_t min_count,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that a region is clear of detections (safety zone).
 *
 * @param node ROS 2 node
 * @param detection_topic Topic with detection messages
 * @param center Center of the region
 * @param radius Radius of the region (meters)
 * @param observation_period How long to observe
 * @return true if no objects detected in region
 */
bool assertRegionClear(
  rclcpp::Node::SharedPtr node,
  const std::string & detection_topic,
  const geometry_msgs::msg::Point & center,
  double radius,
  std::chrono::seconds observation_period = std::chrono::seconds(5));

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__PERCEPTION_ASSERTIONS_HPP_
