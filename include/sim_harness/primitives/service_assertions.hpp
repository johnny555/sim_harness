// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__SERVICE_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__SERVICE_ASSERTIONS_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Result of a service availability check.
 */
struct ServiceResult
{
  /// Whether the service is available
  bool available;

  /// Whether the service call succeeded (if called)
  bool call_succeeded;

  /// Response time (if measured)
  std::chrono::milliseconds response_time;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a service is available.
 *
 * @param node ROS 2 node
 * @param service_name Full service name
 * @param timeout Maximum time to wait
 * @return true if service is available
 */
bool assertServiceAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & service_name,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that an action server is available.
 *
 * @param node ROS 2 node
 * @param action_name Full action name
 * @param timeout Maximum time to wait
 * @return true if action server is available
 */
bool assertActionServerAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that a node is running.
 *
 * Checks by trying to call the node's /get_parameters service.
 *
 * @param node ROS 2 node
 * @param node_name Name of the node to check
 * @param timeout Maximum time to wait
 * @return true if node responds
 */
bool assertNodeRunning(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that all listed nodes are running.
 *
 * @param node ROS 2 node
 * @param node_names List of node names to check
 * @param timeout Maximum time to wait for all nodes
 * @return Vector of (node_name, running) pairs
 */
std::vector<std::pair<std::string, bool>> assertNodesRunning(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & node_names,
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that a parameter exists on a node.
 *
 * @tparam T Parameter type
 * @param node ROS 2 node
 * @param node_name Name of the node with the parameter
 * @param parameter_name Name of the parameter
 * @param expected_value Optional expected value to check
 * @return true if parameter exists (and matches value if specified)
 */
template<typename T>
bool assertParameterExists(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name,
  const std::string & parameter_name,
  std::optional<T> expected_value = std::nullopt);

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__SERVICE_ASSERTIONS_HPP_
