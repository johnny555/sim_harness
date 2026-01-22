// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__PRIMITIVES__LIFECYCLE_ASSERTIONS_HPP_
#define SIM_HARNESS__PRIMITIVES__LIFECYCLE_ASSERTIONS_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace sim_harness::primitives
{

/**
 * @brief Lifecycle states (matches ROS 2 lifecycle_msgs).
 */
enum class LifecycleState
{
  Unknown = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN,
  Unconfigured = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
  Inactive = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
  Active = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
  Finalized = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED
};

/**
 * @brief Convert LifecycleState to string.
 */
std::string lifecycleStateToString(LifecycleState state);

/**
 * @brief Result of a lifecycle node assertion.
 */
struct LifecycleResult
{
  /// Whether the assertion succeeded
  bool success;

  /// Current state of the node
  LifecycleState current_state;

  /// Time taken to reach the state (if applicable)
  std::chrono::milliseconds time_to_reach;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Result of a controller assertion.
 */
struct ControllerResult
{
  /// Whether the controller is in expected state
  bool success;

  /// Controller name
  std::string controller_name;

  /// Current state (e.g., "active", "inactive")
  std::string state;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Result of a localization assertion.
 */
struct LocalizationResult
{
  /// Whether localization is active
  bool active;

  /// Whether localization has converged
  bool converged;

  /// Covariance trace (lower = more confident)
  double covariance_trace;

  /// Human-readable details
  std::string details;
};

/**
 * @brief Assert that a lifecycle node reaches the Active state.
 *
 * Uses the /get_state service to query node state.
 *
 * @param node ROS 2 node for service calls
 * @param lifecycle_node_name Name of the lifecycle node
 * @param timeout Maximum time to wait
 * @return LifecycleResult
 */
LifecycleResult assertLifecycleNodeActive(
  rclcpp::Node::SharedPtr node,
  const std::string & lifecycle_node_name,
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that a lifecycle node is in a specific state.
 *
 * @param node ROS 2 node
 * @param lifecycle_node_name Name of the lifecycle node
 * @param expected_state Expected state
 * @param timeout Maximum time to wait
 * @return LifecycleResult
 */
LifecycleResult assertLifecycleNodeState(
  rclcpp::Node::SharedPtr node,
  const std::string & lifecycle_node_name,
  LifecycleState expected_state,
  std::chrono::seconds timeout = std::chrono::seconds(10));

/**
 * @brief Assert that multiple lifecycle nodes are all active.
 *
 * Useful for checking entire stacks (e.g., Nav2).
 *
 * @param node ROS 2 node
 * @param lifecycle_node_names List of node names to check
 * @param timeout Maximum time to wait for all nodes
 * @return Vector of results, one per node
 */
std::vector<LifecycleResult> assertLifecycleNodesActive(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & lifecycle_node_names,
  std::chrono::seconds timeout = std::chrono::seconds(60));

/**
 * @brief Assert that a ros2_control controller is active.
 *
 * Uses controller_manager services to check state.
 *
 * @param node ROS 2 node
 * @param controller_manager_name Controller manager namespace
 * @param controller_name Name of the controller
 * @param timeout Maximum time to wait
 * @return ControllerResult
 */
ControllerResult assertControllerActive(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  const std::string & controller_name,
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that multiple controllers are active.
 *
 * @param node ROS 2 node
 * @param controller_manager_name Controller manager namespace
 * @param controller_names List of controller names
 * @param timeout Maximum time to wait
 * @return Vector of results
 */
std::vector<ControllerResult> assertControllersActive(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  const std::vector<std::string> & controller_names,
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that the controller manager is available.
 *
 * @param node ROS 2 node
 * @param controller_manager_name Controller manager namespace
 * @param timeout Maximum time to wait
 * @return true if controller manager responds
 */
bool assertControllerManagerAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that the Nav2 navigation stack is fully active.
 *
 * Checks standard Nav2 nodes:
 * - bt_navigator
 * - controller_server
 * - planner_server
 * - recoveries_server
 * - waypoint_follower
 *
 * @param node ROS 2 node
 * @param namespace_ Namespace for Nav2 nodes (empty = default)
 * @param timeout Maximum time to wait
 * @return Vector of results for each Nav2 node
 */
std::vector<LifecycleResult> assertNav2Active(
  rclcpp::Node::SharedPtr node,
  const std::string & namespace_ = "",
  std::chrono::seconds timeout = std::chrono::seconds(60));

/**
 * @brief Assert that SLAM Toolbox is active.
 *
 * @param node ROS 2 node
 * @param node_name SLAM Toolbox node name
 * @param timeout Maximum time to wait
 * @return LifecycleResult
 */
LifecycleResult assertSlamToolboxActive(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name = "slam_toolbox",
  std::chrono::seconds timeout = std::chrono::seconds(30));

/**
 * @brief Assert that localization (AMCL) is active and converged.
 *
 * @param node ROS 2 node
 * @param node_name AMCL node name
 * @param max_covariance_trace Maximum covariance trace for "converged"
 * @param timeout Maximum time to wait
 * @return LocalizationResult
 */
LocalizationResult assertLocalizationActive(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name = "amcl",
  double max_covariance_trace = 1.0,
  std::chrono::seconds timeout = std::chrono::seconds(30));

}  // namespace sim_harness::primitives

#endif  // SIM_HARNESS__PRIMITIVES__LIFECYCLE_ASSERTIONS_HPP_
