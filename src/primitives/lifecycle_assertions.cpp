// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/lifecycle_assertions.hpp"

#include <atomic>
#include <sstream>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

namespace sim_harness::primitives
{

std::string lifecycleStateToString(LifecycleState state)
{
  switch (state) {
    case LifecycleState::Unknown: return "unknown";
    case LifecycleState::Unconfigured: return "unconfigured";
    case LifecycleState::Inactive: return "inactive";
    case LifecycleState::Active: return "active";
    case LifecycleState::Finalized: return "finalized";
    default: return "unknown";
  }
}

LifecycleResult assertLifecycleNodeActive(
  rclcpp::Node::SharedPtr node,
  const std::string & lifecycle_node_name,
  std::chrono::seconds timeout)
{
  return assertLifecycleNodeState(
    node, lifecycle_node_name, LifecycleState::Active, timeout);
}

LifecycleResult assertLifecycleNodeState(
  rclcpp::Node::SharedPtr node,
  const std::string & lifecycle_node_name,
  LifecycleState expected_state,
  std::chrono::seconds timeout)
{
  LifecycleResult result;
  result.success = false;
  result.current_state = LifecycleState::Unknown;
  result.time_to_reach = std::chrono::milliseconds(0);

  // Create a temporary node for service calls to avoid executor conflicts
  // (the passed-in node may already be in an executor)
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "lifecycle_checker_" + std::to_string(temp_node_counter++));

  // Create service client on temp node
  std::string service_name = lifecycle_node_name + "/get_state";
  auto client = temp_node->create_client<lifecycle_msgs::srv::GetState>(service_name);

  auto start = std::chrono::steady_clock::now();

  // Wait for service to be available
  if (!client->wait_for_service(timeout)) {
    std::stringstream ss;
    ss << "Service " << service_name << " not available after " << timeout.count() << "s";
    result.details = ss.str();
    return result;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  // Poll state until it matches or timeout
  while (std::chrono::steady_clock::now() - start < timeout) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);

    // Wait for response with short timeout
    auto status = executor.spin_until_future_complete(
      future, std::chrono::seconds(2));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      result.current_state = static_cast<LifecycleState>(response->current_state.id);

      if (result.current_state == expected_state) {
        result.success = true;
        result.time_to_reach = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start);
        break;
      }
    }

    // Small delay before retrying
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::stringstream ss;
  ss << lifecycle_node_name << ": " << lifecycleStateToString(result.current_state);
  if (result.success) {
    ss << " (reached in " << result.time_to_reach.count() << "ms)";
  } else {
    ss << " (expected: " << lifecycleStateToString(expected_state) << ")";
  }
  result.details = ss.str();

  return result;
}

std::vector<LifecycleResult> assertLifecycleNodesActive(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & lifecycle_node_names,
  std::chrono::seconds timeout)
{
  std::vector<LifecycleResult> results;
  results.reserve(lifecycle_node_names.size());

  // Calculate per-node timeout
  auto per_node_timeout = std::chrono::seconds(
    timeout.count() / std::max(size_t(1), lifecycle_node_names.size()));
  per_node_timeout = std::max(per_node_timeout, std::chrono::seconds(5));

  for (const auto & name : lifecycle_node_names) {
    results.push_back(assertLifecycleNodeActive(node, name, per_node_timeout));
  }

  return results;
}

ControllerResult assertControllerActive(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  const std::string & controller_name,
  std::chrono::seconds timeout)
{
  ControllerResult result;
  result.success = false;
  result.controller_name = controller_name;
  result.state = "unknown";

  // Create a temporary node for service calls to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "controller_checker_" + std::to_string(temp_node_counter++));

  // Create service client on temp node
  std::string service_name = controller_manager_name + "/list_controllers";
  auto client = temp_node->create_client<controller_manager_msgs::srv::ListControllers>(service_name);

  if (!client->wait_for_service(timeout)) {
    result.details = "Controller manager service not available";
    return result;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < timeout) {
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = client->async_send_request(request);

    auto status = executor.spin_until_future_complete(
      future, std::chrono::seconds(2));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();

      for (const auto & controller : response->controller) {
        if (controller.name == controller_name) {
          result.state = controller.state;
          if (controller.state == "active") {
            result.success = true;
          }
          break;
        }
      }

      if (result.success || result.state != "unknown") {
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::stringstream ss;
  ss << controller_name << ": " << result.state;
  if (!result.success && result.state == "unknown") {
    ss << " (controller not found)";
  }
  result.details = ss.str();

  return result;
}

std::vector<ControllerResult> assertControllersActive(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  const std::vector<std::string> & controller_names,
  std::chrono::seconds timeout)
{
  std::vector<ControllerResult> results;
  results.reserve(controller_names.size());

  auto per_controller_timeout = std::chrono::seconds(
    timeout.count() / std::max(size_t(1), controller_names.size()));
  per_controller_timeout = std::max(per_controller_timeout, std::chrono::seconds(5));

  for (const auto & name : controller_names) {
    results.push_back(
      assertControllerActive(node, controller_manager_name, name, per_controller_timeout));
  }

  return results;
}

bool assertControllerManagerAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_manager_name,
  std::chrono::seconds timeout)
{
  std::string service_name = controller_manager_name + "/list_controllers";
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>(service_name);
  return client->wait_for_service(timeout);
}

std::vector<LifecycleResult> assertNav2Active(
  rclcpp::Node::SharedPtr node,
  const std::string & namespace_,
  std::chrono::seconds timeout)
{
  // Standard Nav2 lifecycle nodes
  std::vector<std::string> nav2_nodes = {
    "bt_navigator",
    "controller_server",
    "planner_server",
    "recoveries_server",
    "waypoint_follower"
  };

  // Add namespace prefix if specified
  if (!namespace_.empty()) {
    for (auto & name : nav2_nodes) {
      name = namespace_ + "/" + name;
    }
  }

  return assertLifecycleNodesActive(node, nav2_nodes, timeout);
}

LifecycleResult assertSlamToolboxActive(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name,
  std::chrono::seconds timeout)
{
  return assertLifecycleNodeActive(node, node_name, timeout);
}

LocalizationResult assertLocalizationActive(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name,
  double max_covariance_trace,
  std::chrono::seconds timeout)
{
  LocalizationResult result;
  result.active = false;
  result.converged = false;
  result.covariance_trace = std::numeric_limits<double>::max();

  // First check if AMCL lifecycle node is active
  auto lifecycle_result = assertLifecycleNodeActive(node, node_name, timeout);
  result.active = lifecycle_result.success;

  if (!result.active) {
    result.details = "AMCL not active: " + lifecycle_result.details;
    return result;
  }

  // Create a temporary node for subscription to avoid executor conflicts
  static std::atomic<int> temp_node_counter{0};
  auto temp_node = rclcpp::Node::make_shared(
    "localization_checker_" + std::to_string(temp_node_counter++));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(temp_node);

  std::string pose_topic = "/" + node_name + "/pose";
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose;

  auto sub = temp_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic, 10,
    [&latest_pose](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      latest_pose = msg;
    });

  auto start = std::chrono::steady_clock::now();
  while (!latest_pose &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  if (latest_pose) {
    // Calculate covariance trace (sum of diagonal elements)
    // Covariance is 36 element array [x, y, z, roll, pitch, yaw]
    result.covariance_trace =
      latest_pose->pose.covariance[0] +   // x variance
      latest_pose->pose.covariance[7] +   // y variance
      latest_pose->pose.covariance[35];   // yaw variance

    result.converged = result.covariance_trace <= max_covariance_trace;
  }

  std::stringstream ss;
  ss << "AMCL: active=" << (result.active ? "yes" : "no");
  ss << ", converged=" << (result.converged ? "yes" : "no");
  ss << ", cov_trace=" << result.covariance_trace;
  ss << " (max: " << max_covariance_trace << ")";
  result.details = ss.str();

  return result;
}

}  // namespace sim_harness::primitives
