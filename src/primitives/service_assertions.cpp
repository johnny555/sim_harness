// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/primitives/service_assertions.hpp"

#include <rcl_interfaces/srv/get_parameters.hpp>

namespace sim_harness::primitives
{

bool assertServiceAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & service_name,
  std::chrono::seconds timeout)
{
  // Create a generic client just to check availability
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  return client->wait_for_service(timeout);
}

bool assertActionServerAvailable(
  rclcpp::Node::SharedPtr node,
  const std::string & action_name,
  std::chrono::seconds timeout)
{
  // Action servers expose several services, check for the main goal service
  std::string goal_service = action_name + "/_action/send_goal";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    auto service_names = node->get_service_names_and_types();
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

bool assertNodeRunning(
  rclcpp::Node::SharedPtr node,
  const std::string & node_name,
  std::chrono::seconds timeout)
{
  // Check if node's get_parameters service is available
  std::string service_name = node_name + "/get_parameters";
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  return client->wait_for_service(timeout);
}

std::vector<std::pair<std::string, bool>> assertNodesRunning(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & node_names,
  std::chrono::seconds timeout)
{
  std::vector<std::pair<std::string, bool>> results;
  results.reserve(node_names.size());

  auto per_node_timeout = std::chrono::seconds(
    timeout.count() / std::max(size_t(1), node_names.size()));
  per_node_timeout = std::max(per_node_timeout, std::chrono::seconds(2));

  for (const auto & name : node_names) {
    bool running = assertNodeRunning(node, name, per_node_timeout);
    results.emplace_back(name, running);
  }

  return results;
}

// Template specialization for common types would go here
// For now, the header-only template version can use get_parameters service

}  // namespace sim_harness::primitives
