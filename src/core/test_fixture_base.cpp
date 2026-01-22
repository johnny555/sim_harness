// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/core/test_fixture_base.hpp"

#include <any>

namespace sim_harness
{

void TestFixtureBase::SetUp()
{
  // Get or apply test isolation
  isolation_config_ = getTestIsolationConfig();

  // Create a unique node name for this test
  std::string node_name = generateTestNodeName("sim_test");

  // Create node with simulation time enabled
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  options.parameter_overrides({
    rclcpp::Parameter("use_sim_time", true)
  });

  node_ = std::make_shared<rclcpp::Node>(node_name, options);

  // Add node to executor
  executor_.add_node(node_);

  RCLCPP_INFO(
    node_->get_logger(),
    "Test fixture initialized: node=%s, domain=%s",
    node_name.c_str(),
    isolation_config_.domain_id.c_str());
}

void TestFixtureBase::TearDown()
{
  // Clear all collectors
  collectors_.clear();

  // Remove node from executor
  executor_.remove_node(node_);

  // Reset node
  node_.reset();
}

void TestFixtureBase::spinForDuration(std::chrono::milliseconds duration)
{
  sim_harness::spinForDuration(executor_, duration);
}

bool TestFixtureBase::spinUntilCondition(
  std::function<bool()> condition,
  std::chrono::milliseconds timeout)
{
  return sim_harness::spinUntilCondition(executor_, condition, timeout);
}

void TestFixtureBase::clearMessages(const std::string & key)
{
  // Note: This is a simplified implementation.
  // In a full implementation, we'd need to handle type erasure properly.
  // For now, collectors should be cleared via their direct references.
  (void)key;
  RCLCPP_WARN(
    node_->get_logger(),
    "clearMessages by key is not fully implemented. Use collector->clear() instead.");
}

}  // namespace sim_harness
