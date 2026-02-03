// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__CORE__TEST_FIXTURE_BASE_HPP_
#define SIM_HARNESS__CORE__TEST_FIXTURE_BASE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "sim_harness/core/message_collector.hpp"
#include "sim_harness/core/spin_helpers.hpp"
#include "sim_harness/core/test_isolation.hpp"

namespace sim_harness
{

/**
 * @brief Base class for C++ integration tests with ROS 2 node lifecycle.
 *
 * Provides common utilities for simulation testing:
 * - Automatic ROS 2 node creation/destruction per test
 * - Message collection on arbitrary topics
 * - Spin helpers for waiting on messages or conditions
 * - Test isolation configuration
 *
 * Usage:
 * @code
 * class MyTest : public sim_harness::TestFixtureBase {
 * protected:
 *   void SetUp() override {
 *     TestFixtureBase::SetUp();
 *     odom_ = createMessageCollector<nav_msgs::msg::Odometry>("/odom", "odom");
 *   }
 *   std::shared_ptr<MessageCollector<nav_msgs::msg::Odometry>> odom_;
 * };
 *
 * TEST_F(MyTest, OdometryPublishes) {
 *   spinForDuration(std::chrono::seconds(5));
 *   EXPECT_GT(odom_->count(), 0);
 * }
 * @endcode
 */
class TestFixtureBase : public ::testing::Test
{
protected:
  /**
   * @brief Set up the test fixture.
   *
   * Creates a ROS 2 node and executor for the test.
   * Override this in derived classes, but always call the base implementation.
   */
  void SetUp() override;

  /**
   * @brief Tear down the test fixture.
   *
   * Cleans up the node and executor.
   * Override this in derived classes, but always call the base implementation.
   */
  void TearDown() override;

  /**
   * @brief Spin the executor for a specified duration.
   *
   * Processes callbacks for the given duration, allowing time for
   * messages to be received and simulation to progress.
   *
   * @param duration How long to spin
   */
  void spinForDuration(std::chrono::milliseconds duration);

  /**
   * @brief Spin until a condition is met or timeout occurs.
   *
   * @param condition Function returning true when condition is met
   * @param timeout Maximum time to wait
   * @return true if condition was met, false if timeout occurred
   */
  bool spinUntilCondition(
    std::function<bool()> condition,
    std::chrono::milliseconds timeout);

  /**
   * @brief Create a message collector for a topic.
   *
   * The collector is stored internally and managed by the test fixture.
   *
   * @tparam MsgT The message type
   * @param topic The topic to subscribe to
   * @param key Optional key for later retrieval (defaults to topic name)
   * @return Shared pointer to the collector
   */
  template<typename MsgT>
  std::shared_ptr<MessageCollector<MsgT>> createMessageCollector(
    const std::string & topic,
    const std::string & key = "")
  {
    auto collector = std::make_shared<MessageCollector<MsgT>>(node_, topic);
    std::string storage_key = key.empty() ? topic : key;
    collectors_[storage_key] = collector;
    return collector;
  }

  /**
   * @brief Get messages from a collector by key.
   *
   * @tparam MsgT The message type
   * @param key The collector key (topic name or custom key)
   * @return Vector of collected messages, or empty if key not found or type mismatch
   */
  template<typename MsgT>
  std::vector<MsgT> getMessages(const std::string & key)
  {
    auto it = collectors_.find(key);
    if (it == collectors_.end()) {
      return {};
    }
    // any_cast throws std::bad_any_cast on type mismatch
    try {
      auto collector =
        std::any_cast<std::shared_ptr<MessageCollector<MsgT>>>(it->second);
      return collector->getMessages();
    } catch (const std::bad_any_cast &) {
      return {};
    }
  }

  /**
   * @brief Clear messages from a collector.
   *
   * @param key The collector key
   */
  void clearMessages(const std::string & key);

  /**
   * @brief Get the test node.
   *
   * @return Shared pointer to the ROS 2 node
   */
  rclcpp::Node::SharedPtr getNode() const { return node_; }

  /**
   * @brief Get the logger for this test.
   *
   * @return Logger for the test node
   */
  rclcpp::Logger getLogger() const { return node_->get_logger(); }

  /// The ROS 2 node for this test
  rclcpp::Node::SharedPtr node_;

  /// Single-threaded executor for spinning
  rclcpp::executors::SingleThreadedExecutor executor_;

private:
  /// Storage for message collectors (type-erased)
  std::unordered_map<std::string, std::any> collectors_;

  /// Test isolation configuration
  TestIsolationConfig isolation_config_;
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__CORE__TEST_FIXTURE_BASE_HPP_
