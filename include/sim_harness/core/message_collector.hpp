// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__CORE__MESSAGE_COLLECTOR_HPP_
#define SIM_HARNESS__CORE__MESSAGE_COLLECTOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace sim_harness
{

/**
 * @brief Thread-safe message collector for ROS 2 topics.
 *
 * Subscribes to a topic and collects messages into a vector for later
 * assertion. All operations are thread-safe.
 *
 * @tparam MsgT The ROS 2 message type to collect
 */
template<typename MsgT>
class MessageCollector
{
public:
  using MessagePtr = typename MsgT::SharedPtr;
  using MessageVector = std::vector<MsgT>;

  /**
   * @brief Construct a message collector.
   *
   * @param node The ROS 2 node to create subscription on
   * @param topic The topic to subscribe to
   * @param qos QoS profile (default: 10)
   */
  MessageCollector(
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const rclcpp::QoS & qos = rclcpp::QoS(10))
  : node_(node), topic_(topic)
  {
    subscription_ = node_->create_subscription<MsgT>(
      topic,
      qos,
      [this](const MessagePtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*msg);
        if (callback_) {
          callback_(*msg);
        }
      });
  }

  /**
   * @brief Get all collected messages.
   *
   * @return Copy of all messages received so far
   */
  MessageVector getMessages() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  /**
   * @brief Get the latest message.
   *
   * @return The most recent message, or nullopt if none received
   */
  std::optional<MsgT> getLatest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (messages_.empty()) {
      return std::nullopt;
    }
    return messages_.back();
  }

  /**
   * @brief Get the number of messages collected.
   *
   * @return Number of messages
   */
  size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.size();
  }

  /**
   * @brief Check if any messages have been received.
   *
   * @return true if at least one message received
   */
  bool hasMessages() const
  {
    return count() > 0;
  }

  /**
   * @brief Clear all collected messages.
   */
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.clear();
  }

  /**
   * @brief Set a callback to be invoked on each message.
   *
   * @param callback Function to call with each new message
   */
  void setCallback(std::function<void(const MsgT &)> callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = callback;
  }

  /**
   * @brief Get the topic name.
   *
   * @return The topic this collector is subscribed to
   */
  std::string getTopic() const
  {
    return topic_;
  }

  /**
   * @brief Wait for a minimum number of messages.
   *
   * @param executor Executor to spin while waiting
   * @param min_count Minimum number of messages to wait for
   * @param timeout Maximum time to wait
   * @return true if min_count reached, false on timeout
   */
  bool waitForMessages(
    rclcpp::executors::SingleThreadedExecutor & executor,
    size_t min_count,
    std::chrono::milliseconds timeout)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      executor.spin_some(std::chrono::milliseconds(10));
      if (count() >= min_count) {
        return true;
      }
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;

  mutable std::mutex mutex_;
  MessageVector messages_;
  std::function<void(const MsgT &)> callback_;
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__CORE__MESSAGE_COLLECTOR_HPP_
