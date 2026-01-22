// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__CORE__SPIN_HELPERS_HPP_
#define SIM_HARNESS__CORE__SPIN_HELPERS_HPP_

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

namespace sim_harness
{

/**
 * @brief Spin a node for a specified duration.
 *
 * This function spins the given executor, processing callbacks for the
 * specified duration. Useful for waiting for messages or allowing time
 * for simulation to progress.
 *
 * @param executor The executor to spin
 * @param duration How long to spin for
 * @param spin_interval How often to call spin_some (default 10ms)
 */
inline void spinForDuration(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration,
  std::chrono::milliseconds spin_interval = std::chrono::milliseconds(10))
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < duration) {
    executor.spin_some(spin_interval);
  }
}

/**
 * @brief Spin until a condition is met or timeout occurs.
 *
 * Useful for waiting for a specific state (e.g., messages received,
 * node active) with a timeout to prevent infinite waiting.
 *
 * @param executor The executor to spin
 * @param condition Function returning true when condition is met
 * @param timeout Maximum time to wait
 * @param spin_interval How often to call spin_some (default 10ms)
 * @return true if condition was met, false if timeout occurred
 */
inline bool spinUntilCondition(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::function<bool()> condition,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds spin_interval = std::chrono::milliseconds(10))
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(spin_interval);
    if (condition()) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Spin until a minimum number of messages are received.
 *
 * @param executor The executor to spin
 * @param count_getter Function returning current message count
 * @param min_count Minimum number of messages required
 * @param timeout Maximum time to wait
 * @return true if min_count was reached, false if timeout occurred
 */
inline bool spinUntilMessagesReceived(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::function<size_t()> count_getter,
  size_t min_count,
  std::chrono::milliseconds timeout)
{
  return spinUntilCondition(
    executor,
    [&]() { return count_getter() >= min_count; },
    timeout);
}

}  // namespace sim_harness

#endif  // SIM_HARNESS__CORE__SPIN_HELPERS_HPP_
