// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__SIMULATOR__GAZEBO_BACKEND_HPP_
#define SIM_HARNESS__SIMULATOR__GAZEBO_BACKEND_HPP_

#include "sim_harness/simulator/simulator_interface.hpp"

#include <set>
#include <string>

namespace sim_harness
{

/**
 * @brief Gazebo Harmonic simulator backend.
 *
 * Provides Gazebo-specific functionality for:
 * - Detecting if Gazebo is running
 * - Managing GZ_PARTITION for test isolation
 * - Waiting for Gazebo to be ready
 */
class GazeboBackend : public SimulatorInterface
{
public:
  GazeboBackend();
  ~GazeboBackend() override = default;

  SimulatorType type() const override { return SimulatorType::Gazebo; }

  /**
   * @brief Check if Gazebo processes are running.
   *
   * Looks for processes matching gz, gzserver, ruby.*gz patterns.
   */
  bool isRunning() const override;

  /**
   * @brief Wait until Gazebo is ready.
   *
   * Checks for:
   * 1. Gazebo processes running
   * 2. ROS-Gazebo bridge topics available
   *
   * @param timeout Maximum time to wait
   * @return true if ready, false on timeout
   */
  bool waitUntilReady(std::chrono::seconds timeout) override;

  /**
   * @brief Get the GZ_PARTITION value.
   *
   * Returns the current GZ_PARTITION environment variable value,
   * or generates one based on ROS_DOMAIN_ID if not set.
   */
  std::string getPartition() const override;

  /**
   * @brief Get PIDs of running Gazebo processes.
   *
   * Useful for cleanup and monitoring.
   */
  std::set<int> getGazeboPids() const;

  /**
   * @brief Check if a specific Gazebo topic is available.
   *
   * @param topic Topic name
   * @return true if topic exists
   */
  bool isGazeboTopicAvailable(const std::string & topic) const;

private:
  /// Patterns to match Gazebo processes
  static constexpr const char * GAZEBO_PROCESS_PATTERNS[] = {
    "gz sim",
    "ruby.*gz",
    "gzserver",
    "gzclient"
  };
};

/**
 * @brief Null simulator backend for unit tests.
 *
 * Always reports as "running" and "ready" - useful for testing
 * without an actual simulator.
 */
class NullBackend : public SimulatorInterface
{
public:
  SimulatorType type() const override { return SimulatorType::None; }
  bool isRunning() const override { return true; }
  bool waitUntilReady(std::chrono::seconds) override { return true; }
  std::string getPartition() const override { return "null_partition"; }
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__SIMULATOR__GAZEBO_BACKEND_HPP_
