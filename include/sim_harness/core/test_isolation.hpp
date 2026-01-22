// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__CORE__TEST_ISOLATION_HPP_
#define SIM_HARNESS__CORE__TEST_ISOLATION_HPP_

#include <string>

namespace sim_harness
{

/**
 * @brief Configuration for test isolation.
 *
 * Test isolation prevents crosstalk between concurrent tests by using
 * unique ROS 2 domain IDs and simulator partitions.
 */
struct TestIsolationConfig
{
  /// ROS 2 domain ID (0-232)
  std::string domain_id;

  /// Simulator partition name (e.g., GZ_PARTITION for Gazebo)
  std::string simulator_partition;
};

/**
 * @brief Get the current test isolation configuration.
 *
 * Reads ROS_DOMAIN_ID from environment and maps it to a valid range.
 * Generates a partition name based on the domain ID.
 *
 * Domain ID mapping:
 * - If already in range 100-199: use as-is
 * - If >= 200: map to 100 + ((id - 200) % 100)
 * - Otherwise: map to 100 + (id % 100)
 *
 * @return TestIsolationConfig with domain_id and simulator_partition
 */
TestIsolationConfig getTestIsolationConfig();

/**
 * @brief Apply test isolation configuration to current process.
 *
 * Sets environment variables:
 * - ROS_DOMAIN_ID
 * - GZ_PARTITION (for Gazebo)
 *
 * @param config The isolation configuration to apply
 */
void applyTestIsolation(const TestIsolationConfig & config);

/**
 * @brief Generate a unique test node name.
 *
 * Creates a unique node name incorporating domain ID to avoid collisions.
 *
 * @param base_name Base name for the node
 * @return Unique node name (e.g., "base_name_domain_123")
 */
std::string generateTestNodeName(const std::string & base_name = "test_node");

}  // namespace sim_harness

#endif  // SIM_HARNESS__CORE__TEST_ISOLATION_HPP_
