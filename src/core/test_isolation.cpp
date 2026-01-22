// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/core/test_isolation.hpp"

#include <cstdlib>
#include <random>
#include <sstream>

namespace sim_harness
{

TestIsolationConfig getTestIsolationConfig()
{
  TestIsolationConfig config;

  // Get ROS_DOMAIN_ID from environment (default to 0)
  const char * domain_env = std::getenv("ROS_DOMAIN_ID");
  int domain_id = domain_env ? std::stoi(domain_env) : 0;

  // Map to valid range (100-199 for test isolation)
  int mapped_domain;
  if (domain_id >= 100 && domain_id <= 199) {
    // Already in valid range
    mapped_domain = domain_id;
  } else if (domain_id >= 200) {
    // Map 200+ to 100-199
    mapped_domain = 100 + ((domain_id - 200) % 100);
  } else {
    // Map 0-99 to 100-199
    mapped_domain = 100 + (domain_id % 100);
  }

  config.domain_id = std::to_string(mapped_domain);
  config.simulator_partition = "gz_test_" + config.domain_id;

  return config;
}

void applyTestIsolation(const TestIsolationConfig & config)
{
  // Set ROS domain ID
  setenv("ROS_DOMAIN_ID", config.domain_id.c_str(), 1);

  // Set Gazebo partition (for process isolation)
  setenv("GZ_PARTITION", config.simulator_partition.c_str(), 1);
}

std::string generateTestNodeName(const std::string & base_name)
{
  // Get domain ID for uniqueness
  const char * domain_env = std::getenv("ROS_DOMAIN_ID");
  std::string domain_suffix = domain_env ? domain_env : "0";

  // Add random suffix for additional uniqueness
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 9999);

  std::stringstream ss;
  ss << base_name << "_d" << domain_suffix << "_" << dis(gen);

  return ss.str();
}

}  // namespace sim_harness
