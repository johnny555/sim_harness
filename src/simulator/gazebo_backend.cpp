// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/simulator/gazebo_backend.hpp"

#include <array>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>

namespace sim_harness
{

namespace
{

std::string exec(const char * cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    return "";
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

std::set<int> findPidsByPattern(const std::string & pattern)
{
  std::set<int> pids;
  std::string cmd = "pgrep -f '" + pattern + "' 2>/dev/null";
  std::string output = exec(cmd.c_str());

  std::istringstream iss(output);
  int pid;
  while (iss >> pid) {
    pids.insert(pid);
  }
  return pids;
}

}  // namespace

GazeboBackend::GazeboBackend() = default;

bool GazeboBackend::isRunning() const
{
  return !getGazeboPids().empty();
}

bool GazeboBackend::waitUntilReady(std::chrono::seconds timeout)
{
  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < timeout) {
    if (isRunning()) {
      // Give Gazebo a moment to fully initialize
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;
}

std::string GazeboBackend::getPartition() const
{
  const char * partition = std::getenv("GZ_PARTITION");
  if (partition && partition[0] != '\0') {
    return partition;
  }

  // Generate from ROS_DOMAIN_ID
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  std::string domain_id = domain ? domain : "0";
  return "gz_test_" + domain_id;
}

std::set<int> GazeboBackend::getGazeboPids() const
{
  std::set<int> all_pids;

  for (const char * pattern : GAZEBO_PROCESS_PATTERNS) {
    auto pids = findPidsByPattern(pattern);
    all_pids.insert(pids.begin(), pids.end());
  }

  return all_pids;
}

bool GazeboBackend::isGazeboTopicAvailable(const std::string & topic) const
{
  // Use gz topic -l to list topics
  std::string cmd = "gz topic -l 2>/dev/null | grep -q '" + topic + "'";
  return system(cmd.c_str()) == 0;
}

// Factory implementation
std::unique_ptr<SimulatorInterface> SimulatorInterface::create(SimulatorType type)
{
  switch (type) {
    case SimulatorType::Gazebo:
      return std::make_unique<GazeboBackend>();
    case SimulatorType::None:
      return std::make_unique<NullBackend>();
    case SimulatorType::MuJoCo:
    case SimulatorType::Omniverse:
    default:
      // Not yet implemented - return null backend
      return std::make_unique<NullBackend>();
  }
}

}  // namespace sim_harness
