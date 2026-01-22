// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__SIMULATOR__SIMULATOR_INTERFACE_HPP_
#define SIM_HARNESS__SIMULATOR__SIMULATOR_INTERFACE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>

namespace sim_harness
{

/**
 * @brief Supported simulator types.
 */
enum class SimulatorType
{
  Gazebo,    ///< Gazebo Harmonic
  MuJoCo,    ///< MuJoCo (future)
  Omniverse, ///< NVIDIA Omniverse (future)
  None       ///< No simulator (for unit tests)
};

/**
 * @brief Convert SimulatorType to string.
 */
inline std::string simulatorTypeToString(SimulatorType type)
{
  switch (type) {
    case SimulatorType::Gazebo: return "Gazebo";
    case SimulatorType::MuJoCo: return "MuJoCo";
    case SimulatorType::Omniverse: return "Omniverse";
    case SimulatorType::None: return "None";
    default: return "Unknown";
  }
}

/**
 * @brief Configuration for launching a simulator.
 */
struct SimulatorConfig
{
  /// Path to world file (SDF, MJCF, USD, etc.)
  std::string world_file;

  /// Launch arguments (key-value pairs)
  std::map<std::string, std::string> launch_args;

  /// Maximum time to wait for simulator startup
  std::chrono::seconds startup_timeout{30};

  /// Whether to run headless (no GUI)
  bool headless = true;
};

/**
 * @brief Abstract interface for simulator backends.
 *
 * Allows tests to be written independent of the specific simulator.
 * Implementations handle simulator-specific details.
 *
 * Usage:
 * @code
 * auto sim = SimulatorInterface::create(SimulatorType::Gazebo);
 * sim->waitUntilReady(std::chrono::seconds(30));
 * // Run tests...
 * @endcode
 */
class SimulatorInterface
{
public:
  virtual ~SimulatorInterface() = default;

  /**
   * @brief Get the simulator type.
   */
  virtual SimulatorType type() const = 0;

  /**
   * @brief Check if the simulator is running.
   */
  virtual bool isRunning() const = 0;

  /**
   * @brief Wait until the simulator is ready for testing.
   *
   * @param timeout Maximum time to wait
   * @return true if simulator is ready, false on timeout
   */
  virtual bool waitUntilReady(std::chrono::seconds timeout) = 0;

  /**
   * @brief Get simulator-specific partition/namespace.
   *
   * For Gazebo this is GZ_PARTITION, for others it may be different.
   */
  virtual std::string getPartition() const = 0;

  /**
   * @brief Factory method to create simulator backend.
   *
   * @param type Type of simulator
   * @return Unique pointer to simulator interface
   */
  static std::unique_ptr<SimulatorInterface> create(SimulatorType type);
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__SIMULATOR__SIMULATOR_INTERFACE_HPP_
