// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0
//
// sim_test_utils - Simulator-agnostic C++ test utilities for ROS 2
//
// This is the main include file that aggregates all test utilities.
// Include this header to get access to all testing functionality.

#ifndef SIM_HARNESS__SIM_HARNESS_HPP_
#define SIM_HARNESS__SIM_HARNESS_HPP_

// Core utilities
#include "sim_harness/core/test_fixture_base.hpp"
#include "sim_harness/core/message_collector.hpp"
#include "sim_harness/core/spin_helpers.hpp"
#include "sim_harness/core/test_isolation.hpp"

// Validation
#include "sim_harness/validation/validation_result.hpp"
#include "sim_harness/validation/requirement_validator.hpp"

// Primitives DSL
#include "sim_harness/primitives/vehicle_assertions.hpp"
#include "sim_harness/primitives/sensor_assertions.hpp"
#include "sim_harness/primitives/lifecycle_assertions.hpp"
#include "sim_harness/primitives/service_assertions.hpp"
#include "sim_harness/primitives/navigation_assertions.hpp"
#include "sim_harness/primitives/perception_assertions.hpp"
#include "sim_harness/primitives/timing_assertions.hpp"

// Simulator abstraction
#include "sim_harness/simulator/simulator_interface.hpp"
#include "sim_harness/simulator/gazebo_backend.hpp"

#endif  // SIM_HARNESS__SIM_HARNESS_HPP_
