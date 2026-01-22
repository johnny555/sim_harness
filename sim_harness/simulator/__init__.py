# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Simulator abstraction layer for ROS 2 simulation testing."""

from sim_harness.simulator.simulator_interface import (
    SimulatorType,
    SimulatorConfig,
    SimulatorInterface,
)
from sim_harness.simulator.gazebo_backend import (
    GazeboBackend,
    NullBackend,
)
from sim_harness.simulator.simulation_launcher import (
    SimulationLauncher,
    LaunchConfig,
)
from sim_harness.simulator.simulation_manager import (
    SimulationManager,
    SimulationRequest,
    get_simulation_manager,
)

__all__ = [
    'SimulatorType',
    'SimulatorConfig',
    'SimulatorInterface',
    'GazeboBackend',
    'NullBackend',
    'SimulationLauncher',
    'LaunchConfig',
    'SimulationManager',
    'SimulationRequest',
    'get_simulation_manager',
]
