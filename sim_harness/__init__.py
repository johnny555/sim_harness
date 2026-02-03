# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
sim_harness — Python test utilities for ROS 2 simulation testing.

Quick start::

    from sim_harness import SimTestFixture, assert_lidar_valid

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = assert_lidar_valid(self.node, '/scan')
            assert result.valid, result.details

Extensions for specialized stacks::

    from sim_harness.nav2 import assert_nav2_active, assert_reaches_goal
    from sim_harness.perception import assert_object_detected
"""

# ── Core fixture ──────────────────────────────────────────────────────────
from sim_harness.fixture import (  # noqa: F401
    SimTestFixture,
    SimulationTestFixture,
    ros_node,
    ros_executor,
)

# ── Core utilities ────────────────────────────────────────────────────────
from sim_harness.collector import MessageCollector  # noqa: F401
from sim_harness.spin import (  # noqa: F401
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)

# ── Service checks ────────────────────────────────────────────────────────
from sim_harness.assertions import (  # noqa: F401
    ServiceResult,
    assert_service_available,
    assert_action_server_available,
    assert_node_running,
    assert_nodes_running,
    assert_parameter_exists,
    # Sensor checks
    SensorDataResult,
    assert_sensor_publishing,
    assert_lidar_valid,
    assert_gps_valid,
    assert_imu_valid,
    assert_camera_valid,
    assert_joint_states_valid,
    # Timing checks
    TimingResult,
    assert_publish_rate,
    assert_latency,
    assert_transform_available,
    assert_action_server_responsive,
    # Motion checks
    MovementResult,
    VelocityResult,
    assert_vehicle_moved,
    assert_vehicle_moved_with_ground_truth,
    assert_vehicle_stationary,
    assert_vehicle_velocity,
    assert_vehicle_in_region,
    assert_vehicle_orientation,
)

# ── Simulator (lazy — only imported when used) ────────────────────────────
from sim_harness.simulator.simulation_manager import (  # noqa: F401
    SimulationManager,
    SimulationRequest,
    get_simulation_manager,
)
from sim_harness.simulator.simulator_interface import (  # noqa: F401
    SimulatorType,
    SimulatorConfig,
    SimulatorInterface,
)
from sim_harness.simulator.gazebo_backend import (  # noqa: F401
    GazeboBackend,
    NullBackend,
)
from sim_harness.simulator.simulation_launcher import (  # noqa: F401
    SimulationLauncher,
    LaunchConfig,
    kill_all_gazebo,
)

# ── Validation / requirements ─────────────────────────────────────────────
from sim_harness.validation.validation_result import (  # noqa: F401
    ValidationResult,
    ValidationResultCollector,
    ValidationScope,
)
from sim_harness.validation.requirement_validator import (  # noqa: F401
    RequirementValidator,
)

# ── Test runner (lazy) ────────────────────────────────────────────────────

def get_test_runner():
    """Get the TestRunner class for running launch tests."""
    from sim_harness.test_runner import TestRunner
    return TestRunner

def get_test_registry():
    """Get the TestRegistry class for discovering tests."""
    from sim_harness.test_runner import TestRegistry
    return TestRegistry


__all__ = [
    # Core fixture
    'SimTestFixture',
    'SimulationTestFixture',
    'MessageCollector',
    'spin_for_duration',
    'spin_until_condition',
    'spin_until_messages_received',
    'ros_node',
    'ros_executor',
    # Simulator
    'SimulationManager',
    'SimulationRequest',
    'get_simulation_manager',
    'SimulatorType',
    'SimulatorConfig',
    'SimulatorInterface',
    'GazeboBackend',
    'NullBackend',
    'SimulationLauncher',
    'LaunchConfig',
    'kill_all_gazebo',
    # Validation
    'ValidationResult',
    'ValidationResultCollector',
    'ValidationScope',
    'RequirementValidator',
    # Service checks
    'ServiceResult',
    'assert_service_available',
    'assert_action_server_available',
    'assert_node_running',
    'assert_nodes_running',
    'assert_parameter_exists',
    # Sensor checks
    'SensorDataResult',
    'assert_sensor_publishing',
    'assert_lidar_valid',
    'assert_gps_valid',
    'assert_imu_valid',
    'assert_camera_valid',
    'assert_joint_states_valid',
    # Timing checks
    'TimingResult',
    'assert_publish_rate',
    'assert_latency',
    'assert_transform_available',
    'assert_action_server_responsive',
    # Motion checks
    'MovementResult',
    'VelocityResult',
    'assert_vehicle_moved',
    'assert_vehicle_moved_with_ground_truth',
    'assert_vehicle_stationary',
    'assert_vehicle_velocity',
    'assert_vehicle_in_region',
    'assert_vehicle_orientation',
    # Test runner
    'get_test_runner',
    'get_test_registry',
]
