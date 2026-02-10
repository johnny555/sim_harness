# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
sim_harness -- Python test utilities for ROS 2 simulation testing.

Quick start::

    from sim_harness import SimTestFixture
    from sim_harness.checks import check_lidar_valid

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = check_lidar_valid(self.node, '/scan')
            assert result.ok, result.details

Check functions (Layer 1)::

    from sim_harness.checks import check_sensor_publishing, check_gps_valid
    from sim_harness.nav2 import check_nav2_active, check_reaches_goal
    from sim_harness.perception import check_object_detected

Validation (Layer 2)::

    from sim_harness.validation import RequirementValidator, ValidationScope
"""

# -- Layer 0: Core (what every test needs) ─────────────────────────────────
from sim_harness.fixture import (  # noqa: F401
    SimTestFixture,
    ros_node,
    ros_executor,
)
from sim_harness.collector import MessageCollector  # noqa: F401
from sim_harness.spin import (  # noqa: F401
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)

# -- Simulator (lazy — only imported when used) ────────────────────────────
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

# -- Validation / requirements ─────────────────────────────────────────────
from sim_harness.validation.validation_result import (  # noqa: F401
    ValidationResult,
    ValidationResultCollector,
    ValidationScope,
)
from sim_harness.validation.requirement_validator import (  # noqa: F401
    RequirementValidator,
)

# -- Test runner (lazy) ────────────────────────────────────────────────────

def get_test_runner():
    """Get the TestRunner class for running launch tests."""
    from sim_harness.test_runner import TestRunner
    return TestRunner

def get_test_registry():
    """Get the TestRegistry class for discovering tests."""
    from sim_harness.test_runner import TestRegistry
    return TestRegistry


# -- Backward compatibility ────────────────────────────────────────────────
# Everything previously importable via ``from sim_harness import X`` still
# works.  These names are importable but NOT in ``__all__``.

from sim_harness.fixture import SimulationTestFixture  # noqa: F401

from sim_harness.checks import (  # noqa: F401
    # Result types
    ServiceResult, SensorDataResult, TimingResult, MovementResult, VelocityResult,
    # check_* (primary names)
    check_service_available,
    check_action_server_available,
    check_node_running,
    check_nodes_running,
    check_parameter_exists,
    check_sensor_publishing,
    check_lidar_valid,
    check_gps_valid,
    check_imu_valid,
    check_camera_valid,
    check_joint_states_valid,
    check_publish_rate,
    check_latency,
    check_transform_available,
    check_action_server_responsive,
    check_vehicle_moved,
    check_vehicle_moved_with_ground_truth,
    check_vehicle_stationary,
    check_vehicle_velocity,
    check_vehicle_in_region,
    check_vehicle_orientation,
    # assert_* (deprecated aliases)
    assert_service_available,
    assert_action_server_available,
    assert_node_running,
    assert_nodes_running,
    assert_parameter_exists,
    assert_sensor_publishing,
    assert_lidar_valid,
    assert_gps_valid,
    assert_imu_valid,
    assert_camera_valid,
    assert_joint_states_valid,
    assert_publish_rate,
    assert_latency,
    assert_transform_available,
    assert_action_server_responsive,
    assert_vehicle_moved,
    assert_vehicle_moved_with_ground_truth,
    assert_vehicle_stationary,
    assert_vehicle_velocity,
    assert_vehicle_in_region,
    assert_vehicle_orientation,
)

__all__ = [
    # Layer 0: Core fixture
    'SimTestFixture',
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
    # Test runner
    'get_test_runner',
    'get_test_registry',
]
