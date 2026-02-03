# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
sim_harness — Python test utilities for ROS 2 simulation testing.

Quick start::

    from sim_harness import SimTestFixture
    from sim_harness.checks.sensors import assert_lidar_valid
    from sim_harness.checks.readiness import assert_nav2_active
    from sim_harness.checks.motion import assert_vehicle_moved

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = assert_lidar_valid(self.node, '/scan')
            assert result.valid, result.details

Check modules (organized by purpose):

- ``sim_harness.checks.readiness``  — Is the system up?
- ``sim_harness.checks.sensors``    — Are sensors working?
- ``sim_harness.checks.motion``     — Does the robot move?
- ``sim_harness.checks.navigation`` — Can it navigate?
- ``sim_harness.checks.perception`` — Does it see the world?

Advanced / internal:

- ``sim_harness.core.topic_observer`` — TopicObserver fold combinator
- ``sim_harness.core.hypothesis``     — Property-based testing (optional)
"""

# === Core: the one fixture you need ===
from sim_harness.core.test_fixture import SimTestFixture  # noqa: F401

# Backwards compat alias (SimulationTestFixture is now SimTestFixture)
SimulationTestFixture = SimTestFixture

# === Core utilities ===
from sim_harness.core.message_collector import MessageCollector  # noqa: F401
from sim_harness.core.spin_helpers import (  # noqa: F401
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)

# === Simulator (lazy if Gazebo not installed) ===
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

# === Validation / requirements ===
from sim_harness.validation.validation_result import (  # noqa: F401
    ValidationResult,
    ValidationResultCollector,
    ValidationScope,
)
from sim_harness.validation.requirement_validator import (  # noqa: F401
    RequirementValidator,
)

# === Readiness checks ===
from sim_harness.checks.readiness import (  # noqa: F401
    ServiceResult,
    assert_service_available,
    assert_action_server_available,
    assert_action_server_responsive,
    assert_node_running,
    assert_nodes_running,
    assert_parameter_exists,
    LifecycleState,
    LifecycleResult,
    ControllerResult,
    LocalizationResult,
    assert_lifecycle_node_active,
    assert_lifecycle_node_state,
    assert_lifecycle_nodes_active,
    assert_controller_active,
    assert_controllers_active,
    assert_controller_manager_available,
    assert_nav2_active,
    assert_slam_toolbox_active,
    assert_localization_active,
)

# === Sensor checks ===
from sim_harness.checks.sensors import (  # noqa: F401
    SensorDataResult,
    TimingResult,
    assert_sensor_publishing,
    assert_lidar_valid,
    assert_gps_valid,
    assert_imu_valid,
    assert_camera_valid,
    assert_joint_states_valid,
    assert_publish_rate,
    assert_latency,
    assert_transform_available,
)

# === Motion checks ===
from sim_harness.checks.motion import (  # noqa: F401
    MovementResult,
    VelocityResult,
    assert_vehicle_moved,
    assert_vehicle_moved_with_ground_truth,
    assert_vehicle_stationary,
    assert_vehicle_velocity,
    assert_vehicle_in_region,
    assert_vehicle_orientation,
)

# === Navigation checks ===
from sim_harness.checks.navigation import (  # noqa: F401
    NavigationResult,
    assert_reaches_goal,
    assert_follows_path,
    assert_navigation_action_succeeds,
    assert_costmap_contains_obstacle,
)

# === Perception checks ===
from sim_harness.checks.perception import (  # noqa: F401
    DetectionResult,
    assert_object_detected,
    assert_object_detected_by_class,
    assert_min_objects_detected,
    assert_region_clear,
)

# === Predicate combinators ===
from sim_harness.checks import all_of, any_of, negate  # noqa: F401

# === Readiness check framework ===
from sim_harness.core.readiness_check import (  # noqa: F401
    ReadinessCheck,
    CheckResult,
    CheckItemResult,
    CheckStatus,
    CheckCategory,
    create_standard_check,
)

# === TopicObserver (for building custom checks) ===
from sim_harness.core.topic_observer import (  # noqa: F401
    TopicObserver,
    ObservationResult,
    ParallelObserver,
    SENSOR_QOS,
    collect_messages,
    count_messages,
    latest_message,
    track_max,
    track_timestamps,
)

# === Property-based testing (optional) ===
from sim_harness.core.sim_property import (  # noqa: F401
    sim_property,
    PropertyFailure,
)


# Test runner (lazy import to avoid circular imports)
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
    # Readiness checks
    'ServiceResult',
    'assert_service_available',
    'assert_action_server_available',
    'assert_action_server_responsive',
    'assert_node_running',
    'assert_nodes_running',
    'assert_parameter_exists',
    'LifecycleState',
    'LifecycleResult',
    'ControllerResult',
    'LocalizationResult',
    'assert_lifecycle_node_active',
    'assert_lifecycle_node_state',
    'assert_lifecycle_nodes_active',
    'assert_controller_active',
    'assert_controllers_active',
    'assert_controller_manager_available',
    'assert_nav2_active',
    'assert_slam_toolbox_active',
    'assert_localization_active',
    # Sensor checks
    'SensorDataResult',
    'TimingResult',
    'assert_sensor_publishing',
    'assert_lidar_valid',
    'assert_gps_valid',
    'assert_imu_valid',
    'assert_camera_valid',
    'assert_joint_states_valid',
    'assert_publish_rate',
    'assert_latency',
    'assert_transform_available',
    # Motion checks
    'MovementResult',
    'VelocityResult',
    'assert_vehicle_moved',
    'assert_vehicle_moved_with_ground_truth',
    'assert_vehicle_stationary',
    'assert_vehicle_velocity',
    'assert_vehicle_in_region',
    'assert_vehicle_orientation',
    # Navigation checks
    'NavigationResult',
    'assert_reaches_goal',
    'assert_follows_path',
    'assert_navigation_action_succeeds',
    'assert_costmap_contains_obstacle',
    # Perception checks
    'DetectionResult',
    'assert_object_detected',
    'assert_object_detected_by_class',
    'assert_min_objects_detected',
    'assert_region_clear',
    # Predicate combinators
    'all_of',
    'any_of',
    'negate',
    # Readiness check framework
    'ReadinessCheck',
    'CheckResult',
    'CheckItemResult',
    'CheckStatus',
    'CheckCategory',
    'create_standard_check',
    # TopicObserver (custom checks)
    'TopicObserver',
    'ObservationResult',
    'ParallelObserver',
    'SENSOR_QOS',
    'collect_messages',
    'count_messages',
    'latest_message',
    'track_max',
    'track_timestamps',
    # Property-based testing
    'sim_property',
    'PropertyFailure',
    # Test runner
    'get_test_runner',
    'get_test_registry',
]
