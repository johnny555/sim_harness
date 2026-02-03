# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
sim_harness - Python Test Utilities for ROS 2 Simulation Testing.

A pytest-based test framework for ROS 2 robotic simulations, providing:
- Test fixtures with automatic ROS 2 node management
- Message collectors for topic subscription
- Sensor, vehicle, and navigation assertion helpers
- Requirements traceability and validation

Example usage:
    import pytest
    from sim_harness import SimTestFixture, assert_sensor_publishing

    class TestMyRobot(SimTestFixture):
        def test_lidar_publishes(self):
            collector = self.create_message_collector('/scan', LaserScan)
            self.spin_for_duration(5.0)
            assert collector.count() > 0
"""

from sim_harness.core.test_fixture import SimTestFixture
from sim_harness.core.simulation_fixture import (
    SimulationTestFixture,
    simulation_manager,
    session_simulation,
)
from sim_harness.simulator.simulation_manager import (
    SimulationManager,
    SimulationRequest,
    get_simulation_manager,
)
from sim_harness.core.message_collector import MessageCollector
from sim_harness.core.spin_helpers import (
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)
from sim_harness.core.test_isolation import (
    get_test_isolation_config,
    apply_test_isolation,
    generate_test_node_name,
)

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
    kill_all_gazebo,
)

from sim_harness.validation.validation_result import (
    ValidationResult,
    ValidationResultCollector,
    ValidationScope,
)
from sim_harness.validation.requirement_validator import RequirementValidator

from sim_harness.primitives.sensor_assertions import (
    SensorDataResult,
    assert_sensor_publishing,
    assert_lidar_valid,
    assert_gps_valid,
    assert_imu_valid,
    assert_camera_valid,
    assert_joint_states_valid,
)

from sim_harness.primitives.vehicle_assertions import (
    MovementResult,
    VelocityResult,
    assert_vehicle_moved,
    assert_vehicle_moved_with_ground_truth,
    assert_vehicle_stationary,
    assert_vehicle_velocity,
    assert_vehicle_in_region,
    assert_vehicle_orientation,
)

from sim_harness.primitives.lifecycle_assertions import (
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

from sim_harness.primitives.navigation_assertions import (
    NavigationResult,
    assert_reaches_goal,
    assert_follows_path,
    assert_navigation_action_succeeds,
    assert_costmap_contains_obstacle,
)

from sim_harness.primitives.service_assertions import (
    ServiceResult,
    assert_service_available,
    assert_action_server_available,
    assert_node_running,
    assert_nodes_running,
    assert_parameter_exists,
)

from sim_harness.primitives.timing_assertions import (
    TimingResult,
    assert_publish_rate,
    assert_latency,
    assert_transform_available,
    assert_action_server_responsive,
)

from sim_harness.primitives.perception_assertions import (
    DetectionResult,
    assert_object_detected,
    assert_object_detected_by_class,
    assert_min_objects_detected,
    assert_region_clear,
)

from sim_harness.core.readiness_check import (
    ReadinessCheck,
    CheckResult,
    CheckItemResult,
    CheckStatus,
    CheckCategory,
    create_standard_check,
)

# FP-inspired combinators (TopicObserver, stream properties, predicates)
from sim_harness.core.topic_observer import (
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
from sim_harness.core.stream_properties import (
    PropertyResult,
    for_all_messages,
    eventually,
    monotonic,
    all_of,
    any_of,
    negate,
)

# Hypothesis property-based testing (optional dependency)
from sim_harness.core.sim_property import (
    sim_property,
    PropertyFailure,
    check_recorded_property,
    check_recorded_eventually,
    check_recorded_monotonic,
    hypothesis_check_recorded,
)
from sim_harness.core.strategies import (
    point_strategy,
    vector3_strategy,
    quaternion_strategy,
    yaw_quaternion_strategy,
    pose_strategy,
    twist_strategy,
    twist_strategy_3d,
    navigation_goal_2d,
    waypoints_strategy,
    duration_strategy,
    rate_strategy,
    threshold_strategy,
    angle_strategy,
    speed_strategy,
    distance_strategy,
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
    # Core
    'SimTestFixture',
    'SimulationTestFixture',
    'simulation_manager',
    'session_simulation',
    'MessageCollector',
    'spin_for_duration',
    'spin_until_condition',
    'spin_until_messages_received',
    'get_test_isolation_config',
    'apply_test_isolation',
    'generate_test_node_name',
    # Simulator
    'SimulatorType',
    'SimulatorConfig',
    'SimulatorInterface',
    'GazeboBackend',
    'NullBackend',
    'SimulationLauncher',
    'LaunchConfig',
    'kill_all_gazebo',
    'SimulationManager',
    'SimulationRequest',
    'get_simulation_manager',
    # Validation
    'ValidationResult',
    'ValidationResultCollector',
    'ValidationScope',
    'RequirementValidator',
    # Sensor assertions
    'SensorDataResult',
    'assert_sensor_publishing',
    'assert_lidar_valid',
    'assert_gps_valid',
    'assert_imu_valid',
    'assert_camera_valid',
    'assert_joint_states_valid',
    # Vehicle assertions
    'MovementResult',
    'VelocityResult',
    'assert_vehicle_moved',
    'assert_vehicle_moved_with_ground_truth',
    'assert_vehicle_stationary',
    'assert_vehicle_velocity',
    'assert_vehicle_in_region',
    'assert_vehicle_orientation',
    # Lifecycle assertions
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
    # Navigation assertions
    'NavigationResult',
    'assert_reaches_goal',
    'assert_follows_path',
    'assert_navigation_action_succeeds',
    'assert_costmap_contains_obstacle',
    # Service assertions
    'ServiceResult',
    'assert_service_available',
    'assert_action_server_available',
    'assert_node_running',
    'assert_nodes_running',
    'assert_parameter_exists',
    # Timing assertions
    'TimingResult',
    'assert_publish_rate',
    'assert_latency',
    'assert_transform_available',
    'assert_action_server_responsive',
    # Perception assertions
    'DetectionResult',
    'assert_object_detected',
    'assert_object_detected_by_class',
    'assert_min_objects_detected',
    'assert_region_clear',
    # Readiness checks
    'ReadinessCheck',
    'CheckResult',
    'CheckItemResult',
    'CheckStatus',
    'CheckCategory',
    'create_standard_check',
    # TopicObserver (FP fold combinator)
    'TopicObserver',
    'ObservationResult',
    'ParallelObserver',
    'SENSOR_QOS',
    'collect_messages',
    'count_messages',
    'latest_message',
    'track_max',
    'track_timestamps',
    # Stream properties (Hedgehog-inspired)
    'PropertyResult',
    'for_all_messages',
    'eventually',
    'monotonic',
    'all_of',
    'any_of',
    'negate',
    # Hypothesis property-based testing
    'sim_property',
    'PropertyFailure',
    'check_recorded_property',
    'check_recorded_eventually',
    'check_recorded_monotonic',
    'hypothesis_check_recorded',
    # Hypothesis strategies (ROS message generators)
    'point_strategy',
    'vector3_strategy',
    'quaternion_strategy',
    'yaw_quaternion_strategy',
    'pose_strategy',
    'twist_strategy',
    'twist_strategy_3d',
    'navigation_goal_2d',
    'waypoints_strategy',
    'duration_strategy',
    'rate_strategy',
    'threshold_strategy',
    'angle_strategy',
    'speed_strategy',
    'distance_strategy',
    # Test runner
    'get_test_runner',
    'get_test_registry',
]
