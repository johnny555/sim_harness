# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Core test infrastructure for ROS 2 simulation testing."""

from sim_harness.core.test_fixture import SimTestFixture
from sim_harness.core.simulation_fixture import (
    SimulationTestFixture,
    simulation_manager,
    session_simulation,
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
    TestIsolationConfig,
)
from sim_harness.core.readiness_check import (
    ReadinessCheck,
    CheckResult,
    CheckItemResult,
    CheckStatus,
    CheckCategory,
    create_standard_check,
)
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

__all__ = [
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
    'TestIsolationConfig',
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
]
