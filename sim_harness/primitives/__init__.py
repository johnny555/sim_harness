# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Primitive assertion functions for ROS 2 simulation testing.

Provides DSL-style assertions for sensors, vehicles, navigation, and more.
"""

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

__all__ = [
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
]
