# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Core check functions for ROS 2 simulation testing.

Result types and check functions are split into submodules for navigability:
:mod:`._base` (result types + helpers), :mod:`.services`, :mod:`.sensors`,
:mod:`.motion`. They are all re-exported here so existing callers keep
working: ``from sim_harness.checks import check_lidar_valid``.

When called from a :class:`SimTestFixture` test (where the node is already
attached to a background-spinning executor), functions automatically detect
this and use sleep-based waiting instead of creating a new executor.

Example::

    from sim_harness.checks import check_lidar_valid, check_sensor_publishing

    result = check_lidar_valid(node, '/scan')
    assert result.ok, result.details
"""

# Result types
from ._base import (
    MovementResult,
    SensorDataResult,
    ServiceResult,
    TimingResult,
    VelocityResult,
)

# Service / action / node / parameter / TF checks
from .services import (
    check_action_server_available,
    check_action_server_responsive,
    check_node_running,
    check_nodes_running,
    check_parameter_exists,
    check_service_available,
    check_transform_available,
)

# Sensor + timing checks
from .sensors import (
    check_camera_valid,
    check_gps_valid,
    check_imu_valid,
    check_joint_states_valid,
    check_latency,
    check_lidar_valid,
    check_publish_rate,
    check_sensor_publishing,
)

# Vehicle motion checks
from .motion import (
    check_vehicle_in_region,
    check_vehicle_moved,
    check_vehicle_moved_with_ground_truth,
    check_vehicle_orientation,
    check_vehicle_stationary,
    check_vehicle_velocity,
)

__all__ = [
    # Result types
    "MovementResult", "SensorDataResult", "ServiceResult",
    "TimingResult", "VelocityResult",
    # Services
    "check_action_server_available", "check_action_server_responsive",
    "check_node_running", "check_nodes_running",
    "check_parameter_exists", "check_service_available",
    "check_transform_available",
    # Sensors
    "check_camera_valid", "check_gps_valid", "check_imu_valid",
    "check_joint_states_valid", "check_latency", "check_lidar_valid",
    "check_publish_rate", "check_sensor_publishing",
    # Motion
    "check_vehicle_in_region", "check_vehicle_moved",
    "check_vehicle_moved_with_ground_truth", "check_vehicle_orientation",
    "check_vehicle_stationary", "check_vehicle_velocity",
]
