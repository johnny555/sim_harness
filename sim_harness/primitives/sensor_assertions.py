# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Sensor data validation assertions.

Provides functions to validate sensor data from ROS 2 topics.

Implementation uses :class:`TopicObserver` for subscription lifecycle
and composable predicates from :mod:`sim_harness.core.predicates` for
validation logic. All public signatures are unchanged.
"""

import math
from dataclasses import dataclass
from typing import List

from sensor_msgs.msg import NavSatFix, Imu, Image, JointState, LaserScan

from sim_harness.core.topic_observer import (
    collect_messages,
    count_messages,
    SENSOR_QOS,
)
from sim_harness.core.stream_properties import all_of
from sim_harness.core.predicates import (
    scan_has_min_points,
    scan_ranges_within,
    scan_nan_ratio_below,
    imu_no_nan,
    imu_accel_within,
    imu_gyro_within,
    image_has_data,
    image_dimensions,
    image_encoding,
    gps_no_nan,
    gps_in_bounds,
    gps_has_fix,
    joints_present,
    joints_no_nan,
)


@dataclass
class SensorDataResult:
    """Result of a sensor data validation."""

    valid: bool
    """Whether the sensor data is valid."""

    message_count: int
    """Number of messages received during sampling."""

    publish_rate_hz: float
    """Measured publish rate (Hz)."""

    details: str
    """Human-readable details."""


def assert_sensor_publishing(
    node,
    topic: str,
    expected_rate_hz: float,
    msg_type: type = LaserScan,
    tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that a sensor is publishing at the expected rate.

    Args:
        node: ROS 2 node
        topic: Topic to monitor
        expected_rate_hz: Expected publish rate
        msg_type: Message type class for the topic (default: LaserScan)
        tolerance_percent: Acceptable deviation (percent)
        sample_duration_sec: How long to sample

    Returns:
        SensorDataResult with publish rate info
    """
    obs = count_messages(topic, msg_type, qos=SENSOR_QOS)
    result = obs.run_standalone(node, sample_duration_sec)

    message_count = result.value
    publish_rate = result.publish_rate_hz

    if expected_rate_hz > 0:
        deviation = abs(publish_rate - expected_rate_hz) / expected_rate_hz * 100
        valid = deviation <= tolerance_percent
    else:
        valid = message_count > 0
        deviation = 0

    details = (
        f"Rate: {publish_rate:.1f} Hz, Expected: {expected_rate_hz:.1f} Hz, "
        f"Deviation: {deviation:.1f}%"
    )

    return SensorDataResult(
        valid=valid,
        message_count=message_count,
        publish_rate_hz=publish_rate,
        details=details,
    )


def assert_lidar_valid(
    node,
    topic: str,
    min_range: float = 0.1,
    max_range: float = 100.0,
    min_points: int = 100,
    timeout_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that LIDAR data is valid.

    Checks:
    - Messages are being received
    - Scan has minimum number of valid points
    - Range values are within bounds (no NaN, within sensor limits)

    Args:
        node: ROS 2 node
        topic: LIDAR topic (LaserScan message type)
        min_range: Minimum valid range (meters)
        max_range: Maximum valid range (meters)
        min_points: Minimum number of valid (non-NaN, non-inf) points expected
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult with validation status and point statistics
    """
    obs = collect_messages(topic, LaserScan, qos=SENSOR_QOS)
    result = obs.run_standalone(node, timeout_sec)
    messages = result.value

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No LIDAR messages received on {topic}",
        )

    scan = messages[-1]

    valid = all_of(
        scan_has_min_points(min_points),
        scan_ranges_within(min_range, max_range),
        scan_nan_ratio_below(0.1),
    )(scan)

    # Diagnostic details
    num_points = len(scan.ranges)
    nan_count = sum(1 for r in scan.ranges if math.isnan(r))
    inf_count = sum(1 for r in scan.ranges if math.isinf(r))
    valid_count = num_points - nan_count - inf_count
    out_of_range = sum(
        1 for r in scan.ranges
        if math.isfinite(r) and (r < min_range or r > max_range)
    )

    details = (
        f"Points: {num_points}, Valid: {valid_count}, "
        f"NaN: {nan_count}, Inf: {inf_count}, OutOfRange: {out_of_range}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=result.message_count,
        publish_rate_hz=result.publish_rate_hz,
        details=details,
    )


def assert_gps_valid(
    node,
    topic: str,
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float,
    timeout_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that GPS data is valid and within a region.

    Checks:
    - Messages are being received
    - Coordinates are within specified bounds
    - Fix status is valid
    - No NaN values in coordinates

    Args:
        node: ROS 2 node
        topic: GPS topic
        min_lat: Minimum latitude (degrees)
        max_lat: Maximum latitude (degrees)
        min_lon: Minimum longitude (degrees)
        max_lon: Maximum longitude (degrees)
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult
    """
    obs = collect_messages(topic, NavSatFix, qos=SENSOR_QOS)
    result = obs.run_standalone(node, timeout_sec)
    messages = result.value

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No GPS messages received on {topic}",
        )

    fix = messages[-1]

    valid = all_of(
        gps_no_nan(),
        gps_in_bounds(min_lat, max_lat, min_lon, max_lon),
        gps_has_fix(),
    )(fix)

    in_bounds = (
        min_lat <= fix.latitude <= max_lat
        and min_lon <= fix.longitude <= max_lon
    )

    details = (
        f"Lat: {fix.latitude:.6f}, Lon: {fix.longitude:.6f}, "
        f"Status: {fix.status.status}, InBounds: {in_bounds}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=result.message_count,
        publish_rate_hz=result.publish_rate_hz,
        details=details,
    )


def assert_imu_valid(
    node,
    topic: str,
    max_acceleration: float = 50.0,
    max_angular_velocity: float = 10.0,
    timeout_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that IMU data is valid.

    Checks:
    - Messages are being received
    - Acceleration values are reasonable (not extreme)
    - Angular velocity values are reasonable
    - No NaN values

    Args:
        node: ROS 2 node
        topic: IMU topic
        max_acceleration: Maximum expected acceleration (m/s^2)
        max_angular_velocity: Maximum expected angular velocity (rad/s)
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult
    """
    obs = collect_messages(topic, Imu, qos=SENSOR_QOS)
    result = obs.run_standalone(node, timeout_sec)
    messages = result.value

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No IMU messages received on {topic}",
        )

    imu = messages[-1]

    valid = all_of(
        imu_no_nan(),
        imu_accel_within(max_acceleration),
        imu_gyro_within(max_angular_velocity),
    )(imu)

    a = imu.linear_acceleration
    g = imu.angular_velocity
    accel_mag = math.sqrt(a.x**2 + a.y**2 + a.z**2)
    gyro_mag = math.sqrt(g.x**2 + g.y**2 + g.z**2)
    has_nan = not all(
        math.isfinite(v) for v in [a.x, a.y, a.z, g.x, g.y, g.z]
    )

    details = (
        f"Accel: {accel_mag:.2f} m/s^2, Gyro: {gyro_mag:.2f} rad/s, "
        f"NaN: {has_nan}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=result.message_count,
        publish_rate_hz=result.publish_rate_hz,
        details=details,
    )


def assert_camera_valid(
    node,
    topic: str,
    expected_width: int = 0,
    expected_height: int = 0,
    expected_encoding: str = "",
    timeout_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that camera images are valid.

    Checks:
    - Messages are being received
    - Image dimensions match expected (if specified)
    - Encoding matches (if specified)
    - Image data is non-empty

    Args:
        node: ROS 2 node
        topic: Camera image topic
        expected_width: Expected image width (0 = any)
        expected_height: Expected image height (0 = any)
        expected_encoding: Expected encoding (empty = any)
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult
    """
    obs = collect_messages(topic, Image, qos=SENSOR_QOS)
    result = obs.run_standalone(node, timeout_sec)
    messages = result.value

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No camera messages received on {topic}",
        )

    img = messages[-1]

    # Build predicate dynamically based on which checks are requested
    checks = [image_has_data()]
    if expected_width > 0 and expected_height > 0:
        checks.append(image_dimensions(expected_width, expected_height))
    if expected_encoding:
        checks.append(image_encoding(expected_encoding))

    valid = all_of(*checks)(img)

    details = (
        f"Size: {img.width}x{img.height}, Encoding: {img.encoding}, "
        f"Data: {len(img.data)} bytes"
    )

    return SensorDataResult(
        valid=valid,
        message_count=result.message_count,
        publish_rate_hz=result.publish_rate_hz,
        details=details,
    )


def assert_joint_states_valid(
    node,
    topic: str,
    expected_joints: List[str],
    timeout_sec: float = 5.0,
) -> SensorDataResult:
    """
    Assert that joint states contain expected joints.

    Checks:
    - Messages are being received
    - All expected joints are present
    - Position/velocity/effort values are not NaN

    Args:
        node: ROS 2 node
        topic: Joint states topic
        expected_joints: List of joint names that must be present
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult
    """
    obs = collect_messages(topic, JointState, qos=SENSOR_QOS)
    result = obs.run_standalone(node, timeout_sec)
    messages = result.value

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No joint state messages received on {topic}",
        )

    js = messages[-1]

    valid = all_of(
        joints_present(expected_joints),
        joints_no_nan(),
    )(js)

    missing_joints = [j for j in expected_joints if j not in js.name]

    details = (
        f"Joints: {js.name}, "
        f"Missing: {missing_joints if missing_joints else 'None'}, "
        f"NaN: {not joints_no_nan()(js)}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=result.message_count,
        publish_rate_hz=result.publish_rate_hz,
        details=details,
    )
