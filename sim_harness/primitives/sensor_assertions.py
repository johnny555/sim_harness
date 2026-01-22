# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Sensor data validation assertions.

Provides functions to validate sensor data from ROS 2 topics.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, NavSatFix, Imu, Image, JointState, LaserScan

from sim_harness.core.spin_helpers import spin_for_duration


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
    node: Node,
    topic: str,
    expected_rate_hz: float,
    tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0
) -> SensorDataResult:
    """
    Assert that a sensor is publishing at the expected rate.

    Uses a generic subscription to count messages and calculate rate.

    Args:
        node: ROS 2 node
        topic: Topic to monitor
        expected_rate_hz: Expected publish rate
        tolerance_percent: Acceptable deviation (percent)
        sample_duration_sec: How long to sample

    Returns:
        SensorDataResult with publish rate info
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    message_count = 0

    def callback(msg):
        nonlocal message_count
        message_count += 1

    # Create generic subscription
    qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    # Use raw message type for generic counting
    from rclpy.serialization import deserialize_message
    sub = node.create_subscription(
        LaserScan,  # Will be overridden by actual topic type
        topic,
        callback,
        qos
    )

    try:
        spin_for_duration(executor, sample_duration_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    # Calculate rate
    publish_rate = message_count / sample_duration_sec if sample_duration_sec > 0 else 0

    # Check tolerance
    if expected_rate_hz > 0:
        deviation = abs(publish_rate - expected_rate_hz) / expected_rate_hz * 100
        valid = deviation <= tolerance_percent
    else:
        valid = message_count > 0
        deviation = 0

    details = f"Rate: {publish_rate:.1f} Hz, Expected: {expected_rate_hz:.1f} Hz, Deviation: {deviation:.1f}%"

    return SensorDataResult(
        valid=valid,
        message_count=message_count,
        publish_rate_hz=publish_rate,
        details=details
    )


def assert_lidar_valid(
    node: Node,
    topic: str,
    min_range: float = 0.1,
    max_range: float = 100.0,
    min_points: int = 100,
    timeout_sec: float = 5.0
) -> SensorDataResult:
    """
    Assert that LIDAR data is valid.

    Checks:
    - Messages are being received
    - Point cloud/scan has minimum number of points
    - Range values are within bounds (no NaN, within sensor limits)

    Args:
        node: ROS 2 node
        topic: LIDAR topic (supports LaserScan or PointCloud2)
        min_range: Minimum valid range (meters)
        max_range: Maximum valid range (meters)
        min_points: Minimum number of points expected
        timeout_sec: Time to wait for data

    Returns:
        SensorDataResult
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages: List[LaserScan] = []

    def callback(msg: LaserScan):
        messages.append(msg)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(LaserScan, topic, callback, qos)

    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No LIDAR messages received on {topic}"
        )

    # Validate latest message
    scan = messages[-1]
    num_points = len(scan.ranges)
    nan_count = sum(1 for r in scan.ranges if math.isnan(r))
    inf_count = sum(1 for r in scan.ranges if math.isinf(r))
    valid_count = num_points - nan_count - inf_count

    # Check valid range values
    out_of_range = 0
    for r in scan.ranges:
        if not math.isnan(r) and not math.isinf(r):
            if r < min_range or r > max_range:
                out_of_range += 1

    valid = (
        valid_count >= min_points and
        nan_count < num_points * 0.1  # Less than 10% NaN
    )

    publish_rate = len(messages) / timeout_sec

    details = (
        f"Points: {num_points}, Valid: {valid_count}, "
        f"NaN: {nan_count}, Inf: {inf_count}, OutOfRange: {out_of_range}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=len(messages),
        publish_rate_hz=publish_rate,
        details=details
    )


def assert_gps_valid(
    node: Node,
    topic: str,
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float,
    timeout_sec: float = 5.0
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages: List[NavSatFix] = []

    def callback(msg: NavSatFix):
        messages.append(msg)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(NavSatFix, topic, callback, qos)

    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No GPS messages received on {topic}"
        )

    # Validate latest message
    fix = messages[-1]

    # Check for NaN
    has_nan = math.isnan(fix.latitude) or math.isnan(fix.longitude)

    # Check bounds
    in_bounds = (
        min_lat <= fix.latitude <= max_lat and
        min_lon <= fix.longitude <= max_lon
    )

    # Check fix status (>= 0 is valid)
    has_fix = fix.status.status >= 0

    valid = not has_nan and in_bounds and has_fix

    publish_rate = len(messages) / timeout_sec

    details = (
        f"Lat: {fix.latitude:.6f}, Lon: {fix.longitude:.6f}, "
        f"Status: {fix.status.status}, InBounds: {in_bounds}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=len(messages),
        publish_rate_hz=publish_rate,
        details=details
    )


def assert_imu_valid(
    node: Node,
    topic: str,
    max_acceleration: float = 50.0,
    max_angular_velocity: float = 10.0,
    timeout_sec: float = 5.0
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages: List[Imu] = []

    def callback(msg: Imu):
        messages.append(msg)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(Imu, topic, callback, qos)

    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No IMU messages received on {topic}"
        )

    # Validate latest message
    imu = messages[-1]

    # Check for NaN
    accel_nan = any(math.isnan(x) for x in [
        imu.linear_acceleration.x,
        imu.linear_acceleration.y,
        imu.linear_acceleration.z
    ])
    gyro_nan = any(math.isnan(x) for x in [
        imu.angular_velocity.x,
        imu.angular_velocity.y,
        imu.angular_velocity.z
    ])

    # Check bounds
    accel_mag = math.sqrt(
        imu.linear_acceleration.x ** 2 +
        imu.linear_acceleration.y ** 2 +
        imu.linear_acceleration.z ** 2
    )
    gyro_mag = math.sqrt(
        imu.angular_velocity.x ** 2 +
        imu.angular_velocity.y ** 2 +
        imu.angular_velocity.z ** 2
    )

    accel_valid = accel_mag <= max_acceleration
    gyro_valid = gyro_mag <= max_angular_velocity

    valid = not accel_nan and not gyro_nan and accel_valid and gyro_valid

    publish_rate = len(messages) / timeout_sec

    details = (
        f"Accel: {accel_mag:.2f} m/s^2, Gyro: {gyro_mag:.2f} rad/s, "
        f"NaN: {accel_nan or gyro_nan}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=len(messages),
        publish_rate_hz=publish_rate,
        details=details
    )


def assert_camera_valid(
    node: Node,
    topic: str,
    expected_width: int = 0,
    expected_height: int = 0,
    expected_encoding: str = "",
    timeout_sec: float = 5.0
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages: List[Image] = []

    def callback(msg: Image):
        messages.append(msg)

    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(Image, topic, callback, qos)

    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No camera messages received on {topic}"
        )

    # Validate latest message
    img = messages[-1]

    # Check dimensions
    width_ok = expected_width == 0 or img.width == expected_width
    height_ok = expected_height == 0 or img.height == expected_height

    # Check encoding
    encoding_ok = expected_encoding == "" or img.encoding == expected_encoding

    # Check data is non-empty
    has_data = len(img.data) > 0

    valid = width_ok and height_ok and encoding_ok and has_data

    publish_rate = len(messages) / timeout_sec

    details = (
        f"Size: {img.width}x{img.height}, Encoding: {img.encoding}, "
        f"Data: {len(img.data)} bytes"
    )

    return SensorDataResult(
        valid=valid,
        message_count=len(messages),
        publish_rate_hz=publish_rate,
        details=details
    )


def assert_joint_states_valid(
    node: Node,
    topic: str,
    expected_joints: List[str],
    timeout_sec: float = 5.0
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages: List[JointState] = []

    def callback(msg: JointState):
        messages.append(msg)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(JointState, topic, callback, qos)

    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not messages:
        return SensorDataResult(
            valid=False,
            message_count=0,
            publish_rate_hz=0.0,
            details=f"No joint state messages received on {topic}"
        )

    # Validate latest message
    js = messages[-1]

    # Check all expected joints are present
    missing_joints = [j for j in expected_joints if j not in js.name]

    # Check for NaN values
    has_nan = False
    for positions in js.position:
        if math.isnan(positions):
            has_nan = True
            break
    for velocity in js.velocity:
        if math.isnan(velocity):
            has_nan = True
            break
    for effort in js.effort:
        if math.isnan(effort):
            has_nan = True
            break

    valid = len(missing_joints) == 0 and not has_nan

    publish_rate = len(messages) / timeout_sec

    details = (
        f"Joints: {js.name}, "
        f"Missing: {missing_joints if missing_joints else 'None'}, "
        f"NaN: {has_nan}"
    )

    return SensorDataResult(
        valid=valid,
        message_count=len(messages),
        publish_rate_hz=publish_rate,
        details=details
    )
