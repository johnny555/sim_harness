# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Sensor and timing checks (LIDAR, GPS, IMU, camera, joint states, rates, latency)."""

import math
import time
from typing import List, Type

from rclpy.node import Node

from sim_harness.spin import ExecutorContext

from ._base import MsgT, SensorDataResult, TimingResult, _collect


def check_sensor_publishing(
    node: Node, topic: str, expected_rate_hz: float,
    msg_type=None, tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0, executor=None,
    max_attempts: int = 1,
) -> SensorDataResult:
    """Check that a topic publishes at roughly *expected_rate_hz*.

    Args:
        max_attempts: Number of attempts before returning failure.
            Default ``1`` preserves backward compatibility.  On retry,
            a 1 s pause allows DDS discovery to catch up.
    """
    if msg_type is None:
        from sensor_msgs.msg import LaserScan
        msg_type = LaserScan
    result = SensorDataResult()
    for attempt in range(max_attempts):
        msgs = _collect(node, topic, msg_type, sample_duration_sec, executor=executor)
        rate = len(msgs) / sample_duration_sec if sample_duration_sec > 0 else 0
        tol = expected_rate_hz * tolerance_percent / 100
        ok = abs(rate - expected_rate_hz) <= tol
        result = SensorDataResult(
            valid=ok, message_count=len(msgs), publish_rate_hz=rate,
            details=f"Rate {rate:.1f} Hz (expected {expected_rate_hz:.1f} +/- {tol:.1f})",
        )
        if ok:
            return result
        if attempt < max_attempts - 1:
            node.get_logger().debug(
                f"check_sensor_publishing({topic}) attempt {attempt + 1}/{max_attempts} "
                f"failed: {result.details}, retrying...")
            time.sleep(1.0)
    return result


def check_lidar_valid(
    node: Node, topic: str, min_range: float = 0.1, max_range: float = 100.0,
    min_points: int = 100, timeout_sec: float = 5.0, executor=None,
) -> SensorDataResult:
    """Validate LIDAR scans have enough in-range points."""
    from sensor_msgs.msg import LaserScan
    msgs = _collect(node, topic, LaserScan, timeout_sec, executor=executor)
    if not msgs:
        return SensorDataResult(details="No LaserScan messages received")
    bad = 0
    for scan in msgs:
        pts = sum(1 for r in scan.ranges if math.isfinite(r) and min_range <= r <= max_range)
        if pts < min_points:
            bad += 1
    rate = len(msgs) / timeout_sec if timeout_sec > 0 else 0
    ok = bad == 0
    return SensorDataResult(
        valid=ok, message_count=len(msgs), publish_rate_hz=rate,
        details=f"{len(msgs) - bad}/{len(msgs)} scans valid ({min_points}+ pts in [{min_range}, {max_range}])",
    )


def check_gps_valid(
    node: Node, topic: str, min_lat: float, max_lat: float,
    min_lon: float, max_lon: float, timeout_sec: float = 5.0,
    executor=None,
) -> SensorDataResult:
    """Validate GPS fixes are in-bounds and not NaN."""
    from sensor_msgs.msg import NavSatFix
    msgs = _collect(node, topic, NavSatFix, timeout_sec, executor=executor)
    if not msgs:
        return SensorDataResult(details="No NavSatFix messages received")
    for fix in msgs:
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="GPS contains NaN",
            )
        if not (min_lat <= fix.latitude <= max_lat and min_lon <= fix.longitude <= max_lon):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"GPS out of bounds: ({fix.latitude:.6f}, {fix.longitude:.6f})",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} fixes valid",
    )


def check_imu_valid(
    node: Node, topic: str, max_acceleration: float = 50.0,
    max_angular_velocity: float = 10.0, timeout_sec: float = 5.0,
    executor=None,
) -> SensorDataResult:
    """Validate IMU readings are finite and within bounds."""
    from sensor_msgs.msg import Imu
    msgs = _collect(node, topic, Imu, timeout_sec, executor=executor)
    if not msgs:
        return SensorDataResult(details="No Imu messages received")
    for imu in msgs:
        a = imu.linear_acceleration
        g = imu.angular_velocity
        vals = [a.x, a.y, a.z, g.x, g.y, g.z]
        if not all(math.isfinite(v) for v in vals):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="IMU contains NaN",
            )
        if max(abs(a.x), abs(a.y), abs(a.z)) > max_acceleration:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Acceleration exceeds {max_acceleration} m/s^2",
            )
        if max(abs(g.x), abs(g.y), abs(g.z)) > max_angular_velocity:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Angular velocity exceeds {max_angular_velocity} rad/s",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} IMU msgs valid",
    )


def check_camera_valid(
    node: Node, topic: str, expected_width: int = 0, expected_height: int = 0,
    expected_encoding: str = "", timeout_sec: float = 5.0, executor=None,
) -> SensorDataResult:
    """Validate camera images have data and optional dimension/encoding checks."""
    from sensor_msgs.msg import Image
    msgs = _collect(node, topic, Image, timeout_sec, executor=executor)
    if not msgs:
        return SensorDataResult(details="No Image messages received")
    for img in msgs:
        if len(img.data) == 0:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="Empty image data",
            )
        if expected_width and img.width != expected_width:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Width {img.width} != {expected_width}",
            )
        if expected_height and img.height != expected_height:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Height {img.height} != {expected_height}",
            )
        if expected_encoding and img.encoding != expected_encoding:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Encoding {img.encoding!r} != {expected_encoding!r}",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} images valid",
    )


def check_joint_states_valid(
    node: Node, topic: str, expected_joints: List[str], timeout_sec: float = 5.0,
    executor=None,
) -> SensorDataResult:
    """Validate joint state messages contain expected joints with finite values."""
    from sensor_msgs.msg import JointState
    msgs = _collect(node, topic, JointState, timeout_sec, executor=executor)
    if not msgs:
        return SensorDataResult(details="No JointState messages received")
    for js in msgs:
        missing = [j for j in expected_joints if j not in js.name]
        if missing:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Missing joints: {missing}",
            )
        if not all(math.isfinite(v) for v in js.position):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="Joint positions contain NaN",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} joint state msgs valid",
    )


def check_publish_rate(
    node: Node, topic: str, msg_type: Type[MsgT], expected_rate_hz: float,
    tolerance_percent: float = 10.0, sample_duration_sec: float = 5.0,
    executor=None,
) -> TimingResult:
    """Measure topic publish rate and check it matches expected."""
    timestamps: List[float] = []

    def on_msg(_msg):
        timestamps.append(time.monotonic())

    sub = node.create_subscription(msg_type, topic, on_msg, 10)
    try:
        with ExecutorContext(node, executor) as ec:
            ec.wait(sample_duration_sec)
    finally:
        node.destroy_subscription(sub)

    if len(timestamps) < 2:
        return TimingResult(details="Not enough messages to compute rate")

    intervals = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    avg_interval = sum(intervals) / len(intervals)
    rate = 1.0 / avg_interval if avg_interval > 0 else 0
    tol = expected_rate_hz * tolerance_percent / 100
    ok = abs(rate - expected_rate_hz) <= tol
    return TimingResult(
        within_bounds=ok, measured_rate_hz=rate,
        details=f"Rate {rate:.1f} Hz (expected {expected_rate_hz:.1f} +/- {tol:.1f})",
    )


def check_latency(
    node: Node, topic: str, msg_type: Type[MsgT],
    max_latency_ms: float, sample_duration_sec: float = 5.0,
    executor=None,
) -> TimingResult:
    """Measure message latency (header stamp vs receive time)."""
    latencies: List[float] = []

    def on_msg(msg):
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = node.get_clock().now().nanoseconds * 1e-9
            latencies.append((now - stamp) * 1000)

    sub = node.create_subscription(msg_type, topic, on_msg, 10)
    try:
        with ExecutorContext(node, executor) as ec:
            ec.wait(sample_duration_sec)
    finally:
        node.destroy_subscription(sub)

    if not latencies:
        return TimingResult(details="No messages with header received")

    mn, mx, avg = min(latencies), max(latencies), sum(latencies) / len(latencies)
    ok = mx <= max_latency_ms
    return TimingResult(
        within_bounds=ok, min_latency_ms=mn, max_latency_ms=mx, avg_latency_ms=avg,
        details=f"Latency min={mn:.1f} max={mx:.1f} avg={avg:.1f} ms (limit {max_latency_ms})",
    )
