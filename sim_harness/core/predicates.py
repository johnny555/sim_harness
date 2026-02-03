# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Composable predicates for ROS message validation.

Each predicate is a function ``MsgT -> bool`` that can be combined
with ``all_of``, ``any_of``, and ``negate`` from ``stream_properties``.

Example:
    from sim_harness.core.predicates import (
        scan_has_min_points, scan_ranges_within, scan_nan_ratio_below,
    )
    from sim_harness.core.stream_properties import all_of, for_all_messages

    valid_lidar = all_of(
        scan_has_min_points(100),
        scan_ranges_within(0.1, 30.0),
        scan_nan_ratio_below(0.05),
    )
    result = for_all_messages(node, executor, "/scan", LaserScan,
                              predicate=valid_lidar, timeout_sec=5.0)
"""

import math
from typing import Callable, List


# ---------------------------------------------------------------------------
# LaserScan predicates
# ---------------------------------------------------------------------------

def scan_has_min_points(n: int) -> Callable:
    """Scan contains at least ``n`` finite (non-NaN, non-inf) range values."""
    def check(scan) -> bool:
        valid = sum(1 for r in scan.ranges if math.isfinite(r))
        return valid >= n
    return check


def scan_ranges_within(lo: float, hi: float) -> Callable:
    """All finite range values fall within [lo, hi]."""
    def check(scan) -> bool:
        return all(lo <= r <= hi for r in scan.ranges if math.isfinite(r))
    return check


def scan_nan_ratio_below(threshold: float) -> Callable:
    """Fraction of NaN values is below threshold (0.0 to 1.0)."""
    def check(scan) -> bool:
        if not scan.ranges:
            return False
        nan_count = sum(1 for r in scan.ranges if math.isnan(r))
        return nan_count / len(scan.ranges) < threshold
    return check


def scan_has_range_count(expected: int) -> Callable:
    """Scan has exactly ``expected`` range entries."""
    def check(scan) -> bool:
        return len(scan.ranges) == expected
    return check


# ---------------------------------------------------------------------------
# IMU predicates
# ---------------------------------------------------------------------------

def imu_accel_within(max_magnitude: float) -> Callable:
    """Linear acceleration magnitude is at or below max."""
    def check(imu) -> bool:
        a = imu.linear_acceleration
        return math.sqrt(a.x**2 + a.y**2 + a.z**2) <= max_magnitude
    return check


def imu_gyro_within(max_magnitude: float) -> Callable:
    """Angular velocity magnitude is at or below max."""
    def check(imu) -> bool:
        g = imu.angular_velocity
        return math.sqrt(g.x**2 + g.y**2 + g.z**2) <= max_magnitude
    return check


def imu_no_nan() -> Callable:
    """No NaN values in acceleration or angular velocity."""
    def check(imu) -> bool:
        a = imu.linear_acceleration
        g = imu.angular_velocity
        return all(math.isfinite(v) for v in [a.x, a.y, a.z, g.x, g.y, g.z])
    return check


# ---------------------------------------------------------------------------
# Image predicates
# ---------------------------------------------------------------------------

def image_has_data() -> Callable:
    """Image contains non-empty data."""
    return lambda img: len(img.data) > 0


def image_dimensions(width: int, height: int) -> Callable:
    """Image matches expected width and height."""
    return lambda img: img.width == width and img.height == height


def image_encoding(encoding: str) -> Callable:
    """Image uses expected encoding (e.g. 'rgb8', 'bgr8')."""
    return lambda img: img.encoding == encoding


# ---------------------------------------------------------------------------
# GPS / NavSatFix predicates
# ---------------------------------------------------------------------------

def gps_no_nan() -> Callable:
    """Latitude and longitude are not NaN."""
    def check(fix) -> bool:
        return not (math.isnan(fix.latitude) or math.isnan(fix.longitude))
    return check


def gps_in_bounds(
    min_lat: float, max_lat: float,
    min_lon: float, max_lon: float,
) -> Callable:
    """Coordinates fall within the specified bounding box."""
    def check(fix) -> bool:
        return (min_lat <= fix.latitude <= max_lat and
                min_lon <= fix.longitude <= max_lon)
    return check


def gps_has_fix() -> Callable:
    """GPS status indicates a valid fix (status >= 0)."""
    def check(fix) -> bool:
        return fix.status.status >= 0
    return check


# ---------------------------------------------------------------------------
# JointState predicates
# ---------------------------------------------------------------------------

def joints_present(expected_names: List[str]) -> Callable:
    """All expected joint names are present in the message."""
    def check(js) -> bool:
        return all(name in js.name for name in expected_names)
    return check


def joints_no_nan() -> Callable:
    """No NaN values in position, velocity, or effort arrays."""
    def check(js) -> bool:
        for arr in [js.position, js.velocity, js.effort]:
            if any(math.isnan(v) for v in arr):
                return False
        return True
    return check


# ---------------------------------------------------------------------------
# Odometry predicates
# ---------------------------------------------------------------------------

def odom_position_finite() -> Callable:
    """Odometry position components are all finite."""
    def check(odom) -> bool:
        p = odom.pose.pose.position
        return all(math.isfinite(v) for v in [p.x, p.y, p.z])
    return check


def odom_velocity_below(max_speed: float) -> Callable:
    """Linear velocity magnitude is below max_speed."""
    def check(odom) -> bool:
        v = odom.twist.twist.linear
        return math.sqrt(v.x**2 + v.y**2 + v.z**2) <= max_speed
    return check


def odom_in_region(
    min_x: float, max_x: float,
    min_y: float, max_y: float,
) -> Callable:
    """Odometry position is within a 2D bounding box."""
    def check(odom) -> bool:
        p = odom.pose.pose.position
        return min_x <= p.x <= max_x and min_y <= p.y <= max_y
    return check
