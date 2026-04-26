# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Vehicle motion checks (movement, stationarity, velocity, region, orientation)."""

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node

from sim_harness.spin import ExecutorContext

from ._base import (
    MovementResult, VelocityResult,
    _collect, _dist3, _pos, _speed, _yaw,
)


def check_vehicle_moved(
    node: Node, vehicle_id: str, min_distance: float, velocity: float = 1.0,
    timeout_sec: float = 10.0, odom_topic: Optional[str] = None,
    cmd_vel_topic: Optional[str] = None, use_twist_stamped: bool = True,
    executor=None,
) -> MovementResult:
    """Publish a velocity command and verify the robot moves."""
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, TwistStamped

    otopic = odom_topic or f"/{vehicle_id}/odom"
    ctopic = cmd_vel_topic or f"/{vehicle_id}/cmd_vel"

    positions: list = []
    sub = node.create_subscription(Odometry, otopic, lambda m: positions.append(_pos(m)), 10)
    pub_type = TwistStamped if use_twist_stamped else Twist
    pub = node.create_publisher(pub_type, ctopic, 10)

    try:
        with ExecutorContext(node, executor) as ec:
            end = time.monotonic() + timeout_sec
            while time.monotonic() < end:
                if use_twist_stamped:
                    cmd = TwistStamped()
                    cmd.header.stamp = node.get_clock().now().to_msg()
                    cmd.twist.linear.x = velocity
                else:
                    cmd = Twist()
                    cmd.linear.x = velocity
                pub.publish(cmd)
                ec.spin_once(timeout_sec=0.1)

            # Stop
            stop = TwistStamped() if use_twist_stamped else Twist()
            if use_twist_stamped:
                stop.header.stamp = node.get_clock().now().to_msg()
            pub.publish(stop)
            ec.wait(0.5)

        if len(positions) < 2:
            return MovementResult(details="Not enough odometry messages")
        start, final = positions[0], positions[-1]
        d = _dist3(start, final)
        return MovementResult(
            success=d >= min_distance, distance_moved=d,
            start_position=start, end_position=final,
            details=f"Moved {d:.3f}m (need {min_distance:.3f}m)",
        )
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)


def check_vehicle_stationary(
    node: Node, vehicle_id: str, velocity_threshold: float = 0.01,
    duration_sec: float = 2.0, odom_topic: Optional[str] = None,
    executor=None,
) -> bool:
    """Check that the robot is nearly stationary over a duration."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, duration_sec, executor=executor)
    return all(_speed(m) <= velocity_threshold for m in msgs) if msgs else False


def check_vehicle_velocity(
    node: Node, vehicle_id: str, target_velocity: float, tolerance: float = 0.1,
    timeout_sec: float = 5.0, odom_topic: Optional[str] = None,
    executor=None,
) -> VelocityResult:
    """Check that observed velocity is within tolerance of target."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec, executor=executor)
    if not msgs:
        return VelocityResult(details="No odometry messages")
    speeds = [_speed(m) for m in msgs]
    avg = sum(speeds) / len(speeds)
    ok = abs(avg - target_velocity) <= tolerance
    return VelocityResult(
        success=ok, measured_velocity=avg,
        details=f"Avg velocity {avg:.3f} m/s (target {target_velocity:.3f} +/- {tolerance})",
    )


def check_vehicle_in_region(
    node: Node, vehicle_id: str,
    min_bounds: Tuple[float, float, float],
    max_bounds: Tuple[float, float, float],
    timeout_sec: float = 5.0, odom_topic: Optional[str] = None,
    executor=None,
) -> bool:
    """Check that the robot stays within a bounding box."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec, executor=executor)
    if not msgs:
        return False
    for m in msgs:
        p = _pos(m)
        for i in range(3):
            if not (min_bounds[i] <= p[i] <= max_bounds[i]):
                return False
    return True


def check_vehicle_orientation(
    node: Node, vehicle_id: str, expected_yaw: float,
    tolerance_rad: float = 0.1, timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None, executor=None,
) -> bool:
    """Check that the robot's yaw matches expected within tolerance."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec, executor=executor)
    if not msgs:
        return False
    q = msgs[-1].pose.pose.orientation
    actual = _yaw(q)
    diff = abs(math.atan2(math.sin(actual - expected_yaw), math.cos(actual - expected_yaw)))
    return diff <= tolerance_rad


def check_vehicle_moved_with_ground_truth(
    node: Node, vehicle_id: str, gazebo_model_name: str,
    min_distance: float, velocity: float = 1.0, timeout_sec: float = 10.0,
    odom_topic: Optional[str] = None, cmd_vel_topic: Optional[str] = None,
    use_twist_stamped: bool = True, world_name: str = "empty",
    odom_tolerance: float = 0.5, executor=None,
) -> MovementResult:
    """Like check_vehicle_moved but also compares against Gazebo ground truth."""
    result = check_vehicle_moved(
        node, vehicle_id, min_distance, velocity, timeout_sec,
        odom_topic, cmd_vel_topic, use_twist_stamped, executor=executor,
    )
    try:
        from sim_harness.simulator.gazebo_ground_truth import GazeboGroundTruth
        gt = GazeboGroundTruth(node, world_name=world_name)
        gt_pos = gt.get_model_position(gazebo_model_name)
        if gt_pos:
            result.ground_truth_end = gt_pos
            if result.ground_truth_start:
                result.ground_truth_distance = _dist3(result.ground_truth_start, gt_pos)
                result.odom_error = abs(result.distance_moved - result.ground_truth_distance)
                if result.odom_error > odom_tolerance:
                    result.details += f"; odom error {result.odom_error:.3f}m > {odom_tolerance}m"
    except ImportError:
        result.details += "; ground truth unavailable (gazebo_ground_truth not installed)"
    return result
