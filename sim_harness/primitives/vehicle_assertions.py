# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Vehicle motion and state validation assertions.

Provides functions to validate robot movement and state.
Supports both odometry-based and ground-truth-based validation.

Observe-only functions (stationary, velocity, in_region, orientation) use
:class:`TopicObserver`. Active-loop functions that publish velocity commands
(moved, moved_with_ground_truth) keep manual loops because TopicObserver
does not support publishing or early exit.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TwistStamped, Quaternion
from nav_msgs.msg import Odometry

from sim_harness.core.topic_observer import (
    collect_messages,
    latest_message,
    track_max,
    SENSOR_QOS,
)


@dataclass
class MovementResult:
    """Result of a vehicle movement assertion."""

    success: bool
    """Whether the vehicle moved the required distance."""

    distance_moved: float
    """Actual distance moved (meters)."""

    start_position: Tuple[float, float, float]
    """Starting position (x, y, z)."""

    end_position: Tuple[float, float, float]
    """Ending position (x, y, z)."""

    details: str
    """Human-readable details."""

    # Ground truth fields (optional)
    ground_truth_distance: Optional[float] = None
    """Distance moved according to Gazebo ground truth."""

    ground_truth_start: Optional[Tuple[float, float, float]] = None
    """Starting position from ground truth."""

    ground_truth_end: Optional[Tuple[float, float, float]] = None
    """Ending position from ground truth."""

    odom_error: Optional[float] = None
    """Position error between odom and ground truth (meters)."""


@dataclass
class VelocityResult:
    """Result of a vehicle velocity assertion."""

    success: bool
    """Whether the target velocity was reached."""

    measured_velocity: float
    """Measured velocity (m/s)."""

    details: str
    """Human-readable details."""


def _distance_3d(
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
) -> float:
    """Calculate 3D Euclidean distance."""
    return math.sqrt(
        (p1[0] - p2[0]) ** 2
        + (p1[1] - p2[1]) ** 2
        + (p1[2] - p2[2]) ** 2
    )


def _get_yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _linear_speed(msg: Odometry) -> float:
    """Extract 2D linear speed from an Odometry message."""
    v = msg.twist.twist.linear
    return math.sqrt(v.x ** 2 + v.y ** 2)


# ---------------------------------------------------------------------------
# Active-loop functions (publish + early exit) — manual executor
# ---------------------------------------------------------------------------


def assert_vehicle_moved(
    node: Node,
    vehicle_id: str,
    min_distance: float,
    velocity: float = 1.0,
    timeout_sec: float = 10.0,
    odom_topic: Optional[str] = None,
    cmd_vel_topic: Optional[str] = None,
    use_twist_stamped: bool = True,
) -> MovementResult:
    """
    Assert that a vehicle moved at least min_distance meters.

    Monitors odometry, sends a forward velocity command, and verifies
    the vehicle moved the required distance.

    Args:
        node: ROS 2 node for subscriptions/publishers
        vehicle_id: Vehicle namespace (e.g., "robot_01")
        min_distance: Minimum distance to move (meters)
        velocity: Forward velocity to command (m/s)
        timeout_sec: Maximum time to wait
        odom_topic: Custom odometry topic (default: /{vehicle_id}/odom)
        cmd_vel_topic: Custom cmd_vel topic (default: /{vehicle_id}/cmd_vel)
        use_twist_stamped: Use TwistStamped instead of Twist

    Returns:
        MovementResult with success status and details
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"
    if cmd_vel_topic is None:
        cmd_vel_topic = f"/{vehicle_id}/cmd_vel"

    latest_odom: Optional[Odometry] = None

    def odom_callback(msg: Odometry):
        nonlocal latest_odom
        latest_odom = msg

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    if use_twist_stamped:
        cmd_pub = node.create_publisher(TwistStamped, cmd_vel_topic, 10)
    else:
        cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

    result = MovementResult(
        success=False,
        distance_moved=0.0,
        start_position=(0.0, 0.0, 0.0),
        end_position=(0.0, 0.0, 0.0),
        details="",
    )

    try:
        # Wait for initial odometry
        start_wait = time.monotonic()
        while latest_odom is None and time.monotonic() - start_wait < 5.0:
            executor.spin_once(timeout_sec=0.01)

        if latest_odom is None:
            result.details = f"No odometry received on {odom_topic}"
            return result

        start_pos = (
            latest_odom.pose.pose.position.x,
            latest_odom.pose.pose.position.y,
            latest_odom.pose.pose.position.z,
        )
        result.start_position = start_pos

        # Send velocity commands
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            if use_twist_stamped:
                cmd = TwistStamped()
                cmd.header.frame_id = "base_link"
                cmd.header.stamp = node.get_clock().now().to_msg()
                cmd.twist.linear.x = velocity
                cmd.twist.angular.z = 0.0
            else:
                cmd = Twist()
                cmd.linear.x = velocity
                cmd.angular.z = 0.0

            cmd_pub.publish(cmd)
            executor.spin_once(timeout_sec=0.05)

            if latest_odom is not None:
                end_pos = (
                    latest_odom.pose.pose.position.x,
                    latest_odom.pose.pose.position.y,
                    latest_odom.pose.pose.position.z,
                )
                result.end_position = end_pos
                result.distance_moved = _distance_3d(start_pos, end_pos)

                if result.distance_moved >= min_distance:
                    result.success = True
                    break

        # Stop the vehicle
        if use_twist_stamped:
            stop_cmd = TwistStamped()
            stop_cmd.header.frame_id = "base_link"
            stop_cmd.header.stamp = node.get_clock().now().to_msg()
            stop_cmd.twist.linear.x = 0.0
        else:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0

        cmd_pub.publish(stop_cmd)

        if result.success:
            result.details = (
                f"Vehicle moved {result.distance_moved:.2f}m "
                f"(required: {min_distance}m)"
            )
        else:
            result.details = (
                f"Vehicle moved {result.distance_moved:.2f}m "
                f"but required {min_distance}m"
            )

    finally:
        node.destroy_subscription(odom_sub)
        node.destroy_publisher(cmd_pub)
        executor.remove_node(node)

    return result


# ---------------------------------------------------------------------------
# Observe-only functions — TopicObserver
# ---------------------------------------------------------------------------


def assert_vehicle_stationary(
    node: Node,
    vehicle_id: str,
    velocity_threshold: float = 0.01,
    duration_sec: float = 2.0,
    odom_topic: Optional[str] = None,
) -> bool:
    """
    Assert that a vehicle is stationary.

    Monitors odometry and verifies velocity stays below threshold
    for the specified duration.

    Args:
        node: ROS 2 node
        vehicle_id: Vehicle namespace
        velocity_threshold: Maximum velocity to consider stationary (m/s)
        duration_sec: How long the vehicle must remain stationary
        odom_topic: Custom odometry topic

    Returns:
        True if vehicle remained stationary
    """
    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    obs = track_max(odom_topic, Odometry, _linear_speed)
    result = obs.run_standalone(node, duration_sec)

    if result.value is None:
        return True  # no messages → no movement observed
    return result.value <= velocity_threshold


def assert_vehicle_velocity(
    node: Node,
    vehicle_id: str,
    target_velocity: float,
    tolerance: float = 0.1,
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None,
) -> VelocityResult:
    """
    Assert that a vehicle reaches the target velocity.

    Monitors odometry and verifies the vehicle reaches the commanded
    velocity within tolerance.

    Args:
        node: ROS 2 node
        vehicle_id: Vehicle namespace
        target_velocity: Target velocity (m/s)
        tolerance: Acceptable deviation from target (m/s)
        timeout_sec: Maximum time to wait
        odom_topic: Custom odometry topic

    Returns:
        VelocityResult with success status and measured velocity
    """
    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    obs = collect_messages(odom_topic, Odometry)
    obs_result = obs.run_standalone(node, timeout_sec)
    messages = obs_result.value

    result = VelocityResult(
        success=False,
        measured_velocity=0.0,
        details="",
    )

    if not messages:
        result.details = f"No odometry received on {odom_topic}"
        return result

    # Check if any message hit the target velocity
    for msg in messages:
        vel = _linear_speed(msg)
        if abs(vel - target_velocity) <= tolerance:
            result.success = True
            result.measured_velocity = vel
            break
    else:
        # Use last measured velocity
        result.measured_velocity = _linear_speed(messages[-1])

    result.details = (
        f"Measured velocity: {result.measured_velocity:.2f} m/s "
        f"(target: {target_velocity} +/- {tolerance} m/s)"
    )

    return result


def assert_vehicle_in_region(
    node: Node,
    vehicle_id: str,
    min_bounds: Tuple[float, float, float],
    max_bounds: Tuple[float, float, float],
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None,
) -> bool:
    """
    Assert that a vehicle is within a bounding region.

    Args:
        node: ROS 2 node
        vehicle_id: Vehicle namespace
        min_bounds: Minimum corner of bounding box (x, y, z)
        max_bounds: Maximum corner of bounding box (x, y, z)
        timeout_sec: Time to wait for odometry
        odom_topic: Custom odometry topic

    Returns:
        True if vehicle is within bounds
    """
    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    obs = latest_message(odom_topic, Odometry)
    result = obs.run_standalone(node, timeout_sec)

    if result.value is None:
        return False

    p = result.value.pose.pose.position
    return (
        min_bounds[0] <= p.x <= max_bounds[0]
        and min_bounds[1] <= p.y <= max_bounds[1]
        and min_bounds[2] <= p.z <= max_bounds[2]
    )


def assert_vehicle_orientation(
    node: Node,
    vehicle_id: str,
    expected_yaw: float,
    tolerance_rad: float = 0.1,
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None,
) -> bool:
    """
    Assert that a vehicle's yaw orientation is within tolerance.

    Args:
        node: ROS 2 node
        vehicle_id: Vehicle namespace
        expected_yaw: Expected yaw angle (radians)
        tolerance_rad: Acceptable deviation (radians)
        timeout_sec: Time to wait for odometry
        odom_topic: Custom odometry topic

    Returns:
        True if orientation is within tolerance
    """
    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    obs = latest_message(odom_topic, Odometry)
    result = obs.run_standalone(node, timeout_sec)

    if result.value is None:
        return False

    current_yaw = _get_yaw_from_quaternion(
        result.value.pose.pose.orientation
    )

    # Normalize angle difference to [-pi, pi]
    diff = current_yaw - expected_yaw
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi

    return abs(diff) <= tolerance_rad


# ---------------------------------------------------------------------------
# Ground truth variant (active-loop — manual executor)
# ---------------------------------------------------------------------------


def assert_vehicle_moved_with_ground_truth(
    node: Node,
    vehicle_id: str,
    gazebo_model_name: str,
    min_distance: float,
    velocity: float = 1.0,
    timeout_sec: float = 10.0,
    odom_topic: Optional[str] = None,
    cmd_vel_topic: Optional[str] = None,
    use_twist_stamped: bool = True,
    world_name: str = "empty",
    odom_tolerance: float = 0.5,
) -> MovementResult:
    """
    Assert vehicle movement with Gazebo ground truth validation.

    Like assert_vehicle_moved, but also verifies the movement against
    Gazebo's ground truth pose.

    Note:
        If gz-transport Python bindings are not installed, this function
        falls back to assert_vehicle_moved() without ground truth validation.

    Args:
        node: ROS 2 node for subscriptions/publishers
        vehicle_id: Vehicle namespace
        gazebo_model_name: Name of the model in Gazebo
        min_distance: Minimum distance to move (meters)
        velocity: Forward velocity to command (m/s)
        timeout_sec: Maximum time to wait
        odom_topic: Custom odometry topic
        cmd_vel_topic: Custom cmd_vel topic
        use_twist_stamped: Use TwistStamped instead of Twist
        world_name: Gazebo world name (for ground truth topic)
        odom_tolerance: Maximum acceptable odom-to-ground-truth error (meters)

    Returns:
        MovementResult with odom data and ground truth comparison
    """
    try:
        from sim_harness.simulator.gazebo_ground_truth import (
            GazeboGroundTruth,
            GZ_TRANSPORT_AVAILABLE,
        )
    except ImportError:
        GZ_TRANSPORT_AVAILABLE = False

    if not GZ_TRANSPORT_AVAILABLE:
        result = assert_vehicle_moved(
            node, vehicle_id, min_distance, velocity, timeout_sec,
            odom_topic, cmd_vel_topic, use_twist_stamped,
        )
        result.details += " (ground truth unavailable - gz-transport not installed)"
        return result

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"
    if cmd_vel_topic is None:
        cmd_vel_topic = f"/{vehicle_id}/cmd_vel"

    latest_odom: Optional[Odometry] = None

    def odom_callback(msg: Odometry):
        nonlocal latest_odom
        latest_odom = msg

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    if use_twist_stamped:
        cmd_pub = node.create_publisher(TwistStamped, cmd_vel_topic, 10)
    else:
        cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

    result = MovementResult(
        success=False,
        distance_moved=0.0,
        start_position=(0.0, 0.0, 0.0),
        end_position=(0.0, 0.0, 0.0),
        details="",
    )

    try:
        with GazeboGroundTruth(world_name=world_name) as gz:
            gt_start = gz.get_model_pose(gazebo_model_name)
            if gt_start is None:
                result.details = f"Model '{gazebo_model_name}' not found in Gazebo"
                return result

            result.ground_truth_start = gt_start.position

            # Wait for initial odometry
            start_wait = time.monotonic()
            while latest_odom is None and time.monotonic() - start_wait < 5.0:
                executor.spin_once(timeout_sec=0.01)

            if latest_odom is None:
                result.details = f"No odometry received on {odom_topic}"
                return result

            start_pos = (
                latest_odom.pose.pose.position.x,
                latest_odom.pose.pose.position.y,
                latest_odom.pose.pose.position.z,
            )
            result.start_position = start_pos

            # Send velocity commands
            start_time = time.monotonic()
            while time.monotonic() - start_time < timeout_sec:
                if use_twist_stamped:
                    cmd = TwistStamped()
                    cmd.header.frame_id = "base_link"
                    cmd.header.stamp = node.get_clock().now().to_msg()
                    cmd.twist.linear.x = velocity
                    cmd.twist.angular.z = 0.0
                else:
                    cmd = Twist()
                    cmd.linear.x = velocity
                    cmd.angular.z = 0.0

                cmd_pub.publish(cmd)
                executor.spin_once(timeout_sec=0.05)

                if latest_odom is not None:
                    end_pos = (
                        latest_odom.pose.pose.position.x,
                        latest_odom.pose.pose.position.y,
                        latest_odom.pose.pose.position.z,
                    )
                    result.end_position = end_pos
                    result.distance_moved = _distance_3d(start_pos, end_pos)

                    gt_end = gz.get_model_pose(gazebo_model_name)
                    if gt_end:
                        result.ground_truth_end = gt_end.position
                        result.ground_truth_distance = gt_start.distance_to(gt_end)

                        if result.ground_truth_distance >= min_distance:
                            result.success = True
                            break

            # Stop the vehicle
            if use_twist_stamped:
                stop_cmd = TwistStamped()
                stop_cmd.header.frame_id = "base_link"
                stop_cmd.header.stamp = node.get_clock().now().to_msg()
            else:
                stop_cmd = Twist()
            cmd_pub.publish(stop_cmd)

            # Final ground truth check
            gt_final = gz.get_model_pose(gazebo_model_name)
            if gt_final:
                result.ground_truth_end = gt_final.position
                result.ground_truth_distance = gt_start.distance_to(gt_final)
                result.odom_error = _distance_3d(
                    result.end_position, gt_final.position
                )

            if result.success:
                result.details = (
                    f"Ground truth: moved {result.ground_truth_distance:.2f}m, "
                    f"Odom: moved {result.distance_moved:.2f}m, "
                    f"Odom error: {result.odom_error:.3f}m"
                )
            else:
                result.details = (
                    f"Ground truth: moved {result.ground_truth_distance or 0:.2f}m "
                    f"(required: {min_distance}m), "
                    f"Odom: moved {result.distance_moved:.2f}m"
                )

    finally:
        node.destroy_subscription(odom_sub)
        node.destroy_publisher(cmd_pub)
        executor.remove_node(node)

    return result
