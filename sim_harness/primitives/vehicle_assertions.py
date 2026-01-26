# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Vehicle motion and state validation assertions.

Provides functions to validate robot movement and state.
Supports both odometry-based and ground-truth-based validation.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, Twist, TwistStamped, Quaternion
from nav_msgs.msg import Odometry

from sim_harness.core.spin_helpers import spin_for_duration


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
    p2: Tuple[float, float, float]
) -> float:
    """Calculate 3D Euclidean distance."""
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def _get_yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from quaternion."""
    # Using formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def assert_vehicle_moved(
    node: Node,
    vehicle_id: str,
    min_distance: float,
    velocity: float = 1.0,
    timeout_sec: float = 10.0,
    odom_topic: Optional[str] = None,
    cmd_vel_topic: Optional[str] = None,
    use_twist_stamped: bool = True
) -> MovementResult:
    """
    Assert that a vehicle moved at least min_distance meters.

    Monitors the vehicle's odometry topic, sends a forward velocity command,
    and verifies that the vehicle moved the required distance.

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

    # Default topic names
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
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    # Create velocity publisher
    if use_twist_stamped:
        cmd_pub = node.create_publisher(TwistStamped, cmd_vel_topic, 10)
    else:
        cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

    result = MovementResult(
        success=False,
        distance_moved=0.0,
        start_position=(0.0, 0.0, 0.0),
        end_position=(0.0, 0.0, 0.0),
        details=""
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
            latest_odom.pose.pose.position.z
        )
        result.start_position = start_pos

        # Send velocity commands
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            # Create and publish command
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

            # Spin to receive odometry
            executor.spin_once(timeout_sec=0.05)

            if latest_odom is not None:
                end_pos = (
                    latest_odom.pose.pose.position.x,
                    latest_odom.pose.pose.position.y,
                    latest_odom.pose.pose.position.z
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
            result.details = f"Vehicle moved {result.distance_moved:.2f}m (required: {min_distance}m)"
        else:
            result.details = f"Vehicle moved {result.distance_moved:.2f}m but required {min_distance}m"

    finally:
        node.destroy_subscription(odom_sub)
        node.destroy_publisher(cmd_pub)
        executor.remove_node(node)

    return result


def assert_vehicle_stationary(
    node: Node,
    vehicle_id: str,
    velocity_threshold: float = 0.01,
    duration_sec: float = 2.0,
    odom_topic: Optional[str] = None
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    max_velocity_seen = 0.0

    def odom_callback(msg: Odometry):
        nonlocal max_velocity_seen
        vel = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        )
        max_velocity_seen = max(max_velocity_seen, vel)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    try:
        start_time = time.monotonic()
        while time.monotonic() - start_time < duration_sec:
            executor.spin_once(timeout_sec=0.01)

            if max_velocity_seen > velocity_threshold:
                return False

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)

    return max_velocity_seen <= velocity_threshold


def assert_vehicle_velocity(
    node: Node,
    vehicle_id: str,
    target_velocity: float,
    tolerance: float = 0.1,
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    latest_velocity = 0.0

    def odom_callback(msg: Odometry):
        nonlocal latest_velocity
        latest_velocity = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        )

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    result = VelocityResult(
        success=False,
        measured_velocity=0.0,
        details=""
    )

    try:
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.01)

            result.measured_velocity = latest_velocity
            if abs(latest_velocity - target_velocity) <= tolerance:
                result.success = True
                break

        result.details = (
            f"Measured velocity: {result.measured_velocity:.2f} m/s "
            f"(target: {target_velocity} +/- {tolerance} m/s)"
        )

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)

    return result


def assert_vehicle_in_region(
    node: Node,
    vehicle_id: str,
    min_bounds: Tuple[float, float, float],
    max_bounds: Tuple[float, float, float],
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    position: Optional[Tuple[float, float, float]] = None

    def odom_callback(msg: Odometry):
        nonlocal position
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    try:
        start_time = time.monotonic()
        while position is None and time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.01)

        if position is None:
            return False

        return (
            min_bounds[0] <= position[0] <= max_bounds[0] and
            min_bounds[1] <= position[1] <= max_bounds[1] and
            min_bounds[2] <= position[2] <= max_bounds[2]
        )

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)


def assert_vehicle_orientation(
    node: Node,
    vehicle_id: str,
    expected_yaw: float,
    tolerance_rad: float = 0.1,
    timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if odom_topic is None:
        odom_topic = f"/{vehicle_id}/odom"

    current_yaw: Optional[float] = None

    def odom_callback(msg: Odometry):
        nonlocal current_yaw
        current_yaw = _get_yaw_from_quaternion(msg.pose.pose.orientation)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    result = False
    try:
        start_time = time.monotonic()
        while current_yaw is None and time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.01)

        if current_yaw is None:
            result = False
        else:
            # Normalize angle difference to [-pi, pi]
            diff = current_yaw - expected_yaw
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi

            result = abs(diff) <= tolerance_rad

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)

    return result


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
    odom_tolerance: float = 0.5
) -> MovementResult:
    """
    Assert vehicle movement with Gazebo ground truth validation.

    Like assert_vehicle_moved, but also verifies the movement against
    Gazebo's ground truth pose. This allows you to:
    1. Confirm the robot actually moved in the simulation
    2. Validate that odometry is accurately reflecting the movement

    Args:
        node: ROS 2 node for subscriptions/publishers
        vehicle_id: Vehicle namespace (e.g., "robot_01")
        gazebo_model_name: Name of the model in Gazebo (may differ from vehicle_id)
        min_distance: Minimum distance to move (meters)
        velocity: Forward velocity to command (m/s)
        timeout_sec: Maximum time to wait
        odom_topic: Custom odometry topic (default: /{vehicle_id}/odom)
        cmd_vel_topic: Custom cmd_vel topic (default: /{vehicle_id}/cmd_vel)
        use_twist_stamped: Use TwistStamped instead of Twist
        world_name: Gazebo world name (for ground truth topic)
        odom_tolerance: Maximum acceptable odom-to-ground-truth error (meters)

    Returns:
        MovementResult with success status, odom data, and ground truth comparison

    Example:
        result = assert_vehicle_moved_with_ground_truth(
            node, "turtlebot3", "turtlebot3_waffle",
            min_distance=1.0, world_name="turtlebot3_world"
        )
        assert result.success, f"Robot didn't move: {result.details}"
        assert result.odom_error < 0.1, f"Odom drift: {result.odom_error}m"
    """
    # Import here to avoid circular imports and handle missing gz-transport
    try:
        from sim_harness.simulator.gazebo_ground_truth import (
            GazeboGroundTruth,
            GZ_TRANSPORT_AVAILABLE
        )
    except ImportError:
        GZ_TRANSPORT_AVAILABLE = False

    if not GZ_TRANSPORT_AVAILABLE:
        # Fall back to regular assertion without ground truth
        result = assert_vehicle_moved(
            node, vehicle_id, min_distance, velocity, timeout_sec,
            odom_topic, cmd_vel_topic, use_twist_stamped
        )
        result.details += " (ground truth unavailable - gz-transport not installed)"
        return result

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Default topic names
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
        durability=DurabilityPolicy.VOLATILE
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    # Create velocity publisher
    if use_twist_stamped:
        cmd_pub = node.create_publisher(TwistStamped, cmd_vel_topic, 10)
    else:
        cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

    result = MovementResult(
        success=False,
        distance_moved=0.0,
        start_position=(0.0, 0.0, 0.0),
        end_position=(0.0, 0.0, 0.0),
        details=""
    )

    try:
        # Connect to Gazebo ground truth
        with GazeboGroundTruth(world_name=world_name) as gz:
            # Get initial ground truth pose
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
                latest_odom.pose.pose.position.z
            )
            result.start_position = start_pos

            # Send velocity commands
            start_time = time.monotonic()
            while time.monotonic() - start_time < timeout_sec:
                # Create and publish command
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
                        latest_odom.pose.pose.position.z
                    )
                    result.end_position = end_pos
                    result.distance_moved = _distance_3d(start_pos, end_pos)

                    # Check ground truth
                    gt_end = gz.get_model_pose(gazebo_model_name)
                    if gt_end:
                        result.ground_truth_end = gt_end.position
                        result.ground_truth_distance = gt_start.distance_to(gt_end)

                        # Check if ground truth shows we've moved enough
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

                # Calculate odom error (difference between odom and ground truth)
                result.odom_error = _distance_3d(result.end_position, gt_final.position)

            # Build details message
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
