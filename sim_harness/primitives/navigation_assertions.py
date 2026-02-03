# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Navigation stack validation assertions.

Provides functions to validate robot navigation behavior.

``assert_costmap_contains_obstacle`` uses :class:`TopicObserver`.
The remaining functions use manual loops because they require early exit
with long timeouts (60s+) or use the action server API directly.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose

from sim_harness.core.topic_observer import latest_message


@dataclass
class NavigationResult:
    """Result of a navigation assertion."""

    success: bool
    """Whether the navigation succeeded."""

    final_distance_to_goal: float
    """Final distance to goal (meters)."""

    time_taken_sec: float
    """Time taken to reach goal (seconds)."""

    details: str
    """Human-readable details."""


def _distance_2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate 2D Euclidean distance."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def assert_reaches_goal(
    node: Node,
    goal_pose: PoseStamped,
    tolerance: float = 0.5,
    timeout_sec: float = 60.0,
    odom_topic: str = "/odom",
) -> NavigationResult:
    """
    Assert that the robot reaches a goal position.

    Monitors odometry until vehicle reaches goal within tolerance.

    Args:
        node: ROS 2 node
        goal_pose: Target pose
        tolerance: Distance tolerance (meters)
        timeout_sec: Maximum time to wait
        odom_topic: Odometry topic to monitor

    Returns:
        NavigationResult with success status, final distance, and timing
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    goal_x = goal_pose.pose.position.x
    goal_y = goal_pose.pose.position.y

    latest_position: Optional[Tuple[float, float]] = None

    def odom_callback(msg: Odometry):
        nonlocal latest_position
        latest_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    result = NavigationResult(
        success=False,
        final_distance_to_goal=float('inf'),
        time_taken_sec=0.0,
        details="",
    )

    try:
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            if latest_position is not None:
                distance = _distance_2d(latest_position, (goal_x, goal_y))
                result.final_distance_to_goal = distance

                if distance <= tolerance:
                    result.success = True
                    result.time_taken_sec = time.monotonic() - start_time
                    result.details = (
                        f"Reached goal in {result.time_taken_sec:.1f}s "
                        f"(distance: {distance:.2f}m)"
                    )
                    return result

        result.time_taken_sec = time.monotonic() - start_time
        result.details = (
            f"Timeout after {result.time_taken_sec:.1f}s, "
            f"distance to goal: {result.final_distance_to_goal:.2f}m"
        )

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)

    return result


def assert_follows_path(
    node: Node,
    path: List[PoseStamped],
    corridor_width: float = 1.0,
    timeout_sec: float = 60.0,
    odom_topic: str = "/odom",
) -> NavigationResult:
    """
    Assert that the robot follows a path within a corridor.

    Verifies the vehicle stays within corridor_width of the expected path.

    Args:
        node: ROS 2 node
        path: List of waypoints (must have at least 2)
        corridor_width: Maximum deviation from path (meters)
        timeout_sec: Maximum time to wait
        odom_topic: Odometry topic to monitor

    Returns:
        NavigationResult with success status and maximum deviation observed
    """
    if len(path) < 2:
        return NavigationResult(
            success=False,
            final_distance_to_goal=float('inf'),
            time_taken_sec=0.0,
            details="Path must have at least 2 waypoints",
        )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    path_points = [(p.pose.position.x, p.pose.position.y) for p in path]

    max_deviation = 0.0
    latest_position: Optional[Tuple[float, float]] = None

    def odom_callback(msg: Odometry):
        nonlocal latest_position, max_deviation
        latest_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        min_dist = float('inf')
        for i in range(len(path_points) - 1):
            dist = _point_to_segment_distance(
                latest_position, path_points[i], path_points[i + 1]
            )
            min_dist = min(min_dist, dist)

        max_deviation = max(max_deviation, min_dist)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    odom_sub = node.create_subscription(Odometry, odom_topic, odom_callback, qos)

    result = NavigationResult(
        success=False,
        final_distance_to_goal=float('inf'),
        time_taken_sec=0.0,
        details="",
    )

    try:
        start_time = time.monotonic()
        goal = path_points[-1]

        while time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            if max_deviation > corridor_width:
                result.time_taken_sec = time.monotonic() - start_time
                result.details = (
                    f"Exceeded corridor width: "
                    f"{max_deviation:.2f}m > {corridor_width}m"
                )
                return result

            if latest_position is not None:
                distance = _distance_2d(latest_position, goal)
                result.final_distance_to_goal = distance

                if distance <= corridor_width:
                    result.success = True
                    result.time_taken_sec = time.monotonic() - start_time
                    result.details = (
                        f"Followed path, max deviation: {max_deviation:.2f}m"
                    )
                    return result

        result.time_taken_sec = time.monotonic() - start_time
        result.details = f"Timeout, max deviation: {max_deviation:.2f}m"

    finally:
        node.destroy_subscription(odom_sub)
        executor.remove_node(node)

    return result


def _point_to_segment_distance(
    point: Tuple[float, float],
    seg_start: Tuple[float, float],
    seg_end: Tuple[float, float],
) -> float:
    """Calculate distance from point to line segment."""
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return _distance_2d(point, seg_start)

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return _distance_2d(point, (proj_x, proj_y))


def assert_navigation_action_succeeds(
    node: Node,
    goal_pose: PoseStamped,
    timeout_sec: float = 120.0,
    action_name: str = "/navigate_to_pose",
) -> NavigationResult:
    """
    Assert that a navigation action succeeds.

    Sends a NavigateToPose action and waits for completion.

    Args:
        node: ROS 2 node (unused, kept for API consistency)
        goal_pose: Target pose
        timeout_sec: Maximum time to wait
        action_name: Action server name

    Returns:
        NavigationResult with success status
    """
    temp_node = rclpy.create_node(
        f"nav_action_client_{int(time.time() * 1000) % 10000}"
    )
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    action_client = ActionClient(temp_node, NavigateToPose, action_name)

    result = NavigationResult(
        success=False,
        final_distance_to_goal=float('inf'),
        time_taken_sec=0.0,
        details="",
    )

    try:
        if not action_client.wait_for_server(timeout_sec=10.0):
            result.details = f"Action server {action_name} not available"
            return result

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_goal_future = action_client.send_goal_async(goal_msg)

        start_time = time.monotonic()

        while not send_goal_future.done():
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() - start_time > timeout_sec:
                result.details = "Timeout waiting for goal acceptance"
                return result

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            result.details = "Goal was rejected"
            return result

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() - start_time > timeout_sec:
                result.details = "Timeout waiting for navigation result"
                return result

        action_result = result_future.result()
        result.time_taken_sec = time.monotonic() - start_time

        if action_result.status == 4:  # SUCCEEDED
            result.success = True
            result.final_distance_to_goal = 0.0
            result.details = (
                f"Navigation succeeded in {result.time_taken_sec:.1f}s"
            )
        else:
            result.details = (
                f"Navigation failed with status {action_result.status}"
            )

    finally:
        action_client.destroy()
        executor.remove_node(temp_node)
        temp_node.destroy_node()

    return result


def assert_costmap_contains_obstacle(
    node: Node,
    position: Tuple[float, float],
    costmap_topic: str = "/local_costmap/costmap",
    min_cost: int = 100,
    timeout_sec: float = 5.0,
) -> bool:
    """
    Assert that the costmap contains an obstacle at a position.

    Checks for high-cost cells near the specified position.

    Args:
        node: ROS 2 node
        position: Position to check (x, y)
        costmap_topic: Costmap topic
        min_cost: Minimum cost value to consider as obstacle
        timeout_sec: Time to wait for costmap

    Returns:
        True if obstacle found at position
    """
    costmap_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    obs = latest_message(costmap_topic, OccupancyGrid, qos=costmap_qos)
    result = obs.run_standalone(node, timeout_sec)

    costmap_msg = result.value
    if costmap_msg is None:
        return False

    # Convert position to grid coordinates
    origin_x = costmap_msg.info.origin.position.x
    origin_y = costmap_msg.info.origin.position.y
    resolution = costmap_msg.info.resolution
    width = costmap_msg.info.width

    grid_x = int((position[0] - origin_x) / resolution)
    grid_y = int((position[1] - origin_y) / resolution)

    # Check bounds
    if (
        grid_x < 0 or grid_x >= width
        or grid_y < 0 or grid_y >= costmap_msg.info.height
    ):
        return False

    # Get cost at position
    index = grid_y * width + grid_x
    cost = costmap_msg.data[index]

    return cost >= min_cost
