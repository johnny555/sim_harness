# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Nav2 extensions — lifecycle management, navigation goals, and path assertions.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.nav2 import assert_nav2_active, assert_reaches_goal

    class TestNav(SimTestFixture):
        def test_nav_stack(self):
            results = assert_nav2_active(self.node)
            assert all(r.success for r in results)
"""

import math
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose

from sim_harness.spin import spin_for_duration

# ── Types ─────────────────────────────────────────────────────────────────


class LifecycleState(IntEnum):
    UNKNOWN = State.PRIMARY_STATE_UNKNOWN
    UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED
    INACTIVE = State.PRIMARY_STATE_INACTIVE
    ACTIVE = State.PRIMARY_STATE_ACTIVE
    FINALIZED = State.PRIMARY_STATE_FINALIZED


def lifecycle_state_to_string(state: LifecycleState) -> str:
    return {
        LifecycleState.UNKNOWN: "Unknown",
        LifecycleState.UNCONFIGURED: "Unconfigured",
        LifecycleState.INACTIVE: "Inactive",
        LifecycleState.ACTIVE: "Active",
        LifecycleState.FINALIZED: "Finalized",
    }.get(state, "Unknown")


@dataclass
class LifecycleResult:
    success: bool = False
    current_state: LifecycleState = LifecycleState.UNKNOWN
    time_to_reach_ms: float = 0.0
    details: str = ""


@dataclass
class ControllerResult:
    success: bool = False
    controller_name: str = ""
    state: str = "unknown"
    details: str = ""


@dataclass
class LocalizationResult:
    active: bool = False
    converged: bool = False
    covariance_trace: float = float('inf')
    details: str = ""


@dataclass
class NavigationResult:
    success: bool = False
    final_distance_to_goal: float = float('inf')
    time_taken_sec: float = 0.0
    details: str = ""


# ── Helpers ───────────────────────────────────────────────────────────────


def _dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _temp_node(prefix: str):
    """Create a throwaway node with unique name."""
    return rclpy.create_node(f"{prefix}_{int(time.time() * 1000) % 10000}")


def _point_to_segment_distance(
    pt: Tuple[float, float], s: Tuple[float, float], e: Tuple[float, float],
) -> float:
    dx, dy = e[0] - s[0], e[1] - s[1]
    if dx == 0 and dy == 0:
        return _dist2(pt, s)
    t = max(0, min(1, ((pt[0] - s[0]) * dx + (pt[1] - s[1]) * dy) / (dx * dx + dy * dy)))
    return _dist2(pt, (s[0] + t * dx, s[1] + t * dy))


# ── Lifecycle assertions ──────────────────────────────────────────────────


def assert_lifecycle_node_active(
    node: Node, lifecycle_node_name: str, timeout_sec: float = 30.0,
) -> LifecycleResult:
    """Assert that a lifecycle node reaches the Active state."""
    return assert_lifecycle_node_state(
        node, lifecycle_node_name, LifecycleState.ACTIVE, timeout_sec,
    )


def assert_lifecycle_node_state(
    node: Node, lifecycle_node_name: str,
    expected_state: LifecycleState, timeout_sec: float = 10.0,
) -> LifecycleResult:
    """Assert that a lifecycle node is in a specific state."""
    temp = _temp_node("lifecycle_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(temp)

    service_name = f"/{lifecycle_node_name}/get_state"
    client = temp.create_client(GetState, service_name)
    result = LifecycleResult()

    try:
        start = time.monotonic()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            executor.spin_once(timeout_sec=0.1)

        while time.monotonic() - start < timeout_sec:
            future = client.call_async(GetState.Request())
            while not future.done():
                executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - start > timeout_sec:
                    break
            if future.done() and future.result() is not None:
                result.current_state = LifecycleState(future.result().current_state.id)
                if result.current_state == expected_state:
                    result.success = True
                    result.time_to_reach_ms = (time.monotonic() - start) * 1000
                    result.details = f"{lifecycle_node_name} reached {lifecycle_state_to_string(expected_state)}"
                    return result
            time.sleep(0.1)

        result.time_to_reach_ms = (time.monotonic() - start) * 1000
        result.details = (
            f"{lifecycle_node_name} in {lifecycle_state_to_string(result.current_state)}, "
            f"expected {lifecycle_state_to_string(expected_state)}"
        )
    finally:
        temp.destroy_client(client)
        executor.remove_node(temp)
        temp.destroy_node()
    return result


def assert_lifecycle_nodes_active(
    node: Node, names: List[str], timeout_sec: float = 60.0,
) -> List[LifecycleResult]:
    """Assert that multiple lifecycle nodes are all active."""
    results = []
    remaining = timeout_sec
    for name in names:
        t0 = time.monotonic()
        results.append(assert_lifecycle_node_active(node, name, remaining))
        remaining = max(1.0, remaining - (time.monotonic() - t0))
    return results


# ── Controller assertions ─────────────────────────────────────────────────


def assert_controller_active(
    node: Node, controller_manager_name: str, controller_name: str,
    timeout_sec: float = 30.0,
) -> ControllerResult:
    """Assert that a ros2_control controller is active."""
    from controller_manager_msgs.srv import ListControllers

    temp = _temp_node("ctrl_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(temp)

    service_name = f"/{controller_manager_name}/list_controllers"
    client = temp.create_client(ListControllers, service_name)
    result = ControllerResult(controller_name=controller_name)

    try:
        start = time.monotonic()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            executor.spin_once(timeout_sec=0.1)

        while time.monotonic() - start < timeout_sec:
            future = client.call_async(ListControllers.Request())
            while not future.done():
                executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - start > timeout_sec:
                    break
            if future.done() and future.result() is not None:
                for ctrl in future.result().controller:
                    if ctrl.name == controller_name:
                        result.state = ctrl.state
                        if ctrl.state == "active":
                            result.success = True
                            result.details = f"Controller {controller_name} is active"
                            return result
            time.sleep(0.1)
        result.details = f"Controller {controller_name} state: {result.state}"
    finally:
        temp.destroy_client(client)
        executor.remove_node(temp)
        temp.destroy_node()
    return result


def assert_controllers_active(
    node: Node, controller_manager_name: str,
    controller_names: List[str], timeout_sec: float = 30.0,
) -> List[ControllerResult]:
    """Assert that multiple controllers are active."""
    results = []
    remaining = timeout_sec
    for name in controller_names:
        t0 = time.monotonic()
        results.append(assert_controller_active(node, controller_manager_name, name, remaining))
        remaining = max(1.0, remaining - (time.monotonic() - t0))
    return results


def assert_controller_manager_available(
    node: Node, controller_manager_name: str, timeout_sec: float = 30.0,
) -> bool:
    """Assert that the controller manager is available."""
    from controller_manager_msgs.srv import ListControllers
    temp = _temp_node("cm_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(temp)
    client = temp.create_client(ListControllers, f"/{controller_manager_name}/list_controllers")
    try:
        return client.wait_for_service(timeout_sec=timeout_sec)
    finally:
        temp.destroy_client(client)
        executor.remove_node(temp)
        temp.destroy_node()


# ── Stack-level shortcuts ─────────────────────────────────────────────────


def assert_nav2_active(
    node: Node, namespace: str = "", timeout_sec: float = 60.0,
) -> List[LifecycleResult]:
    """Assert that the Nav2 navigation stack is fully active."""
    nodes = ["bt_navigator", "controller_server", "planner_server",
             "recoveries_server", "waypoint_follower"]
    if namespace:
        nodes = [f"{namespace}/{n}" for n in nodes]
    return assert_lifecycle_nodes_active(node, nodes, timeout_sec)


def assert_slam_toolbox_active(
    node: Node, node_name: str = "slam_toolbox", timeout_sec: float = 30.0,
) -> LifecycleResult:
    """Assert that SLAM Toolbox is active."""
    return assert_lifecycle_node_active(node, node_name, timeout_sec)


def assert_localization_active(
    node: Node, node_name: str = "amcl",
    max_covariance_trace: float = 1.0, timeout_sec: float = 30.0,
    pose_topic: str = "/amcl_pose",
) -> LocalizationResult:
    """Assert that localization (AMCL) is active and converged."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    lifecycle = assert_lifecycle_node_active(node, node_name, timeout_sec)
    result = LocalizationResult(active=lifecycle.success)
    if not lifecycle.success:
        result.details = lifecycle.details
        return result

    temp = _temp_node("localization_checker")
    executor = SingleThreadedExecutor()
    executor.add_node(temp)
    pose_msg = None

    def cb(msg):
        nonlocal pose_msg
        pose_msg = msg

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
    sub = temp.create_subscription(PoseWithCovarianceStamped, pose_topic, cb, qos)

    try:
        t0 = time.monotonic()
        while pose_msg is None and time.monotonic() - t0 < 5.0:
            executor.spin_once(timeout_sec=0.1)
        if pose_msg is not None:
            cov = pose_msg.pose.covariance
            result.covariance_trace = cov[0] + cov[7] + cov[35]
            result.converged = result.covariance_trace <= max_covariance_trace
            result.details = f"Covariance trace: {result.covariance_trace:.4f}"
        else:
            result.details = "No pose received from AMCL"
    finally:
        temp.destroy_subscription(sub)
        executor.remove_node(temp)
        temp.destroy_node()
    return result


# ── Navigation assertions ─────────────────────────────────────────────────


def assert_reaches_goal(
    node: Node, goal_pose: PoseStamped, tolerance: float = 0.5,
    timeout_sec: float = 60.0, odom_topic: str = "/odom",
) -> NavigationResult:
    """Assert that the robot reaches a goal position via odometry."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    goal_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)
    latest_pos: List[Optional[Tuple[float, float]]] = [None]
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)

    def cb(msg):
        latest_pos[0] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    sub = node.create_subscription(Odometry, odom_topic, cb, qos)
    result = NavigationResult()

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            executor.spin_once(timeout_sec=0.1)
            if latest_pos[0] is not None:
                d = _dist2(latest_pos[0], goal_xy)
                result.final_distance_to_goal = d
                if d <= tolerance:
                    result.success = True
                    result.time_taken_sec = time.monotonic() - t0
                    result.details = f"Reached goal in {result.time_taken_sec:.1f}s (dist {d:.2f}m)"
                    return result
        result.time_taken_sec = time.monotonic() - t0
        result.details = f"Timeout after {result.time_taken_sec:.1f}s, dist {result.final_distance_to_goal:.2f}m"
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)
    return result


def assert_follows_path(
    node: Node, path: List[PoseStamped], corridor_width: float = 1.0,
    timeout_sec: float = 60.0, odom_topic: str = "/odom",
) -> NavigationResult:
    """Assert that the robot follows a path within a corridor."""
    if len(path) < 2:
        return NavigationResult(details="Path must have at least 2 waypoints")

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    pts = [(p.pose.position.x, p.pose.position.y) for p in path]
    max_dev = [0.0]
    latest_pos: List[Optional[Tuple[float, float]]] = [None]
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)

    def cb(msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        latest_pos[0] = pos
        min_d = min(_point_to_segment_distance(pos, pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        max_dev[0] = max(max_dev[0], min_d)

    sub = node.create_subscription(Odometry, odom_topic, cb, qos)
    result = NavigationResult()

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            executor.spin_once(timeout_sec=0.1)
            if max_dev[0] > corridor_width:
                result.time_taken_sec = time.monotonic() - t0
                result.details = f"Exceeded corridor: {max_dev[0]:.2f}m > {corridor_width}m"
                return result
            if latest_pos[0] is not None:
                d = _dist2(latest_pos[0], pts[-1])
                result.final_distance_to_goal = d
                if d <= corridor_width:
                    result.success = True
                    result.time_taken_sec = time.monotonic() - t0
                    result.details = f"Followed path, max deviation: {max_dev[0]:.2f}m"
                    return result
        result.time_taken_sec = time.monotonic() - t0
        result.details = f"Timeout, max deviation: {max_dev[0]:.2f}m"
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)
    return result


def assert_navigation_action_succeeds(
    node: Node, goal_pose: PoseStamped, timeout_sec: float = 120.0,
    action_name: str = "/navigate_to_pose",
) -> NavigationResult:
    """Send NavigateToPose action and wait for completion."""
    temp = _temp_node("nav_action_client")
    executor = SingleThreadedExecutor()
    executor.add_node(temp)
    client = ActionClient(temp, NavigateToPose, action_name)
    result = NavigationResult()

    try:
        if not client.wait_for_server(timeout_sec=10.0):
            result.details = f"Action server {action_name} not available"
            return result

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = client.send_goal_async(goal_msg)
        t0 = time.monotonic()

        while not future.done():
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() - t0 > timeout_sec:
                result.details = "Timeout waiting for goal acceptance"
                return result

        handle = future.result()
        if not handle.accepted:
            result.details = "Goal was rejected"
            return result

        result_future = handle.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() - t0 > timeout_sec:
                result.details = "Timeout waiting for navigation result"
                return result

        action_result = result_future.result()
        result.time_taken_sec = time.monotonic() - t0
        if action_result.status == 4:  # SUCCEEDED
            result.success = True
            result.final_distance_to_goal = 0.0
            result.details = f"Navigation succeeded in {result.time_taken_sec:.1f}s"
        else:
            result.details = f"Navigation failed with status {action_result.status}"
    finally:
        client.destroy()
        executor.remove_node(temp)
        temp.destroy_node()
    return result


def assert_costmap_contains_obstacle(
    node: Node, position: Tuple[float, float],
    costmap_topic: str = "/local_costmap/costmap",
    min_cost: int = 100, timeout_sec: float = 5.0,
) -> bool:
    """Check that the costmap contains an obstacle at a position."""
    msgs: list = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    sub = node.create_subscription(OccupancyGrid, costmap_topic, msgs.append, qos)
    try:
        spin_for_duration(executor, timeout_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not msgs:
        return False
    cm = msgs[-1]
    gx = int((position[0] - cm.info.origin.position.x) / cm.info.resolution)
    gy = int((position[1] - cm.info.origin.position.y) / cm.info.resolution)
    if gx < 0 or gx >= cm.info.width or gy < 0 or gy >= cm.info.height:
        return False
    return cm.data[gy * cm.info.width + gx] >= min_cost
