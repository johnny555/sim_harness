# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Nav2 extensions -- lifecycle management, navigation goals, and path checks.

Primary API uses ``check_*`` names; ``assert_*`` aliases are provided for
backward compatibility.

When called from a :class:`SimTestFixture` test (where the node's executor
is spinning in the background), functions that create temp-node/executor
pairs in managed mode still work fine.  When ``executor`` is provided
explicitly, sleep-based waiting is used instead of ``spin_once``.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.nav2 import check_nav2_active, check_reaches_goal

    class TestNav(SimTestFixture):
        def test_nav_stack(self):
            results = check_nav2_active(self.node)
            assert all(r.ok for r in results)
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

from sim_harness.spin import (
    spin_for_duration,
    wait_for_duration, wait_until_condition,
)

# -- Types ─────────────────────────────────────────────────────────────────


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

    @property
    def ok(self) -> bool:
        return self.success


@dataclass
class ControllerResult:
    success: bool = False
    controller_name: str = ""
    state: str = "unknown"
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.success


@dataclass
class LocalizationResult:
    active: bool = False
    converged: bool = False
    covariance_trace: float = float('inf')
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.active


@dataclass
class NavigationResult:
    success: bool = False
    final_distance_to_goal: float = float('inf')
    time_taken_sec: float = 0.0
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.success


# -- Helpers ───────────────────────────────────────────────────────────────


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


def _acquire_managed(node, executor):
    """Set up managed or non-managed executor for nav2 check functions.

    In managed mode, creates a temporary node and SingleThreadedExecutor.
    In non-managed mode (executor provided or node already in an executor),
    uses the caller's node directly and sleep-based waiting.

    Returns (svc_node, executor, temp_node_or_None, managed: bool).
    """
    if executor is not None:
        return node, executor, None, False

    # Managed mode: create temp node + executor
    temp = _temp_node("nav2_checker")
    exc = SingleThreadedExecutor()
    exc.add_node(temp)
    return temp, exc, temp, True


def _spin_once_or_sleep(executor, managed, timeout_sec=0.1):
    """Spin once if managed, otherwise sleep."""
    if managed:
        executor.spin_once(timeout_sec=timeout_sec)
    else:
        time.sleep(timeout_sec)


def _cleanup_managed(svc_node, client, executor, temp, managed):
    """Clean up resources from _acquire_managed."""
    svc_node.destroy_client(client)
    if managed:
        executor.remove_node(temp)
        temp.destroy_node()


# -- Lifecycle checks ─────────────────────────────────────────────────────


def check_lifecycle_node_active(
    node: Node, lifecycle_node_name: str, timeout_sec: float = 30.0,
    executor=None,
) -> LifecycleResult:
    """Check that a lifecycle node reaches the Active state."""
    return check_lifecycle_node_state(
        node, lifecycle_node_name, LifecycleState.ACTIVE, timeout_sec,
        executor=executor,
    )


def check_lifecycle_node_state(
    node: Node, lifecycle_node_name: str,
    expected_state: LifecycleState, timeout_sec: float = 10.0,
    executor=None,
) -> LifecycleResult:
    """Check that a lifecycle node is in a specific state."""
    svc_node, exc, temp, managed = _acquire_managed(node, executor)

    service_name = f"/{lifecycle_node_name}/get_state"
    client = svc_node.create_client(GetState, service_name)
    result = LifecycleResult()

    try:
        start = time.monotonic()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            _spin_once_or_sleep(exc, managed)

        while time.monotonic() - start < timeout_sec:
            future = client.call_async(GetState.Request())
            while not future.done():
                _spin_once_or_sleep(exc, managed)
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
        _cleanup_managed(svc_node, client, exc, temp, managed)
    return result


def check_lifecycle_nodes_active(
    node: Node, names: List[str], timeout_sec: float = 60.0,
    executor=None,
) -> List[LifecycleResult]:
    """Check that multiple lifecycle nodes are all active."""
    results = []
    remaining = timeout_sec
    for name in names:
        t0 = time.monotonic()
        results.append(check_lifecycle_node_active(
            node, name, remaining, executor=executor))
        remaining = max(1.0, remaining - (time.monotonic() - t0))
    return results


# -- Controller checks ────────────────────────────────────────────────────


def check_controller_active(
    node: Node, controller_manager_name: str, controller_name: str,
    timeout_sec: float = 30.0, executor=None,
) -> ControllerResult:
    """Check that a ros2_control controller is active."""
    from controller_manager_msgs.srv import ListControllers

    svc_node, exc, temp, managed = _acquire_managed(node, executor)

    service_name = f"/{controller_manager_name}/list_controllers"
    client = svc_node.create_client(ListControllers, service_name)
    result = ControllerResult(controller_name=controller_name)

    try:
        start = time.monotonic()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            _spin_once_or_sleep(exc, managed)

        while time.monotonic() - start < timeout_sec:
            future = client.call_async(ListControllers.Request())
            while not future.done():
                _spin_once_or_sleep(exc, managed)
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
        _cleanup_managed(svc_node, client, exc, temp, managed)
    return result


def check_controllers_active(
    node: Node, controller_manager_name: str,
    controller_names: List[str], timeout_sec: float = 30.0,
    executor=None,
) -> List[ControllerResult]:
    """Check that multiple controllers are active."""
    results = []
    remaining = timeout_sec
    for name in controller_names:
        t0 = time.monotonic()
        results.append(check_controller_active(
            node, controller_manager_name, name, remaining, executor=executor))
        remaining = max(1.0, remaining - (time.monotonic() - t0))
    return results


def check_controller_manager_available(
    node: Node, controller_manager_name: str, timeout_sec: float = 30.0,
    executor=None,
) -> bool:
    """Check that the controller manager is available."""
    from controller_manager_msgs.srv import ListControllers

    svc_node, exc, temp, managed = _acquire_managed(node, executor)

    client = svc_node.create_client(
        ListControllers, f"/{controller_manager_name}/list_controllers")
    try:
        return client.wait_for_service(timeout_sec=timeout_sec)
    finally:
        _cleanup_managed(svc_node, client, exc, temp, managed)


# -- Stack-level shortcuts ─────────────────────────────────────────────────


def check_nav2_active(
    node: Node, namespace: str = "", timeout_sec: float = 60.0,
    executor=None,
) -> List[LifecycleResult]:
    """Check that the Nav2 navigation stack is fully active."""
    nodes = ["bt_navigator", "controller_server", "planner_server",
             "recoveries_server", "waypoint_follower"]
    if namespace:
        nodes = [f"{namespace}/{n}" for n in nodes]
    return check_lifecycle_nodes_active(node, nodes, timeout_sec,
                                        executor=executor)


def check_slam_toolbox_active(
    node: Node, node_name: str = "slam_toolbox", timeout_sec: float = 30.0,
    executor=None,
) -> LifecycleResult:
    """Check that SLAM Toolbox is active."""
    return check_lifecycle_node_active(node, node_name, timeout_sec,
                                       executor=executor)


def check_localization_active(
    node: Node, node_name: str = "amcl",
    max_covariance_trace: float = 1.0, timeout_sec: float = 30.0,
    pose_topic: str = "/amcl_pose", executor=None,
) -> LocalizationResult:
    """Check that localization (AMCL) is active and converged."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    lifecycle = check_lifecycle_node_active(node, node_name, timeout_sec,
                                            executor=executor)
    result = LocalizationResult(active=lifecycle.success)
    if not lifecycle.success:
        result.details = lifecycle.details
        return result

    svc_node, exc, temp, managed = _acquire_managed(node, executor)

    pose_msg = None

    def cb(msg):
        nonlocal pose_msg
        pose_msg = msg

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
    sub = svc_node.create_subscription(PoseWithCovarianceStamped, pose_topic, cb, qos)

    try:
        t0 = time.monotonic()
        while pose_msg is None and time.monotonic() - t0 < 5.0:
            _spin_once_or_sleep(exc, managed)
        if pose_msg is not None:
            cov = pose_msg.pose.covariance
            result.covariance_trace = cov[0] + cov[7] + cov[35]
            result.converged = result.covariance_trace <= max_covariance_trace
            result.details = f"Covariance trace: {result.covariance_trace:.4f}"
        else:
            result.details = "No pose received from AMCL"
    finally:
        svc_node.destroy_subscription(sub)
        if managed:
            exc.remove_node(temp)
            temp.destroy_node()
    return result


# -- Navigation checks ────────────────────────────────────────────────────


def check_reaches_goal(
    node: Node, goal_pose: PoseStamped, tolerance: float = 0.5,
    timeout_sec: float = 60.0, odom_topic: str = "/odom",
    executor=None,
) -> NavigationResult:
    """Check that the robot reaches a goal position via odometry."""
    from sim_harness.checks import _acquire_executor
    exc, managed = _acquire_executor(node, executor)
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
            if managed:
                exc.spin_once(timeout_sec=0.1)
            else:
                time.sleep(0.1)
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
        if managed:
            exc.remove_node(node)
    return result


def check_follows_path(
    node: Node, path: List[PoseStamped], corridor_width: float = 1.0,
    timeout_sec: float = 60.0, odom_topic: str = "/odom",
    executor=None,
) -> NavigationResult:
    """Check that the robot follows a path within a corridor."""
    if len(path) < 2:
        return NavigationResult(details="Path must have at least 2 waypoints")

    from sim_harness.checks import _acquire_executor
    exc, managed = _acquire_executor(node, executor)
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
            if managed:
                exc.spin_once(timeout_sec=0.1)
            else:
                time.sleep(0.1)
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
        if managed:
            exc.remove_node(node)
    return result


def check_navigation_action_succeeds(
    node: Node, goal_pose: PoseStamped, timeout_sec: float = 120.0,
    action_name: str = "/navigate_to_pose", executor=None,
) -> NavigationResult:
    """Send NavigateToPose action and wait for completion."""
    managed = executor is None
    if managed:
        temp = _temp_node("nav_action_client")
        action_node = temp
        exc = SingleThreadedExecutor()
        exc.add_node(temp)
    else:
        action_node = node
        temp = None
        exc = executor

    client = ActionClient(action_node, NavigateToPose, action_name)
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
            _spin_once_or_sleep(exc, managed)
            if time.monotonic() - t0 > timeout_sec:
                result.details = "Timeout waiting for goal acceptance"
                return result

        handle = future.result()
        if not handle.accepted:
            result.details = "Goal was rejected"
            return result

        result_future = handle.get_result_async()
        while not result_future.done():
            _spin_once_or_sleep(exc, managed)
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
        if managed:
            exc.remove_node(temp)
            temp.destroy_node()
    return result


def check_costmap_contains_obstacle(
    node: Node, position: Tuple[float, float],
    costmap_topic: str = "/local_costmap/costmap",
    min_cost: int = 100, timeout_sec: float = 5.0,
    executor=None,
) -> bool:
    """Check that the costmap contains an obstacle at a position."""
    from sim_harness.checks import _acquire_executor
    msgs: list = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
    exc, managed = _acquire_executor(node, executor)
    sub = node.create_subscription(OccupancyGrid, costmap_topic, msgs.append, qos)
    try:
        if managed:
            spin_for_duration(exc, timeout_sec)
        else:
            wait_for_duration(timeout_sec)
    finally:
        node.destroy_subscription(sub)
        if managed:
            exc.remove_node(node)

    if not msgs:
        return False
    cm = msgs[-1]
    gx = int((position[0] - cm.info.origin.position.x) / cm.info.resolution)
    gy = int((position[1] - cm.info.origin.position.y) / cm.info.resolution)
    if gx < 0 or gx >= cm.info.width or gy < 0 or gy >= cm.info.height:
        return False
    return cm.data[gy * cm.info.width + gx] >= min_cost


# -- Backward compatibility aliases (deprecated) ──────────────────────────

assert_lifecycle_node_active = check_lifecycle_node_active
assert_lifecycle_node_state = check_lifecycle_node_state
assert_lifecycle_nodes_active = check_lifecycle_nodes_active
assert_controller_active = check_controller_active
assert_controllers_active = check_controllers_active
assert_controller_manager_available = check_controller_manager_available
assert_nav2_active = check_nav2_active
assert_slam_toolbox_active = check_slam_toolbox_active
assert_localization_active = check_localization_active
assert_reaches_goal = check_reaches_goal
assert_follows_path = check_follows_path
assert_navigation_action_succeeds = check_navigation_action_succeeds
assert_costmap_contains_obstacle = check_costmap_contains_obstacle
