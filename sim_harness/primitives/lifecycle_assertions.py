# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Lifecycle and controller validation assertions.

Provides functions to validate ROS 2 lifecycle nodes and ros2_control controllers.
"""

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State


class LifecycleState(IntEnum):
    """Lifecycle states (matches ROS 2 lifecycle_msgs)."""
    UNKNOWN = State.PRIMARY_STATE_UNKNOWN
    UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED
    INACTIVE = State.PRIMARY_STATE_INACTIVE
    ACTIVE = State.PRIMARY_STATE_ACTIVE
    FINALIZED = State.PRIMARY_STATE_FINALIZED


def lifecycle_state_to_string(state: LifecycleState) -> str:
    """Convert LifecycleState to string."""
    return {
        LifecycleState.UNKNOWN: "Unknown",
        LifecycleState.UNCONFIGURED: "Unconfigured",
        LifecycleState.INACTIVE: "Inactive",
        LifecycleState.ACTIVE: "Active",
        LifecycleState.FINALIZED: "Finalized",
    }.get(state, "Unknown")


@dataclass
class LifecycleResult:
    """Result of a lifecycle node assertion."""

    success: bool
    """Whether the assertion succeeded."""

    current_state: LifecycleState
    """Current state of the node."""

    time_to_reach_ms: float
    """Time taken to reach the state (milliseconds)."""

    details: str
    """Human-readable details."""


@dataclass
class ControllerResult:
    """Result of a controller assertion."""

    success: bool
    """Whether the controller is in expected state."""

    controller_name: str
    """Controller name."""

    state: str
    """Current state (e.g., "active", "inactive")."""

    details: str
    """Human-readable details."""


@dataclass
class LocalizationResult:
    """Result of a localization assertion."""

    active: bool
    """Whether localization is active."""

    converged: bool
    """Whether localization has converged."""

    covariance_trace: float
    """Covariance trace (lower = more confident)."""

    details: str
    """Human-readable details."""


def assert_lifecycle_node_active(
    node: Node,
    lifecycle_node_name: str,
    timeout_sec: float = 30.0
) -> LifecycleResult:
    """
    Assert that a lifecycle node reaches the Active state.

    Uses the /get_state service to query node state.

    Args:
        node: ROS 2 node for service calls
        lifecycle_node_name: Name of the lifecycle node
        timeout_sec: Maximum time to wait

    Returns:
        LifecycleResult
    """
    return assert_lifecycle_node_state(
        node,
        lifecycle_node_name,
        LifecycleState.ACTIVE,
        timeout_sec
    )


def assert_lifecycle_node_state(
    node: Node,
    lifecycle_node_name: str,
    expected_state: LifecycleState,
    timeout_sec: float = 10.0
) -> LifecycleResult:
    """
    Assert that a lifecycle node is in a specific state.

    Args:
        node: ROS 2 node
        lifecycle_node_name: Name of the lifecycle node
        expected_state: Expected state
        timeout_sec: Maximum time to wait

    Returns:
        LifecycleResult
    """
    # Create a temporary node for service calls to avoid executor conflicts
    temp_node = rclpy.create_node(f"lifecycle_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    service_name = f"/{lifecycle_node_name}/get_state"
    client = temp_node.create_client(GetState, service_name)

    result = LifecycleResult(
        success=False,
        current_state=LifecycleState.UNKNOWN,
        time_to_reach_ms=0.0,
        details=""
    )

    try:
        start_time = time.monotonic()

        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start_time > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            executor.spin_once(timeout_sec=0.1)

        # Poll state until expected or timeout
        while time.monotonic() - start_time < timeout_sec:
            request = GetState.Request()
            future = client.call_async(request)

            # Wait for response
            while not future.done():
                executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - start_time > timeout_sec:
                    break

            if future.done():
                response = future.result()
                if response is not None:
                    result.current_state = LifecycleState(response.current_state.id)

                    if result.current_state == expected_state:
                        result.success = True
                        result.time_to_reach_ms = (time.monotonic() - start_time) * 1000
                        result.details = f"Node {lifecycle_node_name} reached {lifecycle_state_to_string(expected_state)}"
                        return result

            time.sleep(0.1)

        result.time_to_reach_ms = (time.monotonic() - start_time) * 1000
        result.details = (
            f"Node {lifecycle_node_name} in state {lifecycle_state_to_string(result.current_state)}, "
            f"expected {lifecycle_state_to_string(expected_state)}"
        )

    finally:
        temp_node.destroy_client(client)
        executor.remove_node(temp_node)
        temp_node.destroy_node()

    return result


def assert_lifecycle_nodes_active(
    node: Node,
    lifecycle_node_names: List[str],
    timeout_sec: float = 60.0
) -> List[LifecycleResult]:
    """
    Assert that multiple lifecycle nodes are all active.

    Useful for checking entire stacks (e.g., Nav2).

    Args:
        node: ROS 2 node
        lifecycle_node_names: List of node names to check
        timeout_sec: Maximum time to wait for all nodes

    Returns:
        List of results, one per node
    """
    results = []
    remaining_timeout = timeout_sec

    for node_name in lifecycle_node_names:
        start = time.monotonic()
        result = assert_lifecycle_node_active(node, node_name, remaining_timeout)
        results.append(result)
        remaining_timeout -= (time.monotonic() - start)
        remaining_timeout = max(remaining_timeout, 1.0)

    return results


def assert_controller_active(
    node: Node,
    controller_manager_name: str,
    controller_name: str,
    timeout_sec: float = 30.0
) -> ControllerResult:
    """
    Assert that a ros2_control controller is active.

    Uses controller_manager services to check state.

    Args:
        node: ROS 2 node
        controller_manager_name: Controller manager namespace
        controller_name: Name of the controller
        timeout_sec: Maximum time to wait

    Returns:
        ControllerResult
    """
    from controller_manager_msgs.srv import ListControllers

    temp_node = rclpy.create_node(f"controller_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    service_name = f"/{controller_manager_name}/list_controllers"
    client = temp_node.create_client(ListControllers, service_name)

    result = ControllerResult(
        success=False,
        controller_name=controller_name,
        state="unknown",
        details=""
    )

    try:
        start_time = time.monotonic()

        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() - start_time > timeout_sec:
                result.details = f"Service {service_name} not available"
                return result
            executor.spin_once(timeout_sec=0.1)

        while time.monotonic() - start_time < timeout_sec:
            request = ListControllers.Request()
            future = client.call_async(request)

            while not future.done():
                executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - start_time > timeout_sec:
                    break

            if future.done():
                response = future.result()
                if response is not None:
                    for ctrl in response.controller:
                        if ctrl.name == controller_name:
                            result.state = ctrl.state
                            if ctrl.state == "active":
                                result.success = True
                                result.details = f"Controller {controller_name} is active"
                                return result

            time.sleep(0.1)

        result.details = f"Controller {controller_name} state: {result.state}"

    finally:
        temp_node.destroy_client(client)
        executor.remove_node(temp_node)
        temp_node.destroy_node()

    return result


def assert_controllers_active(
    node: Node,
    controller_manager_name: str,
    controller_names: List[str],
    timeout_sec: float = 30.0
) -> List[ControllerResult]:
    """
    Assert that multiple controllers are active.

    Args:
        node: ROS 2 node
        controller_manager_name: Controller manager namespace
        controller_names: List of controller names
        timeout_sec: Maximum time to wait

    Returns:
        List of results
    """
    results = []
    remaining_timeout = timeout_sec

    for ctrl_name in controller_names:
        start = time.monotonic()
        result = assert_controller_active(node, controller_manager_name, ctrl_name, remaining_timeout)
        results.append(result)
        remaining_timeout -= (time.monotonic() - start)
        remaining_timeout = max(remaining_timeout, 1.0)

    return results


def assert_controller_manager_available(
    node: Node,
    controller_manager_name: str,
    timeout_sec: float = 30.0
) -> bool:
    """
    Assert that the controller manager is available.

    Args:
        node: ROS 2 node
        controller_manager_name: Controller manager namespace
        timeout_sec: Maximum time to wait

    Returns:
        True if controller manager responds
    """
    from controller_manager_msgs.srv import ListControllers

    temp_node = rclpy.create_node(f"cm_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    service_name = f"/{controller_manager_name}/list_controllers"
    client = temp_node.create_client(ListControllers, service_name)

    try:
        return client.wait_for_service(timeout_sec=timeout_sec)
    finally:
        temp_node.destroy_client(client)
        executor.remove_node(temp_node)
        temp_node.destroy_node()


def assert_nav2_active(
    node: Node,
    namespace: str = "",
    timeout_sec: float = 60.0
) -> List[LifecycleResult]:
    """
    Assert that the Nav2 navigation stack is fully active.

    Checks standard Nav2 nodes:
    - bt_navigator
    - controller_server
    - planner_server
    - recoveries_server
    - waypoint_follower

    Args:
        node: ROS 2 node
        namespace: Namespace for Nav2 nodes (empty = default)
        timeout_sec: Maximum time to wait

    Returns:
        List of results for each Nav2 node
    """
    nav2_nodes = [
        "bt_navigator",
        "controller_server",
        "planner_server",
        "recoveries_server",
        "waypoint_follower"
    ]

    if namespace:
        nav2_nodes = [f"{namespace}/{n}" for n in nav2_nodes]

    return assert_lifecycle_nodes_active(node, nav2_nodes, timeout_sec)


def assert_slam_toolbox_active(
    node: Node,
    node_name: str = "slam_toolbox",
    timeout_sec: float = 30.0
) -> LifecycleResult:
    """
    Assert that SLAM Toolbox is active.

    Args:
        node: ROS 2 node
        node_name: SLAM Toolbox node name
        timeout_sec: Maximum time to wait

    Returns:
        LifecycleResult
    """
    return assert_lifecycle_node_active(node, node_name, timeout_sec)


def assert_localization_active(
    node: Node,
    node_name: str = "amcl",
    max_covariance_trace: float = 1.0,
    timeout_sec: float = 30.0
) -> LocalizationResult:
    """
    Assert that localization (AMCL) is active and converged.

    Args:
        node: ROS 2 node
        node_name: AMCL node name
        max_covariance_trace: Maximum covariance trace for "converged"
        timeout_sec: Maximum time to wait

    Returns:
        LocalizationResult
    """
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    # First check if node is active
    lifecycle_result = assert_lifecycle_node_active(node, node_name, timeout_sec)

    result = LocalizationResult(
        active=lifecycle_result.success,
        converged=False,
        covariance_trace=float('inf'),
        details=""
    )

    if not lifecycle_result.success:
        result.details = lifecycle_result.details
        return result

    # Check covariance
    temp_node = rclpy.create_node(f"localization_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    pose_msg: Optional[PoseWithCovarianceStamped] = None

    def callback(msg: PoseWithCovarianceStamped):
        nonlocal pose_msg
        pose_msg = msg

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = temp_node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        callback,
        qos
    )

    try:
        start_time = time.monotonic()
        while pose_msg is None and time.monotonic() - start_time < 5.0:
            executor.spin_once(timeout_sec=0.1)

        if pose_msg is not None:
            # Calculate covariance trace (diagonal elements: xx, yy, zz, roll, pitch, yaw)
            cov = pose_msg.pose.covariance
            result.covariance_trace = cov[0] + cov[7] + cov[35]  # xx, yy, yaw
            result.converged = result.covariance_trace <= max_covariance_trace
            result.details = f"Covariance trace: {result.covariance_trace:.4f}"
        else:
            result.details = "No pose received from AMCL"

    finally:
        temp_node.destroy_subscription(sub)
        executor.remove_node(temp_node)
        temp_node.destroy_node()

    return result
