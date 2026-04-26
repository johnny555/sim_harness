# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Service / action / node / parameter / TF availability checks."""

import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from sim_harness.spin import ExecutorContext

from ._base import ServiceResult


def check_service_available(
    node: Node, service_name: str, service_type, timeout_sec: float = 10.0,
    executor=None,
) -> ServiceResult:
    """Check that a ROS service is available."""
    client = node.create_client(service_type, service_name)
    try:
        available = client.wait_for_service(timeout_sec=timeout_sec)
        return ServiceResult(
            available=available, call_succeeded=available,
            details="" if available else f"Service {service_name} not available",
        )
    finally:
        node.destroy_client(client)


def check_action_server_available(
    node: Node, action_name: str, action_type, timeout_sec: float = 10.0,
    executor=None,
) -> ServiceResult:
    """Check that a ROS action server is available."""
    from rclpy.action import ActionClient
    client = ActionClient(node, action_type, action_name)
    try:
        available = client.wait_for_server(timeout_sec=timeout_sec)
        return ServiceResult(
            available=available, call_succeeded=available,
            details="" if available else f"Action {action_name} not available",
        )
    finally:
        client.destroy()


def check_node_running(
    node: Node, target_node_name: str, timeout_sec: float = 10.0,
    executor=None,
) -> bool:
    """Check that a node is visible on the ROS graph."""
    end = time.monotonic() + timeout_sec
    with ExecutorContext(node, executor) as ec:
        while time.monotonic() < end:
            names = [n for n, ns in node.get_node_names_and_namespaces()]
            if target_node_name in names:
                return True
            ec.spin_once(timeout_sec=0.5)
        return False


def check_nodes_running(
    node: Node, node_names: List[str], timeout_sec: float = 30.0,
    executor=None,
) -> List[Tuple[str, bool]]:
    """Check multiple nodes. Returns list of (name, found) tuples."""
    return [(n, check_node_running(node, n, timeout_sec, executor=executor))
            for n in node_names]


def check_parameter_exists(
    node: Node, target_node_name: str, parameter_name: str,
    expected_value=None, timeout_sec: float = 10.0,
    executor=None,
) -> bool:
    """Check that a parameter exists on a remote node."""
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(
        GetParameters, f'/{target_node_name}/get_parameters',
    )
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        req = GetParameters.Request(names=[parameter_name])
        with ExecutorContext(node, executor) as ec:
            future = client.call_async(req)
            ec.wait_until(future.done, timeout_sec)
            if not future.done():
                return False
            resp = future.result()
            if not resp.values:
                return False
            if expected_value is not None:
                val = resp.values[0]
                if hasattr(val, 'string_value') and val.string_value == str(expected_value):
                    return True
                if hasattr(val, 'double_value') and val.double_value == expected_value:
                    return True
                if hasattr(val, 'integer_value') and val.integer_value == expected_value:
                    return True
                if hasattr(val, 'bool_value') and val.bool_value == expected_value:
                    return True
                return False
            return True
    finally:
        node.destroy_client(client)


def check_transform_available(
    node: Node, target_frame: str, source_frame: str,
    timeout_sec: float = 5.0, max_age_ms: float = 1000.0,
    executor=None,
) -> bool:
    """Check that a TF transform is available and recent."""
    try:
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError:
        return False
    buf = Buffer()
    listener = TransformListener(buf, node)
    with ExecutorContext(node, executor) as ec:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            ec.spin_once(timeout_sec=0.1)
            try:
                t = buf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                age_ms = (node.get_clock().now().nanoseconds -
                          rclpy.time.Time.from_msg(t.header.stamp).nanoseconds) / 1e6
                if age_ms <= max_age_ms:
                    return True
            except TransformException:
                pass
            except Exception as e:
                node.get_logger().warning(
                    f"Unexpected error in TF lookup "
                    f"{source_frame}->{target_frame}: {e}")
                pass
        return False


def check_action_server_responsive(
    node: Node, action_name: str, action_type,
    max_response_time_ms: float = 1000.0, executor=None,
) -> bool:
    """Check that an action server accepts a goal within time limit."""
    from rclpy.action import ActionClient
    client = ActionClient(node, action_type, action_name)
    try:
        if not client.wait_for_server(timeout_sec=max_response_time_ms / 1000):
            return False
        goal = action_type.Goal()
        start = time.monotonic()
        future = client.send_goal_async(goal)
        with ExecutorContext(node, executor) as ec:
            ec.wait_until(future.done, max_response_time_ms / 1000)
        elapsed = (time.monotonic() - start) * 1000
        return future.done() and elapsed <= max_response_time_ms
    finally:
        client.destroy()
