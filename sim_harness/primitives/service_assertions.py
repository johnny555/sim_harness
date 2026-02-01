# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
ROS 2 service and action validation assertions.

Provides functions to check service/action availability and node status.
"""

import time
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple, Type, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rcl_interfaces.srv import GetParameters

from sim_harness.core.spin_helpers import temporary_node


@dataclass
class ServiceResult:
    """Result of a service assertion."""

    available: bool
    """Whether the service is available."""

    call_succeeded: bool
    """Whether the service call succeeded."""

    response_time_ms: float
    """Response time in milliseconds."""

    details: str
    """Human-readable details."""


T = TypeVar('T')


def assert_service_available(
    node: Node,
    service_name: str,
    service_type: Type[T],
    timeout_sec: float = 10.0
) -> ServiceResult:
    """
    Assert that a service is available.

    Creates a temporary node internally for the service client.

    Args:
        node: ROS 2 node (unused, kept for API consistency)
        service_name: Service name
        service_type: Service type class
        timeout_sec: Maximum time to wait

    Returns:
        ServiceResult with:
        - available: True if service responded within timeout
        - call_succeeded: Same as available (availability check succeeded)
        - response_time_ms: Time to discover the service
    """
    result = ServiceResult(
        available=False,
        call_succeeded=False,
        response_time_ms=0.0,
        details=""
    )

    with temporary_node("service_checker") as (temp_node, executor):
        client = temp_node.create_client(service_type, service_name)

        try:
            start_time = time.monotonic()
            result.available = client.wait_for_service(timeout_sec=timeout_sec)
            result.response_time_ms = (time.monotonic() - start_time) * 1000
            result.call_succeeded = result.available

            if result.available:
                result.details = f"Service {service_name} available in {result.response_time_ms:.0f}ms"
            else:
                result.details = f"Service {service_name} not available after {timeout_sec}s"

        finally:
            temp_node.destroy_client(client)

    return result


def assert_action_server_available(
    node: Node,
    action_name: str,
    action_type: Type[T],
    timeout_sec: float = 10.0
) -> ServiceResult:
    """
    Assert that an action server is available.

    Creates a temporary node internally for the action client.

    Args:
        node: ROS 2 node (unused, kept for API consistency)
        action_name: Action name
        action_type: Action type class
        timeout_sec: Maximum time to wait

    Returns:
        ServiceResult with:
        - available: True if action server responded within timeout
        - call_succeeded: Same as available (availability check succeeded)
        - response_time_ms: Time to discover the action server
    """
    result = ServiceResult(
        available=False,
        call_succeeded=False,
        response_time_ms=0.0,
        details=""
    )

    with temporary_node("action_checker") as (temp_node, executor):
        action_client = ActionClient(temp_node, action_type, action_name)

        try:
            start_time = time.monotonic()
            result.available = action_client.wait_for_server(timeout_sec=timeout_sec)
            result.response_time_ms = (time.monotonic() - start_time) * 1000
            result.call_succeeded = result.available

            if result.available:
                result.details = f"Action server {action_name} available in {result.response_time_ms:.0f}ms"
            else:
                result.details = f"Action server {action_name} not available after {timeout_sec}s"

        finally:
            action_client.destroy()

    return result


def assert_node_running(
    node: Node,
    target_node_name: str,
    timeout_sec: float = 10.0
) -> bool:
    """
    Assert that a node is running.

    Checks if node responds to /get_parameters service.

    Args:
        node: ROS 2 node for checking
        target_node_name: Name of node to check
        timeout_sec: Maximum time to wait

    Returns:
        True if node is running
    """
    service_name = f"/{target_node_name}/get_parameters"

    with temporary_node("node_checker") as (temp_node, executor):
        client = temp_node.create_client(GetParameters, service_name)

        try:
            return client.wait_for_service(timeout_sec=timeout_sec)
        finally:
            temp_node.destroy_client(client)


def assert_nodes_running(
    node: Node,
    node_names: List[str],
    timeout_sec: float = 30.0
) -> List[Tuple[str, bool]]:
    """
    Assert that multiple nodes are running.

    Args:
        node: ROS 2 node for checking
        node_names: List of node names to check
        timeout_sec: Maximum time to wait for all

    Returns:
        List of (node_name, running) tuples
    """
    results = []
    remaining_timeout = timeout_sec

    for name in node_names:
        start = time.monotonic()
        running = assert_node_running(node, name, remaining_timeout)
        results.append((name, running))
        remaining_timeout -= (time.monotonic() - start)
        remaining_timeout = max(remaining_timeout, 1.0)

    return results


def assert_parameter_exists(
    node: Node,
    target_node_name: str,
    parameter_name: str,
    expected_value: Optional[Any] = None,
    timeout_sec: float = 10.0
) -> bool:
    """
    Assert that a parameter exists on a node.

    Optionally checks if the parameter has an expected value. Creates a
    temporary node internally for the service client.

    Args:
        node: ROS 2 node (unused, kept for API consistency)
        target_node_name: Node that has the parameter
        parameter_name: Parameter name
        expected_value: Optional expected value. Only compared for BOOL,
            INTEGER, DOUBLE, and STRING types. For array/complex types,
            returns True if parameter exists (value comparison skipped).
        timeout_sec: Maximum time to wait

    Returns:
        True if parameter exists (and matches value if specified for simple types)
    """
    service_name = f"/{target_node_name}/get_parameters"

    with temporary_node("param_checker") as (temp_node, executor):
        client = temp_node.create_client(GetParameters, service_name)

        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False

            request = GetParameters.Request()
            request.names = [parameter_name]

            future = client.call_async(request)

            start_time = time.monotonic()
            while not future.done():
                executor.spin_once(timeout_sec=0.1)
                if time.monotonic() - start_time > timeout_sec:
                    return False

            response = future.result()
            if response is None or len(response.values) == 0:
                return False

            param_value = response.values[0]

            # Check if parameter was found (not NOT_SET)
            if param_value.type == 0:  # PARAMETER_NOT_SET
                return False

            # If expected_value specified, compare
            if expected_value is not None:
                # Extract actual value based on type
                if param_value.type == 1:  # BOOL
                    actual = param_value.bool_value
                elif param_value.type == 2:  # INTEGER
                    actual = param_value.integer_value
                elif param_value.type == 3:  # DOUBLE
                    actual = param_value.double_value
                elif param_value.type == 4:  # STRING
                    actual = param_value.string_value
                else:
                    return True  # Parameter exists but can't compare complex types

                return actual == expected_value

            return True

        finally:
            temp_node.destroy_client(client)
