# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Spin helpers for ROS 2 executor management.

Provides utility functions for spinning executors with timeouts and conditions.
"""

import time
from typing import Callable, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def spin_for_duration(
    executor: SingleThreadedExecutor,
    duration_sec: float,
    spin_interval_sec: float = 0.01
) -> None:
    """
    Spin the executor for a specified duration.

    Processes callbacks for the given duration, allowing time for
    messages to be received and simulation to progress.

    Args:
        executor: ROS 2 executor to spin
        duration_sec: How long to spin (seconds)
        spin_interval_sec: Interval between spin_once calls (seconds)
    """
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        executor.spin_once(timeout_sec=spin_interval_sec)


def spin_until_condition(
    executor: SingleThreadedExecutor,
    condition: Callable[[], bool],
    timeout_sec: float,
    spin_interval_sec: float = 0.01
) -> bool:
    """
    Spin until a condition is met or timeout occurs.

    Args:
        executor: ROS 2 executor to spin
        condition: Function returning True when condition is met
        timeout_sec: Maximum time to wait (seconds)
        spin_interval_sec: Interval between spin_once calls (seconds)

    Returns:
        True if condition was met, False if timeout occurred
    """
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        executor.spin_once(timeout_sec=spin_interval_sec)
        if condition():
            return True
    return False


def spin_until_messages_received(
    executor: SingleThreadedExecutor,
    get_count: Callable[[], int],
    min_count: int,
    timeout_sec: float,
    spin_interval_sec: float = 0.01
) -> bool:
    """
    Spin until a minimum number of messages are received.

    Args:
        executor: ROS 2 executor to spin
        get_count: Function returning current message count
        min_count: Minimum number of messages required
        timeout_sec: Maximum time to wait (seconds)
        spin_interval_sec: Interval between spin_once calls (seconds)

    Returns:
        True if minimum count reached, False if timeout occurred
    """
    return spin_until_condition(
        executor,
        lambda: get_count() >= min_count,
        timeout_sec,
        spin_interval_sec
    )


def spin_node_for_duration(
    node: Node,
    duration_sec: float,
    spin_interval_sec: float = 0.01
) -> None:
    """
    Create a temporary executor and spin a node for a duration.

    Convenience function for standalone operations.

    Args:
        node: ROS 2 node to spin
        duration_sec: How long to spin (seconds)
        spin_interval_sec: Interval between spin_once calls (seconds)
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        spin_for_duration(executor, duration_sec, spin_interval_sec)
    finally:
        executor.remove_node(node)


def spin_node_until_condition(
    node: Node,
    condition: Callable[[], bool],
    timeout_sec: float,
    spin_interval_sec: float = 0.01
) -> bool:
    """
    Create a temporary executor and spin a node until condition is met.

    Convenience function for standalone operations.

    Args:
        node: ROS 2 node to spin
        condition: Function returning True when condition is met
        timeout_sec: Maximum time to wait (seconds)
        spin_interval_sec: Interval between spin_once calls (seconds)

    Returns:
        True if condition was met, False if timeout occurred
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        return spin_until_condition(executor, condition, timeout_sec, spin_interval_sec)
    finally:
        executor.remove_node(node)
