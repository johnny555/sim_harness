# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Timing and latency validation assertions.

Provides functions to validate publish rates and message latency.
"""

import time
from dataclasses import dataclass
from typing import List, Optional, Type, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg

from sim_harness.core.spin_helpers import spin_for_duration

MsgT = TypeVar('MsgT')


@dataclass
class TimingResult:
    """Result of a timing assertion."""

    within_bounds: bool
    """Whether timing is within expected bounds."""

    measured_rate_hz: float
    """Measured publish rate (Hz)."""

    min_latency_ms: float
    """Minimum latency observed (ms)."""

    max_latency_ms: float
    """Maximum latency observed (ms)."""

    avg_latency_ms: float
    """Average latency (ms)."""

    details: str
    """Human-readable details."""


def assert_publish_rate(
    node: Node,
    topic: str,
    msg_type: Type[MsgT],
    expected_rate_hz: float,
    tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0
) -> TimingResult:
    """
    Assert that a topic publishes at the expected rate.

    Note:
        This function temporarily adds the node to an internal executor
        and removes it when done. The node remains valid after the call.

    Args:
        node: ROS 2 node
        topic: Topic to monitor
        msg_type: Message type class
        expected_rate_hz: Expected publish rate
        tolerance_percent: Acceptable deviation (percent)
        sample_duration_sec: How long to sample

    Returns:
        TimingResult with:
        - within_bounds: True if measured rate is within tolerance
        - measured_rate_hz: Actual publish rate observed
        - min_interval_ms, max_interval_ms, avg_interval_ms: Inter-message
          intervals (stored in min_latency_ms, max_latency_ms, avg_latency_ms
          fields for API compatibility)
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    message_times: List[float] = []

    def callback(msg):
        message_times.append(time.monotonic())

    qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(msg_type, topic, callback, qos)

    try:
        spin_for_duration(executor, sample_duration_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    result = TimingResult(
        within_bounds=False,
        measured_rate_hz=0.0,
        min_latency_ms=0.0,
        max_latency_ms=0.0,
        avg_latency_ms=0.0,
        details=""
    )

    if len(message_times) < 2:
        result.details = f"Not enough messages received ({len(message_times)})"
        return result

    # Calculate rate
    result.measured_rate_hz = (len(message_times) - 1) / (message_times[-1] - message_times[0])

    # Calculate inter-message intervals
    intervals = [
        (message_times[i] - message_times[i-1]) * 1000
        for i in range(1, len(message_times))
    ]

    result.min_latency_ms = min(intervals)
    result.max_latency_ms = max(intervals)
    result.avg_latency_ms = sum(intervals) / len(intervals)

    # Check tolerance
    if expected_rate_hz > 0:
        deviation = abs(result.measured_rate_hz - expected_rate_hz) / expected_rate_hz * 100
        result.within_bounds = deviation <= tolerance_percent
    else:
        result.within_bounds = len(message_times) > 0

    result.details = (
        f"Rate: {result.measured_rate_hz:.1f} Hz, "
        f"Expected: {expected_rate_hz:.1f} Hz, "
        f"Interval range: [{result.min_latency_ms:.1f}, {result.max_latency_ms:.1f}] ms"
    )

    return result


def assert_latency(
    node: Node,
    topic: str,
    msg_type: Type[MsgT],
    max_latency_ms: float,
    sample_duration_sec: float = 5.0
) -> TimingResult:
    """
    Assert that message latency is within bounds.

    Compares header timestamp to receive time to measure end-to-end latency.

    Note:
        This function temporarily adds the node to an internal executor
        and removes it when done. The node remains valid after the call.

    Args:
        node: ROS 2 node
        topic: Topic to monitor
        msg_type: Message type class (must have header with stamp)
        max_latency_ms: Maximum acceptable latency
        sample_duration_sec: How long to sample

    Returns:
        TimingResult with:
        - within_bounds: True if max observed latency <= max_latency_ms
        - measured_rate_hz: Message rate during sampling period
        - min_latency_ms, max_latency_ms, avg_latency_ms: Latency statistics

    Note:
        Messages without a header.stamp field are silently skipped. If all
        messages lack timestamps, returns with details "No messages with
        timestamps received".
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    latencies: List[float] = []

    def callback(msg):
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            now = node.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            latency_ns = now.nanoseconds - msg_time.nanoseconds
            latencies.append(latency_ns / 1e6)  # Convert to ms

    qos = QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    sub = node.create_subscription(msg_type, topic, callback, qos)

    try:
        spin_for_duration(executor, sample_duration_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    result = TimingResult(
        within_bounds=False,
        measured_rate_hz=0.0,
        min_latency_ms=0.0,
        max_latency_ms=0.0,
        avg_latency_ms=0.0,
        details=""
    )

    if not latencies:
        result.details = "No messages with timestamps received"
        return result

    result.min_latency_ms = min(latencies)
    result.max_latency_ms = max(latencies)
    result.avg_latency_ms = sum(latencies) / len(latencies)
    result.measured_rate_hz = len(latencies) / sample_duration_sec

    result.within_bounds = result.max_latency_ms <= max_latency_ms

    result.details = (
        f"Latency: min={result.min_latency_ms:.1f}ms, "
        f"max={result.max_latency_ms:.1f}ms, "
        f"avg={result.avg_latency_ms:.1f}ms"
    )

    return result


def assert_transform_available(
    node: Node,
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 5.0,
    max_age_ms: float = 1000.0
) -> bool:
    """
    Assert that a TF transform is available and fresh.

    Args:
        node: ROS 2 node
        target_frame: Target frame
        source_frame: Source frame
        timeout_sec: Maximum time to wait
        max_age_ms: Maximum age of transform (ms)

    Returns:
        True if transform is available and fresh
    """
    from tf2_ros import Buffer, TransformListener

    temp_node = rclpy.create_node(f"tf_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, temp_node)

    try:
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            try:
                transform = tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )

                # Check age
                now = temp_node.get_clock().now()
                transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
                age_ms = (now.nanoseconds - transform_time.nanoseconds) / 1e6

                if age_ms <= max_age_ms:
                    return True

            except Exception:
                pass

        return False

    finally:
        executor.remove_node(temp_node)
        temp_node.destroy_node()


def assert_action_server_responsive(
    node: Node,
    action_name: str,
    action_type: Type,
    max_response_time_ms: float = 1000.0
) -> bool:
    """
    Assert that an action server responds within timeout.

    Args:
        node: ROS 2 node
        action_name: Action server name
        action_type: Action type class
        max_response_time_ms: Maximum response time (ms)

    Returns:
        True if server responds within timeout
    """
    temp_node = rclpy.create_node(f"action_latency_checker_{int(time.time() * 1000) % 10000}")
    executor = SingleThreadedExecutor()
    executor.add_node(temp_node)

    action_client = ActionClient(temp_node, action_type, action_name)

    try:
        start_time = time.monotonic()
        available = action_client.wait_for_server(timeout_sec=max_response_time_ms / 1000.0)
        response_time_ms = (time.monotonic() - start_time) * 1000

        return available and response_time_ms <= max_response_time_ms

    finally:
        action_client.destroy()
        executor.remove_node(temp_node)
        temp_node.destroy_node()
