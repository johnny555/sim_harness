# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Stream properties: Hedgehog/Hypothesis-inspired property checking over ROS topics.

Provides ``for_all_messages``, ``eventually``, and ``monotonic`` — combinators
that assert properties over message streams and report exact counterexamples
when a property is violated.

Example:
    from sim_harness.core.stream_properties import for_all_messages, all_of
    from sim_harness.core.predicates import scan_has_min_points, scan_ranges_within

    result = for_all_messages(
        node, executor, "/scan", LaserScan,
        predicate=all_of(scan_has_min_points(100), scan_ranges_within(0.1, 30.0)),
        timeout_sec=5.0,
        description="LIDAR data quality",
    )
    assert result.passed, result.counterexample_details
"""

from dataclasses import dataclass
from typing import TypeVar, Generic, Callable, Type, Optional, List

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile

from sim_harness.core.topic_observer import (
    TopicObserver,
    collect_messages,
    SENSOR_QOS,
)

MsgT = TypeVar('MsgT')

# ---------------------------------------------------------------------------
# Predicate combinators
# ---------------------------------------------------------------------------

Predicate = Callable  # Callable[[MsgT], bool], but keeping it simple for runtime


def all_of(*predicates: Predicate) -> Predicate:
    """All predicates must hold for a given message."""
    def check(msg):
        return all(p(msg) for p in predicates)
    return check


def any_of(*predicates: Predicate) -> Predicate:
    """At least one predicate must hold."""
    def check(msg):
        return any(p(msg) for p in predicates)
    return check


def negate(predicate: Predicate) -> Predicate:
    """Invert a predicate."""
    def check(msg):
        return not predicate(msg)
    return check


# ---------------------------------------------------------------------------
# PropertyResult
# ---------------------------------------------------------------------------

@dataclass
class PropertyResult(Generic[MsgT]):
    """
    Result of checking a property over a message stream.

    When the property fails, ``counterexample`` contains the exact message
    that violated it, and ``first_failure_index`` tells you where in the
    stream it occurred. This is analogous to Hedgehog's counterexample
    reporting.
    """

    passed: bool
    """Whether the property held for all checked messages."""

    total_checked: int
    """Number of messages checked."""

    first_failure_index: Optional[int] = None
    """Index of first message that violated the property (if any)."""

    counterexample: Optional[MsgT] = None
    """The first message that violated the property (if any)."""

    counterexample_details: str = ""
    """Human-readable description of the failure."""

    description: str = ""
    """Human-readable description of the property."""


# ---------------------------------------------------------------------------
# Stream property functions
# ---------------------------------------------------------------------------

def for_all_messages(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    predicate: Predicate,
    timeout_sec: float = 5.0,
    min_samples: int = 1,
    description: str = "",
    qos: Optional[QoSProfile] = None,
) -> PropertyResult[MsgT]:
    """
    Assert that a property holds for ALL messages on a topic.

    This is Hedgehog's ``forAll`` applied to a ROS message stream.
    When the property fails, the result contains the exact counterexample
    message and its index — far more useful than "sensor data invalid".

    Args:
        node: ROS 2 node
        executor: Executor to spin (reuses the caller's)
        topic: Topic to observe
        msg_type: Message type
        predicate: Property that must hold for every message
        timeout_sec: Observation window
        min_samples: Minimum messages required (fails if fewer received)
        description: Human-readable property description
        qos: Optional QoS override

    Returns:
        PropertyResult with pass/fail, total checked, and counterexample
    """
    observer = collect_messages(topic, msg_type, qos=qos)
    obs_result = observer.run(node, executor, timeout_sec)
    messages = obs_result.value

    if len(messages) < min_samples:
        return PropertyResult(
            passed=False,
            total_checked=len(messages),
            counterexample_details=(
                f"Not enough samples: received {len(messages)}, "
                f"need at least {min_samples}"
            ),
            description=description,
        )

    for i, msg in enumerate(messages):
        if not predicate(msg):
            return PropertyResult(
                passed=False,
                total_checked=i + 1,
                first_failure_index=i,
                counterexample=msg,
                counterexample_details=(
                    f"Property violated at message {i}/{len(messages)}: "
                    f"{description}"
                ),
                description=description,
            )

    return PropertyResult(
        passed=True,
        total_checked=len(messages),
        description=description,
    )


def eventually(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    predicate: Predicate,
    timeout_sec: float,
    description: str = "",
    qos: Optional[QoSProfile] = None,
) -> PropertyResult[MsgT]:
    """
    Assert that a predicate becomes true for at least one message.

    Useful for convergence tests: "the robot eventually reaches the goal",
    "localization eventually converges", "the sensor eventually publishes".

    Args:
        node: ROS 2 node
        executor: Executor to spin
        topic: Topic to observe
        msg_type: Message type
        predicate: Condition that must hold for at least one message
        timeout_sec: Maximum observation window
        description: Human-readable description
        qos: Optional QoS override

    Returns:
        PropertyResult — passed=True if any message satisfied the predicate
    """
    observer = collect_messages(topic, msg_type, qos=qos)
    obs_result = observer.run(node, executor, timeout_sec)
    messages = obs_result.value

    for i, msg in enumerate(messages):
        if predicate(msg):
            return PropertyResult(
                passed=True,
                total_checked=i + 1,
                description=description,
            )

    return PropertyResult(
        passed=False,
        total_checked=len(messages),
        counterexample_details=(
            f"Property never held across {len(messages)} messages "
            f"in {timeout_sec}s: {description}"
        ),
        description=description,
    )


def monotonic(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    extract_value: Callable[[MsgT], float],
    timeout_sec: float,
    strict: bool = False,
    description: str = "",
    qos: Optional[QoSProfile] = None,
) -> PropertyResult[MsgT]:
    """
    Assert that an extracted value is monotonically non-decreasing.

    Useful for: timestamps always increasing, cumulative distance traveled,
    monotonic sequence numbers.

    Args:
        node: ROS 2 node
        executor: Executor to spin
        topic: Topic to observe
        msg_type: Message type
        extract_value: Function to extract the numeric value to check
        timeout_sec: Observation window
        strict: If True, requires strictly increasing (no equal values)
        description: Human-readable description
        qos: Optional QoS override

    Returns:
        PropertyResult — passed=True if values are monotonically ordered
    """
    observer = collect_messages(topic, msg_type, qos=qos)
    obs_result = observer.run(node, executor, timeout_sec)
    messages = obs_result.value

    if len(messages) < 2:
        return PropertyResult(
            passed=True,
            total_checked=len(messages),
            description=description,
        )

    prev_val = extract_value(messages[0])
    for i in range(1, len(messages)):
        val = extract_value(messages[i])
        violation = val < prev_val if not strict else val <= prev_val
        if violation:
            return PropertyResult(
                passed=False,
                total_checked=i + 1,
                first_failure_index=i,
                counterexample=messages[i],
                counterexample_details=(
                    f"Monotonicity violated at message {i}: "
                    f"value {val} {'<' if not strict else '<='} previous {prev_val}"
                ),
                description=description,
            )
        prev_val = val

    return PropertyResult(
        passed=True,
        total_checked=len(messages),
        description=description,
    )
