# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
TopicObserver: a fold combinator over ROS 2 message streams.

Replaces the repeated subscribe-spin-check boilerplate found in every
assertion function with a single reusable pattern. A TopicObserver is
parameterized by:
    - initial: the seed value for the fold
    - step:    (accumulator, message) -> accumulator
    - extract: accumulator -> result

This is the functional programming "fold" (reduce) applied to a ROS topic.

Example:
    # Count messages on a topic
    observer = TopicObserver(
        topic="/scan",
        msg_type=LaserScan,
        initial=0,
        step=lambda count, _msg: count + 1,
        extract=lambda count: count,
    )
    result = observer.run(node, executor, timeout_sec=5.0)
    assert result.value > 0

    # Track maximum velocity
    observer = TopicObserver(
        topic="/odom",
        msg_type=Odometry,
        initial=0.0,
        step=lambda mx, msg: max(mx, linear_speed(msg)),
        extract=lambda mx: mx,
    )
    result = observer.run(node, executor, timeout_sec=3.0)
    assert result.value < 0.05, "Robot should be stationary"
"""

import time
from dataclasses import dataclass
from typing import TypeVar, Generic, Callable, Type, Optional, List, Tuple

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sim_harness.core.spin_helpers import spin_for_duration

MsgT = TypeVar('MsgT')
S = TypeVar('S')
R = TypeVar('R')

# Default QoS used across the primitives — best-effort sensor data profile.
SENSOR_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass
class ObservationResult(Generic[R]):
    """Result from a TopicObserver run."""

    value: R
    """The extracted result from the fold."""

    message_count: int
    """Number of messages received during observation."""

    duration_sec: float
    """Actual observation duration in seconds."""

    @property
    def publish_rate_hz(self) -> float:
        """Convenience: approximate publish rate."""
        if self.duration_sec > 0:
            return self.message_count / self.duration_sec
        return 0.0


class TopicObserver(Generic[MsgT, S, R]):
    """
    A fold over a ROS 2 message stream.

    Subscribes to a topic, feeds each message through a step function to
    accumulate state, then extracts a final result. This is the fundamental
    building block for all topic-based assertions.

    Type parameters:
        MsgT: The ROS message type being observed
        S:    The internal accumulator state type
        R:    The final result type (extracted from S)
    """

    def __init__(
        self,
        topic: str,
        msg_type: Type[MsgT],
        initial: S,
        step: Callable[[S, MsgT], S],
        extract: Callable[[S], R],
        qos: Optional[QoSProfile] = None,
    ):
        self.topic = topic
        self.msg_type = msg_type
        self.initial = initial
        self.step = step
        self.extract = extract
        self.qos = qos or SENSOR_QOS

    def run(
        self,
        node: Node,
        executor: SingleThreadedExecutor,
        timeout_sec: float,
    ) -> ObservationResult[R]:
        """
        Execute the observation: subscribe, spin, fold, extract.

        Uses the caller's executor — does NOT create a new one.
        The subscription is created and destroyed within this call.

        Args:
            node: ROS 2 node for the subscription
            executor: Executor to spin (reuses the caller's)
            timeout_sec: How long to observe

        Returns:
            ObservationResult containing the extracted value and metadata
        """
        state = self.initial
        count = 0

        def callback(msg: MsgT):
            nonlocal state, count
            state = self.step(state, msg)
            count += 1

        sub = node.create_subscription(
            self.msg_type, self.topic, callback, self.qos
        )
        try:
            spin_for_duration(executor, timeout_sec)
        finally:
            node.destroy_subscription(sub)

        return ObservationResult(
            value=self.extract(state),
            message_count=count,
            duration_sec=timeout_sec,
        )

    def run_standalone(
        self,
        node: Node,
        timeout_sec: float,
    ) -> ObservationResult[R]:
        """
        Execute with a temporary executor (backward-compatible convenience).

        Creates a SingleThreadedExecutor, adds the node, runs the
        observation, then cleans up. Use ``run()`` with an existing executor
        when possible to avoid repeated executor creation.
        """
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            return self.run(node, executor, timeout_sec)
        finally:
            executor.remove_node(node)

    def map(self, f: Callable[[R], 'R2']) -> 'TopicObserver[MsgT, S, R2]':
        """Transform the extracted result with a pure function."""
        return TopicObserver(
            topic=self.topic,
            msg_type=self.msg_type,
            initial=self.initial,
            step=self.step,
            extract=lambda s: f(self.extract(s)),
            qos=self.qos,
        )


# ---------------------------------------------------------------------------
# Convenience factory functions for common observer patterns
# ---------------------------------------------------------------------------

def collect_messages(
    topic: str,
    msg_type: Type[MsgT],
    qos: Optional[QoSProfile] = None,
) -> TopicObserver[MsgT, List[MsgT], List[MsgT]]:
    """Observer that collects all messages into a list."""
    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=[],
        step=lambda msgs, msg: msgs + [msg],
        extract=lambda msgs: msgs,
        qos=qos,
    )


def count_messages(
    topic: str,
    msg_type: Type[MsgT],
    qos: Optional[QoSProfile] = None,
) -> TopicObserver[MsgT, int, int]:
    """Observer that counts messages."""
    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=0,
        step=lambda n, _: n + 1,
        extract=lambda n: n,
        qos=qos,
    )


def latest_message(
    topic: str,
    msg_type: Type[MsgT],
    qos: Optional[QoSProfile] = None,
) -> TopicObserver[MsgT, Optional[MsgT], Optional[MsgT]]:
    """Observer that captures the latest message."""
    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=None,
        step=lambda _, msg: msg,
        extract=lambda msg: msg,
        qos=qos,
    )


def track_max(
    topic: str,
    msg_type: Type[MsgT],
    extract: Callable[[MsgT], float],
    qos: Optional[QoSProfile] = None,
) -> TopicObserver[MsgT, Optional[float], Optional[float]]:
    """Observer that tracks the maximum of an extracted value."""
    def step(mx: Optional[float], msg: MsgT) -> Optional[float]:
        val = extract(msg)
        if mx is None:
            return val
        return max(mx, val)

    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=None,
        step=step,
        extract=lambda mx: mx,
        qos=qos,
    )


def track_timestamps(
    topic: str,
    msg_type: Type[MsgT],
    qos: Optional[QoSProfile] = None,
) -> TopicObserver[MsgT, List[float], List[float]]:
    """Observer that records wall-clock receipt times for rate calculation."""
    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=[],
        step=lambda times, _: times + [time.monotonic()],
        extract=lambda times: times,
        qos=qos,
    )


# ---------------------------------------------------------------------------
# ParallelObserver: run multiple observers in a single spin period
# ---------------------------------------------------------------------------

class ParallelObserver:
    """
    Run multiple TopicObservers with a single spin period.

    All subscriptions are created before spinning, so all observers
    receive messages simultaneously during the same time window.

    Example:
        odom_result, scan_result = ParallelObserver(
            observe_max_velocity("/odom"),
            count_messages("/scan", LaserScan),
        ).run(node, executor, 5.0)
    """

    def __init__(self, *observers: TopicObserver):
        self.observers = observers

    def run(
        self,
        node: Node,
        executor: SingleThreadedExecutor,
        timeout_sec: float,
    ) -> Tuple[ObservationResult, ...]:
        """Run all observers in parallel, return tuple of results."""
        states = [obs.initial for obs in self.observers]
        counts = [0] * len(self.observers)
        subs = []

        for i, obs in enumerate(self.observers):
            # Use default argument capture to bind loop variable
            def make_cb(idx, observer):
                def cb(msg):
                    nonlocal states, counts
                    states[idx] = observer.step(states[idx], msg)
                    counts[idx] += 1
                return cb

            sub = node.create_subscription(
                obs.msg_type, obs.topic, make_cb(i, obs), obs.qos
            )
            subs.append(sub)

        try:
            spin_for_duration(executor, timeout_sec)
        finally:
            for sub in subs:
                node.destroy_subscription(sub)

        return tuple(
            ObservationResult(
                value=obs.extract(states[i]),
                message_count=counts[i],
                duration_sec=timeout_sec,
            )
            for i, obs in enumerate(self.observers)
        )

    def run_standalone(
        self,
        node: Node,
        timeout_sec: float,
    ) -> Tuple[ObservationResult, ...]:
        """Run with a temporary executor (backward-compatible convenience)."""
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            return self.run(node, executor, timeout_sec)
        finally:
            executor.remove_node(node)
