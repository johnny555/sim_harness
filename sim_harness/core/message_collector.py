# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Thread-safe message collector for ROS 2 topics.

Provides subscription and message storage for testing topic data.
"""

import threading
from typing import Any, Callable, Generic, List, Optional, Type, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Generic message type
MsgT = TypeVar('MsgT')


class MessageCollector(Generic[MsgT]):
    """
    Thread-safe message collector for a ROS 2 topic.

    Subscribes to a topic and stores all received messages for later
    inspection during tests.

    Example:
        collector = MessageCollector(node, '/scan', LaserScan)
        spin_for_duration(executor, 5.0)
        messages = collector.get_messages()
        assert len(messages) > 0
    """

    def __init__(
        self,
        node: Node,
        topic: str,
        msg_type: Type[MsgT],
        qos_profile: Optional[QoSProfile] = None,
        callback: Optional[Callable[[MsgT], None]] = None
    ):
        """
        Create a message collector.

        Args:
            node: ROS 2 node for subscription
            topic: Topic to subscribe to
            msg_type: Message type class
            qos_profile: Optional QoS profile (default: best effort, volatile)
            callback: Optional callback invoked on each message
        """
        self._node = node
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        self._messages: List[MsgT] = []
        self._lock = threading.Lock()

        # Default QoS for sensor data
        if qos_profile is None:
            qos_profile = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )

        self._subscription = node.create_subscription(
            msg_type,
            topic,
            self._on_message,
            qos_profile
        )

    def _on_message(self, msg: MsgT) -> None:
        """Handle incoming message."""
        with self._lock:
            self._messages.append(msg)

        if self._callback is not None:
            self._callback(msg)

    def get_messages(self) -> List[MsgT]:
        """
        Get all collected messages.

        Returns:
            Copy of all messages received so far
        """
        with self._lock:
            return list(self._messages)

    def get_latest(self) -> Optional[MsgT]:
        """
        Get the most recent message.

        Returns:
            Latest message or None if no messages received
        """
        with self._lock:
            return self._messages[-1] if self._messages else None

    def count(self) -> int:
        """
        Get the number of collected messages.

        Returns:
            Message count
        """
        with self._lock:
            return len(self._messages)

    def clear(self) -> None:
        """Clear all collected messages."""
        with self._lock:
            self._messages.clear()

    def wait_for_messages(
        self,
        min_count: int,
        timeout_sec: float,
        executor: Any
    ) -> bool:
        """
        Block until minimum message count is reached.

        Args:
            min_count: Minimum number of messages to wait for
            timeout_sec: Maximum time to wait (seconds)
            executor: ROS 2 executor to spin

        Returns:
            True if minimum count reached, False if timeout
        """
        from sim_harness.core.spin_helpers import spin_until_condition
        return spin_until_condition(
            executor,
            lambda: self.count() >= min_count,
            timeout_sec
        )

    @property
    def topic(self) -> str:
        """Get the topic name."""
        return self._topic

    @property
    def msg_type(self) -> Type[MsgT]:
        """Get the message type."""
        return self._msg_type

    def destroy(self) -> None:
        """
        Destroy the subscription and release resources.

        Safe to call multiple times; subsequent calls are no-ops.
        Does not clear collected messages - call clear() first if needed.
        """
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
