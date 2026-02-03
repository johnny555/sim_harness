# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Lightweight message collector for ROS 2 topics."""

import threading

from rclpy.qos import QoSProfile


class MessageCollector:
    """Subscribe to a ROS topic and buffer incoming messages.

    Managed automatically when created via
    :meth:`SimTestFixture.create_message_collector`.

    Thread-safe: callbacks run on the executor thread while test code
    accesses messages from the test thread.
    """

    def __init__(self, node, topic: str, msg_type, qos_profile=None, max_messages=None):
        self._messages = []
        self._max = max_messages
        self._node = node
        self._lock = threading.Lock()
        qos = qos_profile or QoSProfile(depth=10)
        self._sub = node.create_subscription(msg_type, topic, self._on_msg, qos)

    def _on_msg(self, msg):
        with self._lock:
            if self._max is None or len(self._messages) < self._max:
                self._messages.append(msg)

    def get_messages(self):
        """Return a copy of all collected messages."""
        with self._lock:
            return list(self._messages)

    def count(self) -> int:
        """Number of messages received so far."""
        with self._lock:
            return len(self._messages)

    def clear(self):
        """Discard all collected messages."""
        with self._lock:
            self._messages.clear()

    def latest(self):
        """Most recent message, or ``None``."""
        with self._lock:
            return self._messages[-1] if self._messages else None

    def destroy(self):
        """Remove the underlying subscription."""
        self._node.destroy_subscription(self._sub)
