#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for message collector module."""

import pytest

from sim_harness.collector import MessageCollector


# Mock classes for testing without ROS 2
class MockNode:
    """Mock ROS 2 node for testing."""

    def __init__(self):
        self._subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        """Create a mock subscription."""
        sub = MockSubscription(msg_type, topic, callback)
        self._subscriptions.append(sub)
        return sub

    def destroy_subscription(self, subscription):
        """Destroy a subscription."""
        if subscription in self._subscriptions:
            self._subscriptions.remove(subscription)


class MockSubscription:
    """Mock ROS 2 subscription."""

    def __init__(self, msg_type, topic, callback):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback

    def simulate_message(self, msg):
        """Simulate receiving a message."""
        self.callback(msg)


class MockMessage:
    """Mock message for testing."""

    def __init__(self, data):
        self.data = data


class TestMessageCollector:
    """Tests for MessageCollector class."""

    def test_initialization(self):
        """Test collector initialization."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)
        assert collector.count() == 0

    def test_collect_messages(self):
        """Test collecting messages."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("data1"))
        sub.simulate_message(MockMessage("data2"))

        assert collector.count() == 2

        messages = collector.get_messages()
        assert len(messages) == 2
        assert messages[0].data == "data1"
        assert messages[1].data == "data2"

    def test_latest(self):
        """Test getting latest message."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        assert collector.latest() is None

        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("first"))
        sub.simulate_message(MockMessage("second"))
        sub.simulate_message(MockMessage("third"))

        latest = collector.latest()
        assert latest is not None
        assert latest.data == "third"

    def test_clear_messages(self):
        """Test clearing collected messages."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("data"))

        assert collector.count() == 1
        collector.clear()
        assert collector.count() == 0

    def test_max_messages(self):
        """Test max_messages limit."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage, max_messages=3)

        sub = node._subscriptions[0]
        for i in range(10):
            sub.simulate_message(MockMessage(f"msg_{i}"))

        assert collector.count() == 3

    def test_destroy(self):
        """Test destroying collector subscription."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        assert len(node._subscriptions) == 1
        collector.destroy()
        assert len(node._subscriptions) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
