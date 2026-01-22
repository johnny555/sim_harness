#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for message collector module."""

import pytest
import threading
import time

from sim_harness.core.message_collector import MessageCollector


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

        assert collector.topic == "/test_topic"
        assert collector.msg_type == MockMessage
        assert collector.count() == 0

    def test_collect_messages(self):
        """Test collecting messages."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        # Simulate receiving messages
        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("data1"))
        sub.simulate_message(MockMessage("data2"))

        assert collector.count() == 2

        messages = collector.get_messages()
        assert len(messages) == 2
        assert messages[0].data == "data1"
        assert messages[1].data == "data2"

    def test_get_latest(self):
        """Test getting latest message."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        assert collector.get_latest() is None

        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("first"))
        sub.simulate_message(MockMessage("second"))
        sub.simulate_message(MockMessage("third"))

        latest = collector.get_latest()
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

    def test_callback_invoked(self):
        """Test that custom callback is invoked."""
        node = MockNode()
        callback_data = []

        def custom_callback(msg):
            callback_data.append(msg.data)

        collector = MessageCollector(
            node, "/test_topic", MockMessage,
            callback=custom_callback
        )

        sub = node._subscriptions[0]
        sub.simulate_message(MockMessage("test"))

        assert len(callback_data) == 1
        assert callback_data[0] == "test"

    def test_thread_safety(self):
        """Test thread-safe message collection."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)
        sub = node._subscriptions[0]

        # Simulate concurrent message production
        def producer():
            for i in range(100):
                sub.simulate_message(MockMessage(f"msg_{i}"))
                time.sleep(0.001)

        threads = [threading.Thread(target=producer) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Should have 500 messages (5 threads * 100 messages each)
        assert collector.count() == 500

    def test_destroy(self):
        """Test destroying collector subscription."""
        node = MockNode()
        collector = MessageCollector(node, "/test_topic", MockMessage)

        assert len(node._subscriptions) == 1
        collector.destroy()
        assert len(node._subscriptions) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
