# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Base test fixture for ROS 2 simulation tests.

Provides pytest fixtures with automatic ROS 2 node lifecycle management.
"""

import time
from typing import Any, Callable, Dict, List, Optional, Type, TypeVar

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from sim_harness.core.message_collector import MessageCollector
from sim_harness.core.spin_helpers import (
    spin_for_duration,
    spin_until_condition,
)
from sim_harness.core.test_isolation import (
    get_test_isolation_config,
    apply_test_isolation,
    generate_test_node_name,
    TestIsolationConfig,
)
from sim_harness.validation.requirement_validator import RequirementValidator

MsgT = TypeVar('MsgT')


class SimTestFixture(RequirementValidator):
    """
    Base class for ROS 2 simulation integration tests.

    Provides common utilities for simulation testing:
    - Automatic ROS 2 node creation/destruction per test
    - Message collection on arbitrary topics
    - Spin helpers for waiting on messages or conditions
    - Test isolation configuration
    - Requirements validation support

    Usage with pytest:
        class TestMyRobot(SimTestFixture):
            def test_sensor_publishes(self):
                collector = self.create_message_collector('/scan', LaserScan)
                self.spin_for_duration(5.0)
                assert collector.count() > 0, "No scan messages received"
    """

    # Class-level ROS 2 initialization tracking
    _rclpy_initialized: bool = False

    @pytest.fixture(autouse=True)
    def setup_ros(self) -> None:
        """
        Pytest fixture for ROS 2 setup/teardown.

        Automatically runs before and after each test method.
        """
        # Setup
        self._setup_test()
        yield
        # Teardown
        self._teardown_test()

    def _setup_test(self) -> None:
        """Initialize ROS 2 node and executor for the test."""
        # Initialize rclpy if needed
        if not SimTestFixture._rclpy_initialized:
            rclpy.init()
            SimTestFixture._rclpy_initialized = True

        # Get test isolation config
        self._isolation_config = get_test_isolation_config()
        apply_test_isolation(self._isolation_config)

        # Create unique node name
        node_name = generate_test_node_name(
            "sim_test",
            self._isolation_config.domain_id
        )

        # Create node with use_sim_time
        self._node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )

        # Create executor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Storage for message collectors
        self._collectors: Dict[str, MessageCollector] = {}

        self._node.get_logger().info(
            f"Test initialized: {node_name} (domain: {self._isolation_config.domain_id})"
        )

    def _teardown_test(self) -> None:
        """Clean up ROS 2 resources after test."""
        # Destroy all collectors
        for collector in self._collectors.values():
            collector.destroy()
        self._collectors.clear()

        # Remove node from executor
        if hasattr(self, '_executor') and hasattr(self, '_node'):
            self._executor.remove_node(self._node)
            self._node.destroy_node()

    @classmethod
    def teardown_class(cls) -> None:
        """Clean up ROS 2 at end of test class."""
        if SimTestFixture._rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            SimTestFixture._rclpy_initialized = False

    def spin_for_duration(self, duration_sec: float) -> None:
        """
        Spin the executor for a specified duration.

        Processes callbacks for the given duration, allowing time for
        messages to be received and simulation to progress.

        Args:
            duration_sec: How long to spin (seconds)
        """
        spin_for_duration(self._executor, duration_sec)

    def spin_until_condition(
        self,
        condition: Callable[[], bool],
        timeout_sec: float
    ) -> bool:
        """
        Spin until a condition is met or timeout occurs.

        Args:
            condition: Function returning True when condition is met
            timeout_sec: Maximum time to wait (seconds)

        Returns:
            True if condition was met, False if timeout occurred
        """
        return spin_until_condition(self._executor, condition, timeout_sec)

    def create_message_collector(
        self,
        topic: str,
        msg_type: Type[MsgT],
        key: Optional[str] = None,
        **kwargs: Any
    ) -> MessageCollector[MsgT]:
        """
        Create a message collector for a topic.

        The collector is stored internally and managed by the fixture.

        Args:
            topic: Topic to subscribe to
            msg_type: Message type class
            key: Optional key for later retrieval (defaults to topic name)
            **kwargs: Additional arguments for MessageCollector

        Returns:
            Message collector instance
        """
        collector = MessageCollector(self._node, topic, msg_type, **kwargs)
        storage_key = key if key else topic
        self._collectors[storage_key] = collector
        return collector

    def get_collector(self, key: str) -> Optional[MessageCollector]:
        """
        Get a message collector by key.

        Args:
            key: Collector key (topic name or custom key)

        Returns:
            Message collector or None if not found
        """
        return self._collectors.get(key)

    def clear_messages(self, key: str) -> None:
        """
        Clear messages from a collector.

        Args:
            key: Collector key
        """
        collector = self._collectors.get(key)
        if collector:
            collector.clear()

    @property
    def node(self) -> Node:
        """Get the test node."""
        return self._node

    @property
    def executor(self) -> SingleThreadedExecutor:
        """Get the executor."""
        return self._executor

    @property
    def isolation_config(self) -> TestIsolationConfig:
        """Get the test isolation configuration."""
        return self._isolation_config

    def get_logger(self):
        """Get the logger for this test."""
        return self._node.get_logger()


# Convenience fixture for standalone pytest usage
@pytest.fixture
def ros_node():
    """
    Pytest fixture providing a ROS 2 node.

    Usage:
        def test_something(ros_node):
            # ros_node is a rclpy.node.Node instance
            pub = ros_node.create_publisher(String, '/topic', 10)
    """
    if not SimTestFixture._rclpy_initialized:
        rclpy.init()
        SimTestFixture._rclpy_initialized = True

    config = get_test_isolation_config()
    apply_test_isolation(config)

    node_name = generate_test_node_name("pytest_node", config.domain_id)
    node = rclpy.create_node(
        node_name,
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ]
    )

    yield node

    node.destroy_node()


@pytest.fixture
def ros_executor(ros_node):
    """
    Pytest fixture providing a ROS 2 executor with node.

    Usage:
        def test_something(ros_executor, ros_node):
            collector = MessageCollector(ros_node, '/topic', String)
            spin_for_duration(ros_executor, 1.0)
    """
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    yield executor

    executor.remove_node(ros_node)
