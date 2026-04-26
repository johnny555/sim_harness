# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Pytest fixtures for ROS 2 integration tests.

Project-neutral primitives. Tests request only what they need — that's the
whole point of moving off the ``BaseIntegrationTest`` inheritance model.

Auto-loaded via the ``pytest11`` entry point in ``setup.cfg``; once
``sim_harness`` is on the import path, importing in ``conftest.py`` is
unnecessary.

Quick start::

    def test_lidar_publishes(ros_node, message_collector_factory):
        scan = message_collector_factory(LaserScan, '/scan')
        ros_node.spin_for(2.0)
        assert scan.count() > 0

For Gazebo-backed tests, pair these fixtures with a ``@launch_pytest.fixture``
declared in the consuming module's ``conftest.py`` (or in the test file
itself), and bind tests with ``@pytest.mark.launch(fixture=...)``::

    import launch_pytest

    @launch_pytest.fixture(scope='module')
    def my_launch():
        return LaunchDescription([...])

    @pytest.mark.launch(fixture=my_launch)
    def test_lidar(my_launch, ros_node, message_collector_factory):
        scan = message_collector_factory(LaserScan, '/scan')
        ros_node.spin_until(lambda: scan.count() > 0, timeout_sec=10.0)

Public surface
--------------
* :func:`ros_context` — session-scope ``rclpy.init()`` / ``shutdown``.
* :func:`ros_node` — function-scope node + background spinner thread.
* :func:`message_collector_factory` — factory yielding :class:`MessageCollector`
  instances; auto-destroyed on teardown.
* :func:`wait_for_topic` — free function: block until topic publishes (or
  timeout). Useful for top-of-test "wait for the simulator" checks.
"""

from __future__ import annotations

import threading
import time
import uuid
from typing import TypeVar

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sim_harness.collector import MessageCollector

MsgT = TypeVar("MsgT")


# ---------------------------------------------------------------------------
# rclpy lifecycle (session-scope)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def ros_context():
    """Initialize ``rclpy`` once per pytest session.

    Idempotent: if another fixture or test has already initialized rclpy,
    we yield without doing anything (and don't shut it down at session end —
    the original initializer owns it).
    """
    we_initialized = False
    try:
        if not rclpy.ok():
            rclpy.init()
            we_initialized = True
    except RuntimeError:
        # ``rclpy.ok()`` raises if context was destroyed; treat as initialized.
        pass

    yield

    if we_initialized:
        try:
            rclpy.shutdown()
        except Exception:
            # Already shut down by something else — fine.
            pass


# ---------------------------------------------------------------------------
# Node + spinner (function-scope)
# ---------------------------------------------------------------------------


class _SpinningNode:
    """Wraps an ``rclpy.Node`` with a background ``MultiThreadedExecutor``.

    The executor runs on a daemon thread so subscription / service callbacks
    fire concurrently with the test's main thread, exactly the way
    ``BaseIntegrationTest.spin_for_duration`` used to (but without making the
    test thread responsible for pumping callbacks).
    """

    def __init__(self, node: Node):
        self.node = node
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(node)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name=f"{node.get_name()}-spinner", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                # Spurious wake — keep going.
                if self._stop.is_set():
                    break

    # ---- ergonomic API mirroring the old BaseIntegrationTest -----------

    def spin_for(self, duration_sec: float) -> None:
        """Sleep for ``duration_sec`` while callbacks fire on the spinner thread."""
        time.sleep(duration_sec)

    def spin_until(self, predicate, timeout_sec: float, poll_sec: float = 0.05) -> bool:
        """Poll ``predicate()`` until true or ``timeout_sec`` elapses."""
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(poll_sec)
        return False

    # ---- attribute pass-through to the underlying Node ------------------

    def __getattr__(self, name):
        return getattr(self.node, name)

    # ---- shutdown ------------------------------------------------------

    def shutdown(self) -> None:
        self._stop.set()
        self._executor.remove_node(self.node)
        self._executor.shutdown()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        self._thread.join(timeout=2.0)


@pytest.fixture
def ros_node(ros_context, request) -> _SpinningNode:
    """Per-test ROS 2 node with a background spinner thread.

    Yields a :class:`_SpinningNode` that delegates attribute access to the
    underlying ``rclpy.Node``, plus convenience methods ``spin_for(s)`` and
    ``spin_until(pred, timeout)`` that mirror the old
    ``BaseIntegrationTest`` API. The node uses ``use_sim_time=True`` by
    default — override by setting ``ros_node.use_sim_time = False`` in your
    test (or pass it via parameter overrides yourself).

    The node name combines the test name with a short uuid suffix to avoid
    parameter-server collisions when tests run in parallel.
    """
    test_name = request.node.name.replace("[", "_").replace("]", "").replace("/", "_")
    suffix = uuid.uuid4().hex[:8]
    node = rclpy.create_node(
        f"{test_name}_{suffix}",
        automatically_declare_parameters_from_overrides=True,
    )
    if not node.has_parameter("use_sim_time"):
        node.declare_parameter("use_sim_time", True)

    spinning = _SpinningNode(node)
    try:
        yield spinning
    finally:
        spinning.shutdown()


# ---------------------------------------------------------------------------
# Message collectors (function-scope factory)
# ---------------------------------------------------------------------------


@pytest.fixture
def message_collector_factory(ros_node):
    """Factory fixture yielding ``MessageCollector`` instances.

    Each call subscribes the test's ``ros_node`` to ``topic`` and starts
    buffering ``msg_type`` messages. All collectors created during the test
    are destroyed automatically on teardown.

    Example::

        def test_lidar(ros_node, message_collector_factory):
            scan  = message_collector_factory(LaserScan, '/scan')
            stamp = message_collector_factory(Imu, '/imu')
            ros_node.spin_for(2.0)
            assert scan.count() > 0 and stamp.count() > 0
    """
    created: list[MessageCollector] = []

    def _make(msg_type, topic: str, *, qos=None, max_messages=None) -> MessageCollector:
        collector = MessageCollector(
            ros_node.node, topic, msg_type,
            qos_profile=qos, max_messages=max_messages,
        )
        created.append(collector)
        return collector

    yield _make

    for collector in created:
        try:
            collector.destroy()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Topic-readiness helper (free function, not a fixture)
# ---------------------------------------------------------------------------


def wait_for_topic(node: Node, topic: str, msg_type, timeout_sec: float = 30.0) -> bool:
    """Block until ``topic`` publishes its first message, or ``timeout_sec`` elapses.

    Useful for "wait for the simulator to be ready" checks at the top of a
    test module. Uses the supplied ``node``'s subscription machinery (and
    therefore must be called while ``node`` is being spun — typically true
    when using the :func:`ros_node` fixture).

    Returns True if a message arrived, False if timed out.
    """
    seen = threading.Event()

    def _cb(_msg):
        seen.set()

    sub = node.create_subscription(msg_type, topic, _cb, 10)
    try:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            if seen.is_set():
                return True
            time.sleep(0.05)
        return False
    finally:
        node.destroy_subscription(sub)
