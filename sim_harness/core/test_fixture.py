# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test fixture for ROS 2 simulation tests.

Provides :class:`SimTestFixture` — the single entry point for all
sim_harness tests. Set ``LAUNCH_PACKAGE`` / ``LAUNCH_FILE`` class
attributes to enable automatic Gazebo lifecycle management; leave
them empty for tests that only need a ROS 2 node.

Example (with simulation)::

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'turtlebot3_gazebo'
        LAUNCH_FILE = 'turtlebot3_world.launch.py'
        WORLD = 'turtlebot3_world'

        def test_sensor_publishes(self):
            collector = self.create_message_collector('/scan', LaserScan)
            self.spin_for_duration(5.0)
            assert collector.count() > 0

Example (ROS-only, no sim management)::

    class TestMyNode(SimTestFixture):
        def test_service_available(self):
            from sim_harness import assert_node_running
            assert assert_node_running(self.node, 'my_node')
"""

from typing import Any, Callable, Dict, Optional, Type, TypeVar

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

    Provides:

    - Automatic ROS 2 node creation/destruction per test
    - Optional Gazebo simulation lifecycle (set ``LAUNCH_PACKAGE``)
    - Message collection on arbitrary topics
    - Spin helpers for waiting on messages or conditions
    - Test isolation (unique ROS_DOMAIN_ID per test)
    - Requirements validation support

    Simulation management attributes (set on subclass to enable):

    - ``LAUNCH_PACKAGE``: ROS package containing the launch file
    - ``LAUNCH_FILE``: Launch file name
    - ``LAUNCH_ARGS``: Dict of additional launch arguments
    - ``WORLD``: World name (used to determine if restart is needed)
    - ``ROBOT_MODEL``: Robot model name (used for restart decisions)
    - ``STARTUP_TIMEOUT``: Max seconds to wait for simulation start
    - ``GAZEBO_STARTUP_DELAY``: Extra wait after Gazebo reports ready
    - ``REQUIRE_SIM``: If True, skip tests when sim can't start
    - ``USE_EXISTING_SIM``: If True, don't manage sim lifecycle

    Simulations are automatically reused when configs match across
    test classes, and restarted when they differ.
    """

    # Class-level ROS 2 initialization tracking
    _rclpy_initialized: bool = False

    # --- Simulation management (optional) ---
    LAUNCH_PACKAGE: str = ""
    LAUNCH_FILE: str = ""
    LAUNCH_ARGS: dict = {}
    WORLD: str = ""
    ROBOT_MODEL: str = ""
    STARTUP_TIMEOUT: float = 60.0
    GAZEBO_STARTUP_DELAY: float = 5.0
    REQUIRE_SIM: bool = True
    USE_EXISTING_SIM: bool = False

    @pytest.fixture(autouse=True)
    def setup_ros(self) -> None:
        """
        Pytest fixture for full setup/teardown.

        Handles simulation lifecycle (if configured), then ROS 2
        node creation, then calls ``on_setup()``. Reverses on teardown.
        """
        # Step 1: Simulation management
        sim_managed = self._setup_simulation()

        # Step 2: ROS node setup
        self._setup_test()
        try:
            self.on_setup()
        except BaseException:
            self.on_teardown()
            self._teardown_test()
            if sim_managed:
                self._release_simulation()
            raise

        yield

        # Teardown (reverse order)
        self.on_teardown()
        self._teardown_test()
        if sim_managed:
            self._release_simulation()

    def on_setup(self) -> None:
        """
        Hook for subclass setup after ROS initialization.

        Override this to create TF listeners, publishers, collectors, etc.
        """
        pass

    def on_teardown(self) -> None:
        """
        Hook for subclass teardown before ROS cleanup.

        Override this to destroy publishers, timers, or other resources.
        """
        pass

    # -----------------------------------------------------------------
    # Simulation lifecycle (lazy imports — no Gazebo dependency if unused)
    # -----------------------------------------------------------------

    def _setup_simulation(self) -> bool:
        """
        Start or reuse a simulation if configured.

        Returns True if simulation was actively managed (needs release).
        """
        cls = self.__class__

        if cls.USE_EXISTING_SIM:
            try:
                from sim_harness.simulator.gazebo_backend import GazeboBackend
                if not GazeboBackend().is_running() and cls.REQUIRE_SIM:
                    pytest.skip(
                        "Simulation not running and USE_EXISTING_SIM=True"
                    )
            except ImportError:
                if cls.REQUIRE_SIM:
                    pytest.skip("Gazebo backend not available")
            return False

        if not cls.LAUNCH_PACKAGE or not cls.LAUNCH_FILE:
            return False

        try:
            from sim_harness.simulator.simulation_manager import (
                SimulationRequest,
                get_simulation_manager,
            )
        except ImportError:
            if cls.REQUIRE_SIM:
                pytest.skip("sim_harness.simulator not available")
            return False

        manager = get_simulation_manager()
        sim_request = SimulationRequest(
            package=cls.LAUNCH_PACKAGE,
            launch_file=cls.LAUNCH_FILE,
            launch_args=cls.LAUNCH_ARGS,
            world=cls.WORLD,
            robot_model=cls.ROBOT_MODEL,
        )

        try:
            success = manager.request(
                sim_request,
                startup_timeout=cls.STARTUP_TIMEOUT,
                gazebo_delay=cls.GAZEBO_STARTUP_DELAY,
                require_sim=cls.REQUIRE_SIM,
            )
            if not success and cls.REQUIRE_SIM:
                pytest.skip(
                    f"Failed to start simulation: "
                    f"{cls.LAUNCH_PACKAGE}/{cls.LAUNCH_FILE}"
                )
                return False
        except RuntimeError as e:
            if cls.REQUIRE_SIM:
                pytest.skip(str(e))
            return False

        return True

    def _release_simulation(self) -> None:
        """Release the simulation (but don't stop — allow reuse)."""
        try:
            from sim_harness.simulator.simulation_manager import (
                get_simulation_manager,
            )
            get_simulation_manager().release()
        except ImportError:
            pass

    # -----------------------------------------------------------------
    # ROS 2 node lifecycle
    # -----------------------------------------------------------------

    def _setup_test(self) -> None:
        """Initialize ROS 2 node and executor for the test."""
        self._isolation_config = get_test_isolation_config()
        apply_test_isolation(self._isolation_config)

        if not SimTestFixture._rclpy_initialized:
            rclpy.init()
            SimTestFixture._rclpy_initialized = True
        else:
            try:
                context = rclpy.get_default_context()
                if not context.ok():
                    rclpy.init()
            except Exception:
                rclpy.init()

        node_name = generate_test_node_name(
            "sim_test",
            self._isolation_config.domain_id,
        )

        self._node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._collectors: Dict[str, MessageCollector] = {}

        self._node.get_logger().info(
            f"Test initialized: {node_name} "
            f"(domain: {self._isolation_config.domain_id})"
        )

    def _teardown_test(self) -> None:
        """Clean up ROS 2 resources after test."""
        for collector in self._collectors.values():
            collector.destroy()
        self._collectors.clear()

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

    # -----------------------------------------------------------------
    # Spin helpers
    # -----------------------------------------------------------------

    def spin_for_duration(self, duration_sec: float) -> None:
        """
        Spin the executor for a specified duration.

        Args:
            duration_sec: How long to spin (seconds)
        """
        spin_for_duration(self._executor, duration_sec)

    def spin_until_condition(
        self,
        condition: Callable[[], bool],
        timeout_sec: float,
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

    # -----------------------------------------------------------------
    # Message collectors
    # -----------------------------------------------------------------

    def create_message_collector(
        self,
        topic: str,
        msg_type: Type[MsgT],
        key: Optional[str] = None,
        **kwargs: Any,
    ) -> 'MessageCollector[MsgT]':
        """
        Create a message collector for a topic.

        The collector is managed by the fixture and destroyed on teardown.

        Args:
            topic: Topic to subscribe to
            msg_type: Message type class
            key: Optional key for later retrieval (defaults to topic name)
            **kwargs: Additional arguments for MessageCollector

        Returns:
            MessageCollector typed to the message type provided
        """
        collector = MessageCollector(self._node, topic, msg_type, **kwargs)
        storage_key = key if key else topic
        self._collectors[storage_key] = collector
        return collector

    def get_collector(self, key: str) -> Optional[MessageCollector]:
        """Get a message collector by key."""
        return self._collectors.get(key)

    def clear_messages(self, key: str) -> None:
        """Clear messages from a collector."""
        collector = self._collectors.get(key)
        if collector:
            collector.clear()

    # -----------------------------------------------------------------
    # Properties
    # -----------------------------------------------------------------

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

    # -----------------------------------------------------------------
    # Simulation convenience methods (no-ops when sim not configured)
    # -----------------------------------------------------------------

    @property
    def simulation_manager(self):
        """Get the singleton SimulationManager (or None)."""
        try:
            from sim_harness.simulator.simulation_manager import (
                get_simulation_manager,
            )
            return get_simulation_manager()
        except ImportError:
            return None

    @property
    def current_simulation(self):
        """Get the current simulation configuration (or None)."""
        mgr = self.simulation_manager
        return mgr.current_config if mgr else None

    def restart_simulation(self) -> bool:
        """
        Force restart the current simulation.

        Returns:
            True if restart succeeded, False if not configured
        """
        mgr = self.simulation_manager
        if mgr is None:
            return False
        return mgr.restart(
            startup_timeout=self.STARTUP_TIMEOUT,
            gazebo_delay=self.GAZEBO_STARTUP_DELAY,
        )

    def wait_for_simulation(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait for simulation to be ready.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if simulation is ready, False if timeout or not configured
        """
        try:
            from sim_harness.simulator.gazebo_backend import GazeboBackend
            return GazeboBackend().wait_until_ready(timeout_sec)
        except ImportError:
            return False


# Backwards compatibility alias
SimulationTestFixture = SimTestFixture


# -----------------------------------------------------------------
# Standalone pytest fixtures
# -----------------------------------------------------------------

@pytest.fixture
def ros_node():
    """
    Pytest fixture providing a ROS 2 node.

    Usage::

        def test_something(ros_node):
            pub = ros_node.create_publisher(String, '/topic', 10)
    """
    config = get_test_isolation_config()
    apply_test_isolation(config)

    if not SimTestFixture._rclpy_initialized:
        rclpy.init()
        SimTestFixture._rclpy_initialized = True

    node_name = generate_test_node_name("pytest_node", config.domain_id)
    node = rclpy.create_node(
        node_name,
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ],
    )

    yield node
    node.destroy_node()


@pytest.fixture
def ros_executor(ros_node):
    """
    Pytest fixture providing a ROS 2 executor with node already added.

    Usage::

        def test_something(ros_executor, ros_node):
            from sim_harness.core.spin_helpers import spin_for_duration
            spin_for_duration(ros_executor, 1.0)
    """
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    yield executor
    executor.remove_node(ros_node)
