# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test fixture for ROS 2 simulation tests.

Uses a MultiThreadedExecutor spinning in a background thread so that ROS 2
action client callbacks are processed concurrently with test logic.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.checks import check_lidar_valid

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = check_lidar_valid(self.node, '/scan')
            assert result.ok, result.details
"""

import os
import random
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter

from sim_harness.collector import MessageCollector
from sim_harness.spin import spin_for_duration, spin_until_condition


def _ensure_rclpy_init():
    """Initialize rclpy if not already initialized, handling stale contexts."""
    if not SimTestFixture._rclpy_initialized:
        rclpy.init()
        SimTestFixture._rclpy_initialized = True
    else:
        try:
            if not rclpy.get_default_context().ok():
                rclpy.init()
        except Exception:
            rclpy.init()


class SimTestFixture:
    """Base class for ROS 2 simulation integration tests.

    Provides a ROS node, executor, message collection, and spin helpers.
    Set ``LAUNCH_PACKAGE`` / ``LAUNCH_FILE`` to manage a Gazebo simulation.

    The executor runs in a background thread so action-client callbacks are
    processed without requiring explicit spin calls.

    Declarative readiness checks
    ----------------------------
    Override these class variables to gate test execution on simulation
    readiness.  Checks run **once per class** (not per test) after the
    simulation is confirmed running.

    ``READINESS_LIFECYCLE_NODES``
        Lifecycle node names that must reach *Active*, e.g.
        ``['ns/planner_server', 'ns/controller_server']``.
    ``READINESS_TOPICS``
        ``(topic, msg_type, expected_rate_hz)`` tuples, e.g.
        ``[('/odom', Odometry, 10.0)]``.
    ``READINESS_TF_FRAMES``
        ``(target_frame, source_frame)`` pairs, e.g.
        ``[('odom', 'base_link')]``.
    ``READINESS_TIMEOUT``
        Per-check timeout in seconds (default 120).
    ``READINESS_MAX_ATTEMPTS``
        Retry count per check (default 3).

    For custom one-time readiness logic, override :meth:`on_sim_ready`.
    """

    _rclpy_initialized: bool = False
    _readiness_completed: set = set()

    # Simulation management (optional — leave empty if not needed)
    LAUNCH_PACKAGE: str = ""
    LAUNCH_FILE: str = ""
    LAUNCH_ARGS: dict = {}
    WORLD: str = ""
    ROBOT_MODEL: str = ""
    STARTUP_TIMEOUT: float = 60.0
    GAZEBO_STARTUP_DELAY: float = 5.0
    REQUIRE_SIM: bool = True
    USE_EXISTING_SIM: bool = False

    # Declarative readiness checks (override in subclasses)
    READINESS_LIFECYCLE_NODES: list = []
    READINESS_TOPICS: list = []
    READINESS_TF_FRAMES: list = []
    READINESS_TIMEOUT: float = 120.0
    READINESS_MAX_ATTEMPTS: int = 3

    @pytest.fixture(autouse=True)
    def setup_ros(self):
        """Autouse fixture: simulation (if configured) -> readiness -> node -> on_setup."""
        sim_managed = self._setup_simulation()

        # Run declarative readiness checks once per class
        cls = self.__class__
        if id(cls) not in SimTestFixture._readiness_completed and sim_managed:
            has_checks = (
                cls.READINESS_LIFECYCLE_NODES
                or cls.READINESS_TOPICS
                or cls.READINESS_TF_FRAMES
            )
            if has_checks:
                self._run_readiness_checks()
            SimTestFixture._readiness_completed.add(id(cls))

        self._setup_node(skip_domain_randomization=sim_managed)
        try:
            # on_setup runs BEFORE the spin thread starts, so action client
            # creation doesn't race with the executor's wait-set reads.
            self.on_setup()
        except BaseException:
            self.on_teardown()
            self._teardown_node()
            if sim_managed:
                self._release_simulation()
            raise
        # Start background executor AFTER on_setup so all subscriptions and
        # action clients are registered before the first rcl_wait() call.
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        yield
        self.on_teardown()
        self._teardown_node()
        if sim_managed:
            self._release_simulation()

    def on_setup(self):
        """Override for custom setup after node creation."""

    def on_teardown(self):
        """Override for custom teardown before node destruction."""

    def on_sim_ready(self):
        """Override for custom one-time readiness checks.

        Called once per class after all declarative ``READINESS_*`` checks
        pass.  A temporary ROS node is available as ``self._node`` during
        this call (it is destroyed afterwards).
        """

    # -- Readiness gate -----------------------------------------------------

    def _run_readiness_checks(self):
        """Run declarative readiness checks once per class.

        Creates a bare temporary node (not attached to any executor) so that
        ``_acquire_managed()`` in the check functions can add it to their own
        SingleThreadedExecutor for DDS discovery.
        """
        cls = self.__class__

        _ensure_rclpy_init()

        domain = int(os.environ.get('ROS_DOMAIN_ID', 0))
        temp_node = rclpy.create_node(
            f"readiness_{domain}_{random.randint(0, 9999):04d}",
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )

        try:
            # Lifecycle checks
            if cls.READINESS_LIFECYCLE_NODES:
                from sim_harness.nav2 import check_lifecycle_node_active
                for name in cls.READINESS_LIFECYCLE_NODES:
                    result = check_lifecycle_node_active(
                        temp_node, name,
                        timeout_sec=cls.READINESS_TIMEOUT,
                        max_attempts=cls.READINESS_MAX_ATTEMPTS,
                    )
                    if not result.ok:
                        pytest.skip(
                            f"Readiness: lifecycle node {name} not active "
                            f"after {cls.READINESS_MAX_ATTEMPTS} attempts: "
                            f"{result.details}")
                print(f"[Readiness] {len(cls.READINESS_LIFECYCLE_NODES)} "
                      f"lifecycle node(s) active", flush=True)

            # Topic checks
            if cls.READINESS_TOPICS:
                from sim_harness.checks import check_sensor_publishing
                for entry in cls.READINESS_TOPICS:
                    topic, msg_type, rate_hz = entry
                    result = check_sensor_publishing(
                        temp_node, topic,
                        expected_rate_hz=rate_hz,
                        msg_type=msg_type,
                        tolerance_percent=90.0,
                        sample_duration_sec=5.0,
                        max_attempts=cls.READINESS_MAX_ATTEMPTS,
                    )
                    if not result.ok:
                        pytest.skip(
                            f"Readiness: topic {topic} not publishing "
                            f"after {cls.READINESS_MAX_ATTEMPTS} attempts: "
                            f"{result.details}")
                print(f"[Readiness] {len(cls.READINESS_TOPICS)} "
                      f"topic(s) publishing", flush=True)

            # TF checks
            if cls.READINESS_TF_FRAMES:
                from sim_harness.checks import check_transform_available
                for target, source in cls.READINESS_TF_FRAMES:
                    ok = check_transform_available(
                        temp_node, target, source,
                        timeout_sec=min(cls.READINESS_TIMEOUT, 30.0),
                    )
                    if not ok:
                        pytest.skip(
                            f"Readiness: TF {target} -> {source} not available")
                print(f"[Readiness] {len(cls.READINESS_TF_FRAMES)} "
                      f"TF frame(s) available", flush=True)

            # Custom hook
            self._node = temp_node
            try:
                self.on_sim_ready()
            finally:
                self._node = None  # type: ignore[assignment]

        finally:
            temp_node.destroy_node()

    # -- Node lifecycle -----------------------------------------------------

    def _setup_node(self, skip_domain_randomization: bool = False):
        if skip_domain_randomization:
            domain = int(os.environ.get('ROS_DOMAIN_ID', 0))
        else:
            domain = random.randint(100, 199)
            os.environ['ROS_DOMAIN_ID'] = str(domain)

        _ensure_rclpy_init()

        self._node = rclpy.create_node(
            f"sim_test_{domain}_{random.randint(0, 9999):04d}",
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._collectors = {}
        self._active_goal_handles = []
        self._goal_handles_lock = threading.Lock()
        # Spin thread is started in setup_ros after on_setup() completes

    def _teardown_node(self):
        # Cancel any active action goals (executor must still be running)
        with self._goal_handles_lock:
            handles = list(self._active_goal_handles)
            self._active_goal_handles.clear()
        for gh in handles:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                print(f"[SimTestFixture] Warning: failed to cancel goal: {e}")
        if handles:
            time.sleep(0.5)

        # Stop background spin thread FIRST — no more wait-set reads after this
        if hasattr(self, '_executor'):
            self._executor.shutdown()
        if hasattr(self, '_spin_thread'):
            self._spin_thread.join(timeout=2.0)

        # Now safe to destroy subscriptions and collectors (wait set is idle)
        for name, c in self._collectors.items():
            try:
                c.destroy()
            except Exception as e:
                print(f"[SimTestFixture] Warning: failed to destroy collector "
                      f"'{name}': {e}")
        self._collectors.clear()

        # Clean up node
        if hasattr(self, '_executor') and hasattr(self, '_node'):
            try:
                self._executor.remove_node(self._node)
            except Exception as e:
                print(f"[SimTestFixture] Warning: failed to remove node from "
                      f"executor: {e}")
            try:
                self._node.destroy_node()
            except Exception as e:
                print(f"[SimTestFixture] Warning: failed to destroy node: {e}")

    @classmethod
    def teardown_class(cls):
        if SimTestFixture._rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"[SimTestFixture] Warning: rclpy.shutdown() failed: {e}")
            SimTestFixture._rclpy_initialized = False

    # -- Simulation lifecycle (lazy imports) --------------------------------

    def _setup_simulation(self) -> bool:
        cls = self.__class__
        if cls.USE_EXISTING_SIM:
            try:
                from sim_harness.simulator.gazebo_backend import GazeboBackend
                if not GazeboBackend().is_running() and cls.REQUIRE_SIM:
                    pytest.skip("Simulation not running and USE_EXISTING_SIM=True")
            except ImportError:
                if cls.REQUIRE_SIM:
                    pytest.skip("Gazebo backend not available")
            return False

        if not cls.LAUNCH_PACKAGE or not cls.LAUNCH_FILE:
            return False

        try:
            from sim_harness.simulator.simulation_manager import (
                SimulationRequest, get_simulation_manager,
            )
        except ImportError:
            if cls.REQUIRE_SIM:
                pytest.skip("sim_harness.simulator not available")
            return False

        manager = get_simulation_manager()
        req = SimulationRequest(
            package=cls.LAUNCH_PACKAGE, launch_file=cls.LAUNCH_FILE,
            launch_args=cls.LAUNCH_ARGS, world=cls.WORLD,
            robot_model=cls.ROBOT_MODEL,
        )
        try:
            ok = manager.request(
                req, startup_timeout=cls.STARTUP_TIMEOUT,
                gazebo_delay=cls.GAZEBO_STARTUP_DELAY,
                require_sim=cls.REQUIRE_SIM,
            )
            if not ok and cls.REQUIRE_SIM:
                pytest.skip(f"Failed to start sim: {cls.LAUNCH_PACKAGE}/{cls.LAUNCH_FILE}")
                return False
        except RuntimeError as e:
            if cls.REQUIRE_SIM:
                pytest.skip(str(e))
            return False
        return True

    def _release_simulation(self):
        try:
            from sim_harness.simulator.simulation_manager import get_simulation_manager
            get_simulation_manager().release()
        except ImportError:
            pass

    # -- Properties ---------------------------------------------------------

    @property
    def node(self):
        return self._node

    @property
    def executor(self):
        return self._executor

    def get_logger(self):
        return self._node.get_logger()

    # -- Spin helpers -------------------------------------------------------

    def spin_for_duration(self, duration_sec: float):
        """Wait for *duration_sec* while background executor processes callbacks."""
        time.sleep(duration_sec)

    def spin_until_condition(self, condition, timeout_sec: float) -> bool:
        """Poll *condition()* until True or timeout."""
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.05)
        return False

    # -- Goal handle tracking -----------------------------------------------

    def register_goal_handle(self, goal_handle):
        """Register an action goal handle for automatic cancellation on teardown.

        Call this after a successful ``send_goal_async`` to ensure the goal
        is canceled when the test finishes, preventing stale controller state
        in subsequent tests.
        """
        with self._goal_handles_lock:
            self._active_goal_handles.append(goal_handle)

    def unregister_goal_handle(self, goal_handle):
        """Remove a goal handle from tracking (e.g. after it completes)."""
        with self._goal_handles_lock:
            try:
                self._active_goal_handles.remove(goal_handle)
            except ValueError:
                pass

    # -- Message collectors -------------------------------------------------

    def create_message_collector(self, topic, msg_type, key=None, **kwargs):
        """Create a managed message collector for a topic."""
        c = MessageCollector(self._node, topic, msg_type, **kwargs)
        self._collectors[key or topic] = c
        return c

    def get_collector(self, key):
        return self._collectors.get(key)

    def clear_messages(self, key):
        c = self._collectors.get(key)
        if c:
            c.clear()


SimulationTestFixture = SimTestFixture


# -- Standalone pytest fixtures ---------------------------------------------

@pytest.fixture
def ros_node():
    """Pytest fixture providing a plain ROS 2 node."""
    _ensure_rclpy_init()
    d = random.randint(100, 199)
    os.environ['ROS_DOMAIN_ID'] = str(d)
    node = rclpy.create_node(
        f"pytest_{d}_{random.randint(0, 9999):04d}",
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    yield node
    node.destroy_node()


@pytest.fixture
def ros_executor(ros_node):
    """Pytest fixture providing executor with node attached."""
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    yield executor
    executor.remove_node(ros_node)
