# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test fixture for ROS 2 simulation tests.

Example::

    from sim_harness import SimTestFixture, assert_lidar_valid

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = assert_lidar_valid(self.node, '/scan')
            assert result.valid, result.details
"""

import os
import random

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from sim_harness.collector import MessageCollector
from sim_harness.spin import spin_for_duration, spin_until_condition


class SimTestFixture:
    """Base class for ROS 2 simulation integration tests.

    Provides a ROS node, executor, message collection, and spin helpers.
    Set ``LAUNCH_PACKAGE`` / ``LAUNCH_FILE`` to manage a Gazebo simulation.
    """

    _rclpy_initialized: bool = False

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

    @pytest.fixture(autouse=True)
    def setup_ros(self):
        """Autouse fixture: simulation (if configured) -> node -> on_setup."""
        sim_managed = self._setup_simulation()
        self._setup_node()
        try:
            self.on_setup()
        except BaseException:
            self.on_teardown()
            self._teardown_node()
            if sim_managed:
                self._release_simulation()
            raise
        yield
        self.on_teardown()
        self._teardown_node()
        if sim_managed:
            self._release_simulation()

    def on_setup(self):
        """Override for custom setup after node creation."""

    def on_teardown(self):
        """Override for custom teardown before node destruction."""

    # -- Node lifecycle -----------------------------------------------------

    def _setup_node(self):
        domain = random.randint(1, 230)
        os.environ['ROS_DOMAIN_ID'] = str(domain)

        if not SimTestFixture._rclpy_initialized:
            rclpy.init()
            SimTestFixture._rclpy_initialized = True
        else:
            try:
                if not rclpy.get_default_context().ok():
                    rclpy.init()
            except Exception:
                rclpy.init()

        self._node = rclpy.create_node(
            f"sim_test_{domain}_{random.randint(0, 9999):04d}",
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._collectors = {}

    def _teardown_node(self):
        for c in self._collectors.values():
            c.destroy()
        self._collectors.clear()
        if hasattr(self, '_executor') and hasattr(self, '_node'):
            self._executor.remove_node(self._node)
            self._node.destroy_node()

    @classmethod
    def teardown_class(cls):
        if SimTestFixture._rclpy_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass
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
        spin_for_duration(self._executor, duration_sec)

    def spin_until_condition(self, condition, timeout_sec: float) -> bool:
        return spin_until_condition(self._executor, condition, timeout_sec)

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
    if not SimTestFixture._rclpy_initialized:
        rclpy.init()
        SimTestFixture._rclpy_initialized = True
    d = random.randint(1, 230)
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
