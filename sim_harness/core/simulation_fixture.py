# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Simulation test fixture with automatic simulation lifecycle management.

Extends SimTestFixture to automatically start and stop simulations.
"""

import pytest
from typing import Optional

from sim_harness.core.test_fixture import SimTestFixture
from sim_harness.core.test_isolation import (
    get_test_isolation_config,
    apply_test_isolation,
)
from sim_harness.simulator.simulation_launcher import (
    SimulationLauncher,
    LaunchConfig,
)
from sim_harness.simulator.gazebo_backend import GazeboBackend


class SimulationTestFixture(SimTestFixture):
    """
    Test fixture that automatically manages simulation lifecycle.

    Extends SimTestFixture to start the simulation before tests
    and stop it after tests complete.

    Configure via class attributes:
        class TestMyRobot(SimulationTestFixture):
            # Required: simulation launch config
            LAUNCH_PACKAGE = 'turtlebot3_gazebo'
            LAUNCH_FILE = 'turtlebot3_world.launch.py'

            # Optional settings
            LAUNCH_ARGS = {'use_sim_time': 'true'}
            STARTUP_TIMEOUT = 60.0
            REQUIRE_SIM = True  # Skip tests if sim can't start

            def test_robot_moves(self):
                # Simulation is already running
                ...

    Or use the existing simulation (don't start/stop):
        class TestMyRobot(SimulationTestFixture):
            USE_EXISTING_SIM = True  # Don't start, assume running

            def test_something(self):
                ...
    """

    # Override these in subclasses
    LAUNCH_PACKAGE: str = ""
    LAUNCH_FILE: str = ""
    LAUNCH_ARGS: dict = {}
    ENV_VARS: dict = {}
    STARTUP_TIMEOUT: float = 60.0
    GAZEBO_STARTUP_DELAY: float = 5.0
    REQUIRE_SIM: bool = True  # Skip tests if sim can't start
    USE_EXISTING_SIM: bool = False  # If True, don't start/stop sim

    # Class-level launcher (shared across tests in class)
    _launcher: Optional[SimulationLauncher] = None
    _sim_started_by_us: bool = False

    @pytest.fixture(autouse=True)
    def setup_simulation(self, request):
        """
        Pytest fixture for simulation setup/teardown.

        Runs before setup_ros to ensure simulation is ready.
        Note: This must run before setup_ros, which is achieved by
        having setup_ros depend on this fixture.
        """
        cls = self.__class__

        # Apply test isolation FIRST so simulation uses same domain ID
        isolation_config = get_test_isolation_config()
        apply_test_isolation(isolation_config)

        # Check if simulation is already running
        gazebo = GazeboBackend()
        sim_already_running = gazebo.is_running()

        if cls.USE_EXISTING_SIM:
            # Use existing simulation, don't manage lifecycle
            if not sim_already_running:
                if cls.REQUIRE_SIM:
                    pytest.skip("Simulation not running and USE_EXISTING_SIM=True")
            yield
            return

        # Start simulation if not already running
        if not sim_already_running and cls.LAUNCH_PACKAGE and cls.LAUNCH_FILE:
            if cls._launcher is None:
                cls._launcher = SimulationLauncher()

            if not cls._launcher.is_running():
                # Merge env vars with isolation settings
                env_vars = dict(cls.ENV_VARS)
                env_vars['ROS_DOMAIN_ID'] = str(isolation_config.domain_id)
                if isolation_config.gz_partition:
                    env_vars['GZ_PARTITION'] = isolation_config.gz_partition

                config = LaunchConfig(
                    package=cls.LAUNCH_PACKAGE,
                    launch_file=cls.LAUNCH_FILE,
                    launch_args=cls.LAUNCH_ARGS,
                    env_vars=env_vars,
                    startup_timeout_sec=cls.STARTUP_TIMEOUT,
                    gazebo_startup_delay_sec=cls.GAZEBO_STARTUP_DELAY,
                )

                started = cls._launcher.start(config, wait_for_ready=True)
                if started:
                    cls._sim_started_by_us = True
                elif cls.REQUIRE_SIM:
                    pytest.skip(f"Failed to start simulation: {cls.LAUNCH_PACKAGE}/{cls.LAUNCH_FILE}")

        elif not sim_already_running and cls.REQUIRE_SIM and not cls.USE_EXISTING_SIM:
            # No launch config and sim not running
            if cls.LAUNCH_PACKAGE:
                pytest.skip(f"Simulation not running: {cls.LAUNCH_PACKAGE}/{cls.LAUNCH_FILE}")

        yield

        # Teardown is handled at class level

    @pytest.fixture(autouse=True)
    def setup_ros(self, setup_simulation) -> None:
        """
        Override setup_ros to depend on setup_simulation.

        This ensures simulation is started before ROS node is created.
        """
        # Call parent's setup logic
        self._setup_test()
        yield
        self._teardown_test()

    @classmethod
    def teardown_class(cls) -> None:
        """Stop simulation at end of test class."""
        # Stop simulation if we started it
        if cls._sim_started_by_us and cls._launcher is not None:
            cls._launcher.stop()
            cls._launcher = None
            cls._sim_started_by_us = False

        # Call parent teardown
        super().teardown_class()

    @property
    def simulation(self) -> Optional[SimulationLauncher]:
        """Get the simulation launcher instance."""
        return self.__class__._launcher

    def wait_for_simulation(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait for simulation to be ready.

        Useful if you need to ensure simulation is fully initialized.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if simulation is ready
        """
        gazebo = GazeboBackend()
        return gazebo.wait_until_ready(timeout_sec)


# Pytest fixture for module-scoped simulation
@pytest.fixture(scope="module")
def simulation_launcher():
    """
    Module-scoped fixture that provides a simulation launcher.

    Usage:
        def test_with_sim(simulation_launcher):
            simulation_launcher.start(LaunchConfig(
                package='my_pkg',
                launch_file='sim.launch.py'
            ))
            # Tests run with simulation
            # Automatically stops at end of module
    """
    launcher = SimulationLauncher()
    yield launcher
    launcher.stop()


# Pytest fixture for session-scoped simulation (one sim for all tests)
@pytest.fixture(scope="session")
def session_simulation():
    """
    Session-scoped fixture for simulations that should persist across all tests.

    Usage:
        @pytest.fixture(scope="session", autouse=True)
        def start_simulation(session_simulation):
            session_simulation.start(LaunchConfig(...))
            yield
            # Stops automatically
    """
    launcher = SimulationLauncher()
    yield launcher
    launcher.stop()
