# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Simulation test fixture with automatic simulation lifecycle management.

Uses the singleton SimulationManager to coordinate simulations across tests.
"""

import pytest
from typing import Optional

from sim_harness.core.test_fixture import SimTestFixture
from sim_harness.core.test_isolation import get_test_isolation_config
from sim_harness.simulator.simulation_manager import (
    SimulationManager,
    SimulationRequest,
    get_simulation_manager,
)
from sim_harness.simulator.gazebo_backend import GazeboBackend


class SimulationTestFixture(SimTestFixture):
    """
    Test fixture that automatically manages simulation lifecycle.

    Uses the singleton SimulationManager to:
    - Start simulation if not running
    - Reuse simulation if config matches
    - Restart simulation if config differs substantially

    Configure via class attributes:
        class TestMyRobot(SimulationTestFixture):
            # Required: simulation launch config
            LAUNCH_PACKAGE = 'turtlebot3_gazebo'
            LAUNCH_FILE = 'turtlebot3_world.launch.py'

            # Optional: used to determine if restart needed
            WORLD = 'turtlebot3_world'
            ROBOT_MODEL = 'waffle'

            # Optional settings
            LAUNCH_ARGS = {'use_sim_time': 'true'}
            STARTUP_TIMEOUT = 60.0
            GAZEBO_STARTUP_DELAY = 5.0  # Extra wait after Gazebo starts
            REQUIRE_SIM = True  # Skip tests if sim can't start

            def test_robot_moves(self):
                # Simulation is already running
                ...

    Or use the existing simulation (don't manage lifecycle):
        class TestMyRobot(SimulationTestFixture):
            USE_EXISTING_SIM = True  # Don't start, assume running

            def test_something(self):
                ...

    Simulations are automatically reused when configs match:
        # These two test classes will share the same simulation
        class TestNavigation(SimulationTestFixture):
            LAUNCH_PACKAGE = 'turtlebot3_gazebo'
            LAUNCH_FILE = 'turtlebot3_world.launch.py'
            WORLD = 'turtlebot3_world'

        class TestSensors(SimulationTestFixture):
            LAUNCH_PACKAGE = 'turtlebot3_gazebo'
            LAUNCH_FILE = 'turtlebot3_world.launch.py'
            WORLD = 'turtlebot3_world'  # Same world = reuse sim

    Simulations restart when configs differ substantially:
        # This will trigger a simulation restart
        class TestDifferentWorld(SimulationTestFixture):
            LAUNCH_PACKAGE = 'turtlebot3_gazebo'
            LAUNCH_FILE = 'turtlebot3_house.launch.py'
            WORLD = 'turtlebot3_house'  # Different world = restart
    """

    # Override these in subclasses
    LAUNCH_PACKAGE: str = ""
    LAUNCH_FILE: str = ""
    LAUNCH_ARGS: dict = {}
    WORLD: str = ""  # Used to determine if restart needed
    ROBOT_MODEL: str = ""  # Used to determine if restart needed
    STARTUP_TIMEOUT: float = 60.0
    GAZEBO_STARTUP_DELAY: float = 5.0
    REQUIRE_SIM: bool = True  # Skip tests if sim can't start
    USE_EXISTING_SIM: bool = False  # If True, don't manage sim lifecycle

    @pytest.fixture(autouse=True)
    def setup_simulation(self, request):
        """
        Pytest fixture for simulation setup/teardown.

        Uses the singleton SimulationManager to coordinate simulations.
        """
        cls = self.__class__
        manager = get_simulation_manager()
        gazebo = GazeboBackend()

        if cls.USE_EXISTING_SIM:
            # Use existing simulation, don't manage lifecycle
            if not gazebo.is_running():
                if cls.REQUIRE_SIM:
                    pytest.skip("Simulation not running and USE_EXISTING_SIM=True")
            yield
            return

        # Build simulation request
        if cls.LAUNCH_PACKAGE and cls.LAUNCH_FILE:
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
            except RuntimeError as e:
                if cls.REQUIRE_SIM:
                    pytest.skip(str(e))
                yield
                return

            yield

            # Release (but don't stop - allow reuse)
            manager.release()

        elif not gazebo.is_running() and cls.REQUIRE_SIM:
            pytest.skip("No simulation configured and none running")
            yield
        else:
            yield

    @pytest.fixture(autouse=True)
    def setup_ros(self, setup_simulation) -> None:
        """
        Override setup_ros to depend on setup_simulation.

        This ensures simulation is started before ROS node is created.
        The setup_simulation parameter creates a pytest fixture dependency,
        guaranteeing the simulation fixture runs first.
        """
        # Duplicate parent's setup/teardown logic (can't call fixture directly)
        self._setup_test()
        try:
            self.on_setup()
        except BaseException:
            # Cleanup if on_setup fails (including pytest.skip)
            # Without this, the node is never destroyed, corrupting rclpy context
            self.on_teardown()
            self._teardown_test()
            raise
        yield
        self.on_teardown()
        self._teardown_test()

    @property
    def simulation_manager(self) -> SimulationManager:
        """Get the singleton SimulationManager."""
        return get_simulation_manager()

    @property
    def current_simulation(self) -> Optional[SimulationRequest]:
        """Get the current simulation configuration."""
        return get_simulation_manager().current_config

    def restart_simulation(self) -> bool:
        """
        Force restart the current simulation.

        Useful when a test needs a fresh simulation state.

        Returns:
            True if restart succeeded
        """
        return get_simulation_manager().restart(
            startup_timeout=self.STARTUP_TIMEOUT,
            gazebo_delay=self.GAZEBO_STARTUP_DELAY,
        )

    def wait_for_simulation(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait for simulation to be ready.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if simulation is ready
        """
        gazebo = GazeboBackend()
        return gazebo.wait_until_ready(timeout_sec)


# Pytest fixtures for different scopes

@pytest.fixture(scope="module")
def simulation_manager():
    """
    Module-scoped fixture providing the SimulationManager.

    Usage:
        def test_with_sim(simulation_manager):
            simulation_manager.request(SimulationRequest(
                package='my_pkg',
                launch_file='sim.launch.py'
            ))
            # Run tests
            simulation_manager.release()
    """
    manager = get_simulation_manager()
    yield manager
    # Don't stop - let other modules reuse


@pytest.fixture(scope="session")
def session_simulation():
    """
    Session-scoped fixture for the SimulationManager.

    Stops simulation at end of test session.

    Usage:
        @pytest.fixture(scope="session", autouse=True)
        def start_simulation(session_simulation):
            session_simulation.request(SimulationRequest(...))
            yield
            # Stops automatically at session end
    """
    manager = get_simulation_manager()
    yield manager
    manager.stop(force=True)
