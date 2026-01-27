# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Singleton simulation manager for coordinating Gazebo lifecycle.

Ensures only one Gazebo simulation runs at a time and handles
restarting when test configurations differ substantially.
"""

import atexit
import hashlib
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

from sim_harness.simulator.gazebo_backend import GazeboBackend
from sim_harness.simulator.simulation_launcher import (
    SimulationLauncher,
    LaunchConfig,
)
from sim_harness.core.test_isolation import (
    TestIsolationConfig,
    apply_test_isolation,
    get_test_isolation_config,
)


@dataclass
class SimulationRequest:
    """Request for a simulation configuration."""

    package: str
    """ROS 2 package containing the launch file."""

    launch_file: str
    """Launch file name."""

    launch_args: Dict[str, str] = field(default_factory=dict)
    """Launch arguments."""

    world: str = ""
    """World file or name (used to determine if restart needed)."""

    robot_model: str = ""
    """Robot model (used to determine if restart needed)."""

    def config_hash(self) -> str:
        """
        Generate a hash of the configuration.

        Used to determine if two configs are substantially different.
        """
        key_parts = [
            self.package,
            self.launch_file,
            self.world,
            self.robot_model,
            # Include sorted launch args that affect simulation
            *sorted(f"{k}={v}" for k, v in self.launch_args.items()
                    if k in _RESTART_TRIGGERING_ARGS),
        ]
        key = "|".join(key_parts)
        return hashlib.md5(key.encode()).hexdigest()[:12]

    def to_launch_config(
        self,
        env_vars: Optional[Dict[str, str]] = None,
        startup_timeout: float = 60.0,
        gazebo_delay: float = 5.0,
    ) -> LaunchConfig:
        """Convert to LaunchConfig for the launcher."""
        return LaunchConfig(
            package=self.package,
            launch_file=self.launch_file,
            launch_args=self.launch_args,
            env_vars=env_vars or {},
            startup_timeout_sec=startup_timeout,
            gazebo_startup_delay_sec=gazebo_delay,
        )


# Launch args that trigger a restart when changed
_RESTART_TRIGGERING_ARGS: Set[str] = {
    "world",
    "world_name",
    "robot_model",
    "model",
    "vehicle",
    "headless",
    "gui",
}


class SimulationManager:
    """
    Singleton manager for simulation lifecycle.

    Coordinates Gazebo simulation across all tests, ensuring:

    - Only one simulation runs at a time
    - Simulation is reused when configs are compatible
    - Simulation is restarted when configs differ substantially
    - Proper cleanup on exit

    Example::

        manager = SimulationManager.get_instance()

        # Request a simulation - starts or reuses existing
        manager.request(SimulationRequest(
            package='turtlebot3_gazebo',
            launch_file='turtlebot3_world.launch.py',
            world='turtlebot3_world',
        ))

        # Run tests...

        # Release when done (doesn't stop, allows reuse)
        manager.release()

        # Force stop if needed
        manager.stop()
    """

    _instance: Optional['SimulationManager'] = None
    _lock = threading.Lock()

    def __init__(self):
        """Private constructor - use get_instance()."""
        self._launcher: Optional[SimulationLauncher] = None
        self._current_request: Optional[SimulationRequest] = None
        self._current_hash: str = ""
        self._gazebo = GazeboBackend()
        self._isolation_config: Optional[TestIsolationConfig] = None
        self._started_by_us: bool = False
        self._active_users: int = 0  # Track how many tests are using the sim

    @classmethod
    def get_instance(cls) -> 'SimulationManager':
        """
        Get the singleton instance.

        Thread-safe singleton accessor.
        """
        if cls._instance is None:
            with cls._lock:
                # Double-check after acquiring lock
                if cls._instance is None:
                    cls._instance = cls()
                    # Register cleanup on exit
                    atexit.register(cls._cleanup_on_exit)
        return cls._instance

    @classmethod
    def _cleanup_on_exit(cls) -> None:
        """Cleanup handler called on program exit."""
        if cls._instance is not None:
            cls._instance.stop(force=True)

    def request(
        self,
        request: SimulationRequest,
        startup_timeout: float = 60.0,
        gazebo_delay: float = 5.0,
        require_sim: bool = True,
    ) -> bool:
        """
        Request a simulation with the given configuration.

        If a compatible simulation is already running, reuses it.
        If a different simulation is running, restarts with new config.
        If no simulation is running, starts a new one.

        Args:
            request: Simulation configuration request
            startup_timeout: Max time to wait for startup
            gazebo_delay: Additional delay after Gazebo detected
            require_sim: If True, raises if sim can't start

        Returns:
            True if simulation is ready

        Raises:
            RuntimeError: If require_sim=True and simulation can't start
        """
        with self._lock:
            request_hash = request.config_hash()

            # Check if we can reuse current simulation
            if self._can_reuse(request_hash):
                self._active_users += 1
                return True

            # Need to (re)start simulation
            if self._launcher is not None and self._started_by_us:
                self._stop_internal()

            # Apply test isolation
            self._isolation_config = get_test_isolation_config()
            apply_test_isolation(self._isolation_config)

            # Check if a functional simulation is already running externally
            # Use is_responsive() not is_running() to detect zombie processes
            if self._gazebo.is_responsive():
                # External simulation running and responsive - try to use it
                self._current_request = request
                self._current_hash = request_hash
                self._started_by_us = False
                self._active_users += 1
                return True

            # If processes exist but aren't responsive, kill them (zombie cleanup)
            if self._gazebo.is_running():
                self._gazebo.kill_gazebo()
                time.sleep(1.0)  # Brief wait for processes to terminate

            # Start new simulation
            success = self._start_internal(
                request, startup_timeout, gazebo_delay
            )

            if success:
                self._current_request = request
                self._current_hash = request_hash
                self._started_by_us = True
                self._active_users += 1
                return True

            if require_sim:
                raise RuntimeError(
                    f"Failed to start simulation: {request.package}/{request.launch_file}"
                )
            return False

    def release(self) -> None:
        """
        Release the simulation (decrement user count).

        Does NOT stop the simulation - allows other tests to reuse it.
        Call stop() to actually stop the simulation.
        """
        with self._lock:
            if self._active_users > 0:
                self._active_users -= 1

    def stop(self, force: bool = False) -> None:
        """
        Stop the simulation.

        Args:
            force: If True, stop even if other users are active
        """
        with self._lock:
            if not force and self._active_users > 0:
                return

            self._stop_internal()
            self._active_users = 0

    def restart(
        self,
        request: Optional[SimulationRequest] = None,
        startup_timeout: float = 60.0,
        gazebo_delay: float = 5.0,
    ) -> bool:
        """
        Force restart the simulation.

        Args:
            request: New config (uses current if None)
            startup_timeout: Max time to wait for startup
            gazebo_delay: Additional delay after Gazebo detected

        Returns:
            True if simulation restarted successfully
        """
        with self._lock:
            if request is None:
                request = self._current_request

            if request is None:
                return False

            self._stop_internal()

            success = self._start_internal(request, startup_timeout, gazebo_delay)
            if success:
                self._current_request = request
                self._current_hash = request.config_hash()
                self._started_by_us = True
            return success

    def is_running(self) -> bool:
        """Check if simulation is currently running."""
        return self._gazebo.is_running()

    @property
    def current_config(self) -> Optional[SimulationRequest]:
        """Get the current simulation configuration."""
        return self._current_request

    @property
    def active_users(self) -> int:
        """Get count of active users."""
        return self._active_users

    def _can_reuse(self, request_hash: str) -> bool:
        """Check if current simulation can be reused for request."""
        if not self._gazebo.is_running():
            return False
        if self._current_hash == "":
            return False
        return self._current_hash == request_hash

    def _start_internal(
        self,
        request: SimulationRequest,
        startup_timeout: float,
        gazebo_delay: float,
    ) -> bool:
        """Internal method to start simulation (must hold lock)."""
        if self._launcher is None:
            self._launcher = SimulationLauncher()

        # Build env vars with isolation
        env_vars: Dict[str, str] = {}
        if self._isolation_config:
            env_vars['ROS_DOMAIN_ID'] = str(self._isolation_config.domain_id)
            env_vars['GZ_PARTITION'] = self._isolation_config.gz_partition

        config = request.to_launch_config(
            env_vars=env_vars,
            startup_timeout=startup_timeout,
            gazebo_delay=gazebo_delay,
        )

        return self._launcher.start(config, wait_for_ready=True)

    def _stop_internal(self) -> None:
        """Internal method to stop simulation (must hold lock)."""
        if self._launcher is not None:
            self._launcher.stop()

        # Also kill any orphaned Gazebo processes
        self._gazebo.kill_gazebo()

        self._current_request = None
        self._current_hash = ""
        self._started_by_us = False


# Convenience function
def get_simulation_manager() -> SimulationManager:
    """Get the singleton SimulationManager instance."""
    return SimulationManager.get_instance()
