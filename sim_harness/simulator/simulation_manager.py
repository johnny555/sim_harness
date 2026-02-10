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
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

from sim_harness.simulator.gazebo_backend import GazeboBackend
from sim_harness.simulator.simulation_launcher import (
    SimulationLauncher,
    LaunchConfig,
)
import random


def _discover_ros_domain_from_running_nodes() -> Optional[int]:
    """Read ROS_DOMAIN_ID from the environment of a running ROS 2 node process.

    Searches for known Nav2 processes (planner_server, controller_server) and
    reads their ``/proc/<pid>/environ`` to find the domain they launched on.

    Returns:
        The domain ID if found, or None.
    """
    for pattern in ("planner_server", "controller_server", "bt_navigator"):
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True, text=True, timeout=3,
            )
            for line in result.stdout.strip().split('\n'):
                if not line:
                    continue
                pid = int(line)
                try:
                    with open(f"/proc/{pid}/environ", "rb") as f:
                        env_bytes = f.read()
                    for entry in env_bytes.split(b'\x00'):
                        if entry.startswith(b'ROS_DOMAIN_ID='):
                            return int(entry.split(b'=', 1)[1])
                except (OSError, ValueError):
                    continue
        except Exception:
            continue
    return None


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
        self._isolation_domain_id: Optional[int] = None
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

            # Check if a functional simulation is already running externally
            # BEFORE changing domain ID — the external sim may be on a
            # specific domain and we need to adopt it, not override it.
            if self._gazebo.is_responsive():
                # External simulation running and responsive — adopt its
                # domain ID so the test node communicates on the same domain.
                ext_domain = _discover_ros_domain_from_running_nodes()
                if ext_domain is not None:
                    self._isolation_domain_id = ext_domain
                else:
                    # Fall back to current env value
                    self._isolation_domain_id = int(
                        os.environ.get('ROS_DOMAIN_ID', 0))
                os.environ['ROS_DOMAIN_ID'] = str(self._isolation_domain_id)
                print(f"[SimManager] Reusing responsive external simulation "
                      f"(domain {self._isolation_domain_id})", flush=True)
                self._current_request = request
                self._current_hash = request_hash
                self._started_by_us = False
                self._active_users += 1
                return True

            # If processes exist but aren't responsive, kill them (zombie cleanup)
            if self._gazebo.is_running():
                print("[SimManager] Killing unresponsive Gazebo processes",
                      flush=True)
                self._gazebo.kill_gazebo()
                time.sleep(1.0)  # Brief wait for processes to terminate

            # Apply test isolation (unique ROS_DOMAIN_ID) for the new sim
            self._isolation_domain_id = random.randint(100, 199)
            os.environ['ROS_DOMAIN_ID'] = str(self._isolation_domain_id)
            print(f"[SimManager] Domain ID set to {self._isolation_domain_id}",
                  flush=True)

            # Start new simulation
            print(f"[SimManager] Starting new simulation "
                  f"(timeout={startup_timeout}s, delay={gazebo_delay}s)",
                  flush=True)
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
        if not self._current_hash:
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
        if self._isolation_domain_id is not None:
            env_vars['ROS_DOMAIN_ID'] = str(self._isolation_domain_id)
            env_vars['GZ_PARTITION'] = f'gz_test_{self._isolation_domain_id}'

        config = request.to_launch_config(
            env_vars=env_vars,
            startup_timeout=startup_timeout,
            gazebo_delay=gazebo_delay,
        )

        success = self._launcher.start(config, wait_for_ready=True)
        if not success:
            self._launcher.stop()  # Clean up so next attempt doesn't hit "already running"
        return success

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
