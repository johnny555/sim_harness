# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Simulation launcher for starting and stopping ROS 2 simulations.

Provides utilities for launching simulations via ros2 launch and
managing their lifecycle during tests.
"""

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from sim_harness.simulator.gazebo_backend import GazeboBackend


@dataclass
class LaunchConfig:
    """Configuration for launching a simulation."""

    package: str
    """ROS 2 package containing the launch file."""

    launch_file: str
    """Launch file name (e.g., 'simulation.launch.py')."""

    launch_args: Dict[str, str] = field(default_factory=dict)
    """Launch arguments (key=value pairs)."""

    env_vars: Dict[str, str] = field(default_factory=dict)
    """Additional environment variables to set."""

    startup_timeout_sec: float = 60.0
    """Maximum time to wait for simulation startup."""

    gazebo_startup_delay_sec: float = 5.0
    """Additional delay after Gazebo processes detected."""


class SimulationLauncher:
    """
    Launches and manages ROS 2 simulations for testing.

    Handles:
    - Starting simulations via ros2 launch
    - Waiting for readiness
    - Clean shutdown
    - Process cleanup

    Example:
        launcher = SimulationLauncher()
        launcher.start(LaunchConfig(
            package='turtlebot3_gazebo',
            launch_file='turtlebot3_world.launch.py',
            launch_args={'use_sim_time': 'true'}
        ))

        # Run tests...

        launcher.stop()
    """

    def __init__(self):
        self._process: Optional[subprocess.Popen] = None
        self._gazebo = GazeboBackend()
        self._config: Optional[LaunchConfig] = None

    def start(
        self,
        config: LaunchConfig,
        wait_for_ready: bool = True
    ) -> bool:
        """
        Start a simulation.

        Args:
            config: Launch configuration
            wait_for_ready: Whether to wait for simulation to be ready

        Returns:
            True if simulation started successfully
        """
        if self._process is not None:
            raise RuntimeError("Simulation already running. Call stop() first.")

        self._config = config

        # Build launch command
        cmd = ["ros2", "launch", config.package, config.launch_file]
        for key, value in config.launch_args.items():
            cmd.append(f"{key}:={value}")

        # Build environment
        env = os.environ.copy()
        env.update(config.env_vars)

        # Start the process
        try:
            self._process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid  # Create new process group for cleanup
            )
        except Exception as e:
            print(f"Failed to start simulation: {e}")
            return False

        if wait_for_ready:
            return self.wait_until_ready(config.startup_timeout_sec)

        return True

    def wait_until_ready(self, timeout_sec: float = 60.0) -> bool:
        """
        Wait until the simulation is ready.

        Checks for Gazebo processes and waits for initialization.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if ready, False on timeout
        """
        if self._gazebo.wait_until_ready(timeout_sec):
            # Additional delay for full initialization
            if self._config:
                time.sleep(self._config.gazebo_startup_delay_sec)
            return True
        return False

    def stop(self, timeout_sec: float = 10.0) -> None:
        """
        Stop the simulation.

        Sends SIGINT first, then SIGKILL if needed.

        Args:
            timeout_sec: Maximum time to wait for graceful shutdown
        """
        if self._process is None:
            return

        # Try graceful shutdown first
        try:
            # Send SIGINT to process group
            os.killpg(os.getpgid(self._process.pid), signal.SIGINT)

            # Wait for process to exit
            try:
                self._process.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                # Force kill if still running
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                self._process.wait(timeout=5)

        except Exception as e:
            print(f"Error stopping simulation: {e}")

        finally:
            self._process = None
            # Clean up any remaining Gazebo processes
            self._gazebo.kill_gazebo()

    def is_running(self) -> bool:
        """Check if the simulation is running."""
        if self._process is None:
            return False

        # Check if process is still alive
        if self._process.poll() is not None:
            self._process = None
            return False

        return self._gazebo.is_running()

    @property
    def process(self) -> Optional[subprocess.Popen]:
        """Get the subprocess handle."""
        return self._process

    def __enter__(self) -> 'SimulationLauncher':
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - ensure cleanup."""
        self.stop()


def kill_all_gazebo() -> None:
    """Kill all Gazebo processes. Useful for cleanup."""
    GazeboBackend().kill_gazebo()
