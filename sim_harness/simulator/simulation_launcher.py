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
        self._log_file = None

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
        # Ensure DISPLAY points to an available X socket
        current_display = env.get('DISPLAY', '')
        display_num = current_display.lstrip(':') if current_display else ''
        if not display_num or not os.path.exists(f'/tmp/.X11-unix/X{display_num}'):
            for d in [':0', ':1', ':99']:
                if os.path.exists(f'/tmp/.X11-unix/X{d[1:]}'):
                    env['DISPLAY'] = d
                    break
            else:
                env['DISPLAY'] = current_display or ':0'

        # Start the process
        # Note: stdout must NOT be PIPE unless actively read, as the buffer
        # fills and blocks the subprocess (especially Gazebo's verbose output).
        print(f"[SimLauncher] Starting: {' '.join(cmd)}", flush=True)
        print(f"[SimLauncher] ROS_DOMAIN_ID={env.get('ROS_DOMAIN_ID', 'NOT SET')}, "
              f"GZ_PARTITION={env.get('GZ_PARTITION', 'NOT SET')}, "
              f"DISPLAY={env.get('DISPLAY', 'NOT SET')}", flush=True)
        log_path = f'/tmp/sim_launch_{os.getpid()}.log'
        print(f"[SimLauncher] Log: {log_path}", flush=True)
        log_file = open(log_path, 'w')
        try:
            self._process = subprocess.Popen(
                cmd,
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid  # Create new process group for cleanup
            )
        except Exception as e:
            print(f"Failed to start simulation: {e}")
            log_file.close()
            return False
        self._log_file = log_file  # Keep reference for cleanup

        if wait_for_ready:
            return self.wait_until_ready(config.startup_timeout_sec)

        return True

    def wait_until_ready(self, timeout_sec: float = 60.0) -> bool:
        """
        Wait until the simulation is ready.

        Checks for Gazebo processes and waits for initialization. After
        Gazebo is detected, an additional delay is applied based on
        the launch config's gazebo_startup_delay_sec setting to allow
        for full initialization.

        Also monitors the launched subprocess — if it dies before Gazebo
        is detected, returns immediately instead of waiting for the full
        timeout.

        Note:
            Total wait time may exceed timeout_sec due to the additional
            startup delay applied after Gazebo processes are detected.

        Args:
            timeout_sec: Maximum time to wait for Gazebo processes

        Returns:
            True if ready, False on timeout or subprocess death
        """
        start = time.monotonic()
        print(f"[SimLauncher] Waiting for ready (timeout={timeout_sec}s)...",
              flush=True)
        logged_at = 0
        while time.monotonic() - start < timeout_sec:
            elapsed = time.monotonic() - start
            # Check if our launch subprocess died
            if self._process is not None and self._process.poll() is not None:
                print(f"[SimLauncher] Subprocess exited with code "
                      f"{self._process.returncode} after {elapsed:.1f}s",
                      flush=True)
                return False
            if self._gazebo.is_running():
                # Process detected — now wait for /clock (confirms active sim)
                print(f"[SimLauncher] Gazebo process detected after {elapsed:.1f}s, "
                      f"waiting for /clock...", flush=True)
                remaining = timeout_sec - elapsed
                resp_start = time.monotonic()
                while time.monotonic() - resp_start < remaining:
                    if self._gazebo.is_responsive():
                        resp_elapsed = time.monotonic() - resp_start
                        print(f"[SimLauncher] Gazebo responsive after "
                              f"{elapsed + resp_elapsed:.1f}s total", flush=True)
                        if self._config:
                            time.sleep(self._config.gazebo_startup_delay_sec)
                        return True
                    time.sleep(0.5)
                # Process exists but never became responsive — treat as success
                # with extra startup delay to allow ROS bridge initialization
                print(f"[SimLauncher] Gazebo process alive but /clock not "
                      f"confirmed; proceeding with startup delay", flush=True)
                if self._config:
                    time.sleep(self._config.gazebo_startup_delay_sec)
                return True
            if int(elapsed) > logged_at and int(elapsed) % 10 == 0:
                logged_at = int(elapsed)
                poll = (self._process.poll() if self._process else 'N/A')
                print(f"[SimLauncher] Still waiting... {elapsed:.0f}s "
                      f"(process poll={poll})", flush=True)
            time.sleep(0.5)
        print(f"[SimLauncher] Timeout after {timeout_sec}s", flush=True)
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
            # Close log file descriptor
            if self._log_file:
                try:
                    self._log_file.close()
                except Exception:
                    pass
                self._log_file = None
            # Clean up Gazebo and associated ROS sim processes
            self._gazebo.kill_all_sim_processes()

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
    """Kill all Gazebo and associated ROS sim processes. Useful for cleanup."""
    GazeboBackend().kill_all_sim_processes()
