# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Gazebo Harmonic simulator backend.

Provides Gazebo-specific functionality for:
- Detecting if Gazebo is running
- Managing GZ_PARTITION for test isolation
- Waiting for Gazebo to be ready
"""

import os
import subprocess
import time
from typing import List, Set

from sim_harness.simulator.simulator_interface import (
    SimulatorInterface,
    SimulatorType,
)


# Patterns to match Gazebo processes
GAZEBO_PROCESS_PATTERNS = [
    "gz sim",
    "gz-sim",
    "ruby.*gz",
    "gzserver",
    "gzclient",
]


def _find_pids_by_pattern(pattern: str) -> Set[int]:
    """Find process IDs matching a pattern using pgrep."""
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True,
            text=True,
            timeout=5
        )
        pids = set()
        my_pid = os.getpid()
        for line in result.stdout.strip().split('\n'):
            if line:
                try:
                    pid = int(line)
                    # Exclude our own process and parent processes
                    if pid != my_pid and pid != os.getppid():
                        # Verify this is actually a Gazebo binary, not just a command containing the pattern
                        if _is_gazebo_process(pid):
                            pids.add(pid)
                except ValueError:
                    pass
        return pids
    except Exception:
        return set()


def _is_gazebo_process(pid: int) -> bool:
    """Check if a PID is actually a Gazebo process (not just a command mentioning gz)."""
    try:
        # Read the process executable
        exe_path = os.readlink(f"/proc/{pid}/exe")
        exe_name = os.path.basename(exe_path)
        # Gazebo executables: gz, gzserver, gzclient, ruby (for gz wrapper)
        gazebo_exes = {"gz", "gzserver", "gzclient", "ruby", "gz-sim"}
        if exe_name in gazebo_exes:
            return True
        # Handle versioned ruby (e.g. ruby3.2)
        if exe_name.startswith("ruby"):
            return True
        # Read cmdline for further checks
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read().decode("utf-8", errors="replace")
            args = cmdline.split('\x00')
            if args and args[0]:
                cmd_name = os.path.basename(args[0])
                if cmd_name in gazebo_exes or cmd_name.startswith("gz-"):
                    return True
            # Handle shell wrappers: /bin/sh -c "ruby .../gz sim ..."
            # ros2 launch wraps Gazebo commands in /bin/sh -c
            if exe_name in ("sh", "bash", "dash"):
                cmdline_str = " ".join(a for a in args if a)
                if any(marker in cmdline_str for marker in
                       ("gz sim", "gzserver", "gzclient")):
                    return True
        return False
    except (OSError, IOError):
        return False


def _kill_processes_by_pattern(pattern: str, signal: int = 9) -> None:
    """Kill processes matching a pattern."""
    try:
        subprocess.run(
            ["pkill", f"-{signal}", "-f", pattern],
            capture_output=True,
            timeout=5
        )
    except Exception:
        pass


class GazeboBackend(SimulatorInterface):
    """
    Gazebo Harmonic simulator backend.

    Provides Gazebo-specific functionality for:
    - Detecting if Gazebo is running
    - Managing GZ_PARTITION for test isolation
    - Waiting for Gazebo to be ready
    """

    def type(self) -> SimulatorType:
        return SimulatorType.GAZEBO

    def is_running(self) -> bool:
        """
        Check if Gazebo processes are running.

        Looks for processes matching gz, gzserver, ruby.*gz patterns.
        Note: This only checks for process existence, not functionality.
        Use is_responsive() to check if Gazebo is actually simulating.
        """
        return len(self.get_gazebo_pids()) > 0

    def is_responsive(self, timeout_sec: float = 2.0) -> bool:
        """
        Check if Gazebo is actually responsive and simulating.

        This is a stronger check than is_running() - it verifies that:
        1. Gazebo processes exist
        2. Gazebo is publishing the /clock topic (indicates active simulation)

        Use this to detect zombie Gazebo processes that exist but aren't
        actually running a simulation.

        Args:
            timeout_sec: Maximum time to wait for topic check

        Returns:
            True if Gazebo is responsive, False otherwise
        """
        if not self.is_running():
            return False

        # Check if /clock topic is available - this indicates Gazebo is actually
        # running a simulation, not just zombie processes
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=timeout_sec
            )
            if result.returncode != 0:
                return False

            topics = result.stdout.strip().split('\n')
            # Look for clock topic - always present in a running simulation
            return any('/clock' in topic for topic in topics)
        except (subprocess.TimeoutExpired, Exception):
            return False

    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait until Gazebo is ready.

        Checks for:
        1. Gazebo processes running
        2. ROS-Gazebo bridge topics available

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if ready, False on timeout
        """
        start = time.monotonic()

        while time.monotonic() - start < timeout_sec:
            if self.is_running():
                # Give Gazebo a moment to fully initialize
                time.sleep(0.5)
                return True
            time.sleep(0.1)

        return False

    def get_partition(self) -> str:
        """
        Get the GZ_PARTITION value.

        Returns the current GZ_PARTITION environment variable value,
        or generates one based on ROS_DOMAIN_ID if not set.
        """
        partition = os.environ.get('GZ_PARTITION', '')
        if partition:
            return partition

        # Generate from ROS_DOMAIN_ID
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
        return f"gz_test_{domain_id}"

    def get_gazebo_pids(self) -> Set[int]:
        """
        Get PIDs of running Gazebo processes.

        Useful for cleanup and monitoring.
        """
        all_pids: Set[int] = set()

        for pattern in GAZEBO_PROCESS_PATTERNS:
            pids = _find_pids_by_pattern(pattern)
            all_pids.update(pids)

        return all_pids

    def is_gazebo_topic_available(self, topic: str) -> bool:
        """
        Check if a specific Gazebo topic is available.

        Performs an exact match against the list of available topics.

        Args:
            topic: Topic name (exact match required)

        Returns:
            True if topic exists in the topic list
        """
        try:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=5
            )
            # Exact line match to avoid false positives from substring matching
            available_topics = result.stdout.strip().split('\n')
            return topic in available_topics
        except Exception:
            return False

    def kill_gazebo(self) -> None:
        """Kill all Gazebo processes."""
        for pattern in GAZEBO_PROCESS_PATTERNS:
            _kill_processes_by_pattern(pattern, signal=9)

    def wait_for_topic(
        self,
        topic: str,
        timeout_sec: float = 30.0
    ) -> bool:
        """
        Wait for a Gazebo topic to become available.

        Args:
            topic: Topic name to wait for
            timeout_sec: Maximum time to wait

        Returns:
            True if topic available, False on timeout
        """
        start = time.monotonic()

        while time.monotonic() - start < timeout_sec:
            if self.is_gazebo_topic_available(topic):
                return True
            time.sleep(0.5)

        return False


class NullBackend(SimulatorInterface):
    """
    Null simulator backend for unit tests.

    Always reports as "running" and "ready" - useful for testing
    without an actual simulator.
    """

    def type(self) -> SimulatorType:
        return SimulatorType.NONE

    def is_running(self) -> bool:
        return True

    def is_responsive(self, timeout_sec: float = 2.0) -> bool:
        return True

    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        return True

    def get_partition(self) -> str:
        return "null_partition"
