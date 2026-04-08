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
import signal
import subprocess
import time
from typing import List, Set

from sim_harness.simulator.simulator_interface import (
    SimulatorConfig,
    SimulatorInterface,
    SimulatorType,
)


# Resolve gz command path once at module load
_GZ_CMD: str = "gz"
_gz_vendor = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
if os.path.exists(_gz_vendor):
    _GZ_CMD = _gz_vendor


# Patterns to match Gazebo processes
GAZEBO_PROCESS_PATTERNS = [
    "gz sim",
    "gz-sim",
    "ruby.*gz",
    "gzserver",
    "gzclient",
]

# ROS processes spawned alongside the simulator that must be cleaned up
ROS_SIM_PROCESS_PATTERNS = [
    "parameter_bridge",
    "robot_state_publisher",
    "bt_phase_marker",
    "vehicle_markers",
    "lidar_body_filter",
    "lidar_noise_injector",
    "ekf_wrapper_node",
    "twist_to_twist_stamped",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "lifecycle_manager",
]


def _find_pids_by_patterns(patterns: List[str]) -> Set[int]:
    """Find process IDs matching any of the patterns using a single pgrep call."""
    # Combine into one regex: "pattern1|pattern2|..."
    combined = "|".join(patterns)
    try:
        result = subprocess.run(
            ["pgrep", "-f", combined],
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
                    if pid != my_pid and pid != os.getppid():
                        pids.add(pid)
                except ValueError:
                    pass
        return pids
    except Exception as e:
        print(f"[Gazebo] Warning: failed to find PIDs for "
              f"{patterns}: {e}")
        return set()


def _find_pids_by_pattern(pattern: str) -> Set[int]:
    """Find process IDs matching a pattern using pgrep."""
    return _find_pids_by_patterns([pattern])


def _is_gazebo_process(pid: int) -> bool:
    """Check if a PID is actually a Gazebo process (not just a command mentioning gz)."""
    try:
        # Skip zombie processes — they can't be killed and shouldn't block startup
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("State:"):
                    if "Z" in line:
                        return False
                    break

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


def _is_pid_alive(pid: int) -> bool:
    """Check if a process is alive using signal 0."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # Exists but we don't have permission


def _check_pids_alive(pids: Set[int]) -> Set[int]:
    """Return the subset of *pids* that are still alive."""
    return {pid for pid in pids if _is_pid_alive(pid)}


def _send_signal_to_pids(
    pids: Set[int], sig: signal.Signals, signal_name: str
) -> None:
    """Send a signal to each PID individually, with logging."""
    for pid in pids:
        try:
            os.kill(pid, sig)
            print(f"[cleanup] Sent {signal_name} to PID {pid}")
        except ProcessLookupError:
            print(f"[cleanup] PID {pid} already dead")
        except PermissionError:
            print(f"[cleanup] WARNING: No permission to kill PID {pid}")


_GRACEFUL_TIMEOUT_SEC = 15
_SHUTDOWN_CHECK_INTERVAL_SEC = 1
_POST_KILL_WAIT_SEC = 2


def _wait_for_pids_shutdown(
    pids: Set[int], timeout: int = _GRACEFUL_TIMEOUT_SEC, tag: str = "cleanup"
) -> Set[int]:
    """Wait for PIDs to exit, returning those still alive after *timeout*."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        remaining = _check_pids_alive(pids)
        if not remaining:
            return set()
        elapsed = int(time.monotonic() - start)
        if elapsed > 0 and elapsed % 3 == 0:
            print(f"[{tag}] Waiting for {len(remaining)} process(es)... "
                  f"({elapsed}s/{timeout}s) PIDs: {remaining}")
        time.sleep(_SHUTDOWN_CHECK_INTERVAL_SEC)
    return _check_pids_alive(pids)


def _reap_zombies() -> int:
    """Reap any zombie child processes of the current process.

    Returns the number of zombies reaped.  This prevents zombie accumulation
    when children are killed but never waited on.
    """
    reaped = 0
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if pid == 0:
                break
            reaped += 1
        except ChildProcessError:
            # No child processes at all
            break
    if reaped:
        print(f"[cleanup] Reaped {reaped} zombie process(es)")
    return reaped


def _graceful_kill(pids: Set[int], tag: str = "cleanup") -> None:
    """SIGTERM → wait → SIGKILL flow for a set of PIDs."""
    if not pids:
        return
    alive = _check_pids_alive(pids)
    if not alive:
        return
    _send_signal_to_pids(alive, signal.SIGTERM, "SIGTERM")
    remaining = _wait_for_pids_shutdown(alive, _GRACEFUL_TIMEOUT_SEC, tag)
    if remaining:
        print(f"[{tag}] {len(remaining)} process(es) did not exit gracefully, "
              f"sending SIGKILL: {remaining}")
        _send_signal_to_pids(remaining, signal.SIGKILL, "SIGKILL")
        time.sleep(_POST_KILL_WAIT_SEC)
    # Reap any zombies that are our children
    _reap_zombies()


def _kill_processes_by_patterns(patterns: List[str], sig: int = 9) -> None:
    """Kill processes matching any of the patterns using a single pkill call."""
    combined = "|".join(patterns)
    try:
        subprocess.run(
            ["pkill", f"-{sig}", "-f", combined],
            capture_output=True,
            timeout=5
        )
    except Exception as e:
        print(f"[Gazebo] Warning: pkill failed for pattern "
              f"'{combined}': {e}")


def _kill_processes_by_pattern(pattern: str, sig: int = 9) -> None:
    """Kill processes matching a pattern."""
    _kill_processes_by_patterns([pattern], sig)


class GazeboBackend(SimulatorInterface):
    """
    Gazebo Harmonic simulator backend.

    Provides Gazebo-specific functionality for:
    - Detecting if Gazebo is running
    - Managing GZ_PARTITION for test isolation
    - Waiting for Gazebo to be ready
    """

    def __init__(self, config=None):
        super().__init__(config)

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

    @staticmethod
    def _list_gz_topics(timeout_sec: float = 5.0) -> List[str]:
        """List all Gazebo topics using the gz CLI (cached path)."""
        try:
            result = subprocess.run(
                [_GZ_CMD, "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=timeout_sec
            )
            if result.returncode != 0:
                print(f"[Gazebo] gz topic -l returned exit code "
                      f"{result.returncode}: {result.stderr.strip()}")
                return []
            return result.stdout.strip().split('\n')
        except subprocess.TimeoutExpired:
            print(f"[Gazebo] gz topic -l timed out after {timeout_sec}s")
            return []
        except Exception as e:
            print(f"[Gazebo] Failed to list topics: {e}")
            return []

    def is_responsive(self, timeout_sec: float = 10.0) -> bool:
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

        topics = self._list_gz_topics(timeout_sec)
        return any('/clock' in topic for topic in topics)

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

        Uses a single pgrep call with combined pattern for efficiency.
        """
        pids = _find_pids_by_patterns(GAZEBO_PROCESS_PATTERNS)
        # Filter to actual Gazebo processes (not just commands mentioning gz)
        return {pid for pid in pids if _is_gazebo_process(pid)}

    def is_gazebo_topic_available(self, topic: str) -> bool:
        """
        Check if a specific Gazebo topic is available.

        Performs an exact match against the list of available topics.

        Args:
            topic: Topic name (exact match required)

        Returns:
            True if topic exists in the topic list
        """
        return topic in self._list_gz_topics()

    def start(self, config=None) -> bool:
        """Start is handled externally by SimulationLauncher."""
        return True

    def stop(self) -> bool:
        """Stop Gazebo gracefully."""
        self.kill_gazebo()
        return True

    def shutdown(self) -> None:
        """Forcefully shutdown Gazebo."""
        self.kill_all_sim_processes()

    def kill_gazebo(self, graceful: bool = True) -> None:
        """Kill all Gazebo processes.

        Args:
            graceful: If True, send SIGTERM first and wait before SIGKILL.
                If False, send SIGKILL immediately (legacy behaviour).
        """
        if graceful:
            pids = self.get_gazebo_pids()
            _graceful_kill(pids, tag="kill_gazebo")
        else:
            _kill_processes_by_patterns(GAZEBO_PROCESS_PATTERNS, sig=9)

    def kill_all_sim_processes(self, graceful: bool = True) -> None:
        """Kill Gazebo and all associated ROS sim processes.

        Cleans up ``parameter_bridge``, ``robot_state_publisher``, and other
        orphan processes that ``kill_gazebo()`` alone would leave behind.

        Args:
            graceful: If True, send SIGTERM first and wait before SIGKILL.
                If False, send SIGKILL immediately (legacy behaviour).
        """
        if graceful:
            all_patterns = GAZEBO_PROCESS_PATTERNS + ROS_SIM_PROCESS_PATTERNS
            pids = _find_pids_by_patterns(all_patterns)
            _graceful_kill(pids, tag="kill_all_sim")
        else:
            _kill_processes_by_patterns(
                GAZEBO_PROCESS_PATTERNS + ROS_SIM_PROCESS_PATTERNS, sig=9
            )
            _reap_zombies()

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

    def __init__(self, config=None):
        super().__init__(config)

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

    def start(self, config=None) -> bool:
        return True

    def stop(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass
