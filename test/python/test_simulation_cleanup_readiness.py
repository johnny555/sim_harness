# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for simulation cleanup ownership and readiness defaults."""

import signal
import subprocess
import threading
from unittest.mock import Mock, call

from sim_harness.simulator import simulation_manager
from sim_harness.simulator.simulation_launcher import (
    LaunchConfig,
    SimulationLauncher,
)
from sim_harness.simulator.simulation_manager import (
    SimulationManager,
    SimulationRequest,
)


def _launcher_with_process() -> tuple[SimulationLauncher, Mock]:
    launcher = SimulationLauncher()
    process = Mock()
    process.pid = 1234
    process.poll.return_value = None
    process.wait.return_value = None
    launcher._process = process
    launcher._process_group_id = 4321
    launcher._gazebo = Mock()
    launcher._log_file = Mock()
    return launcher, process


def test_launcher_stop_signals_owned_process_group(monkeypatch):
    launcher, _ = _launcher_with_process()
    killpg = Mock()
    monkeypatch.setattr("os.killpg", killpg)

    launcher.stop()

    killpg.assert_called_once_with(4321, signal.SIGINT)


def test_launcher_stop_does_not_global_cleanup_by_default(monkeypatch):
    launcher, _ = _launcher_with_process()
    monkeypatch.setattr("os.killpg", Mock())

    launcher.stop()

    launcher._gazebo.kill_all_sim_processes.assert_not_called()


def test_launcher_stop_global_cleanup_is_explicit(monkeypatch):
    launcher, _ = _launcher_with_process()
    monkeypatch.setattr("os.killpg", Mock())
    gazebo = launcher._gazebo

    launcher.stop(allow_global_cleanup=True)

    gazebo.kill_all_sim_processes.assert_called_once()


def test_launcher_stop_escalates_owned_process_group(monkeypatch):
    launcher, process = _launcher_with_process()
    process.wait.side_effect = [
        subprocess.TimeoutExpired(cmd="ros2 launch", timeout=1.0),
        None,
    ]
    killpg = Mock()
    monkeypatch.setattr("os.killpg", killpg)

    launcher.stop()

    assert killpg.call_args_list == [
        call(4321, signal.SIGINT),
        call(4321, signal.SIGKILL),
    ]


def test_wait_until_ready_returns_false_when_subprocess_exits():
    launcher = SimulationLauncher()
    launcher._process = Mock()
    launcher._process.returncode = 2
    launcher._process.poll.return_value = 2

    assert launcher.wait_until_ready(timeout_sec=1.0) is False


def test_wait_until_ready_requires_clock_by_default(monkeypatch):
    launcher = SimulationLauncher()
    launcher._process = Mock()
    launcher._process.poll.return_value = None
    launcher._gazebo = Mock()
    launcher._gazebo.is_running.return_value = True
    launcher._gazebo.is_responsive.return_value = False
    monkeypatch.setattr("time.sleep", lambda _: None)

    assert launcher.wait_until_ready(timeout_sec=0.01) is False


def test_wait_until_ready_accepts_responsive_gazebo(monkeypatch):
    launcher = SimulationLauncher()
    launcher._config = LaunchConfig(
        package="pkg",
        launch_file="sim.launch.py",
        gazebo_startup_delay_sec=0.0,
    )
    launcher._process = Mock()
    launcher._process.poll.return_value = None
    launcher._gazebo = Mock()
    launcher._gazebo.is_running.return_value = True
    launcher._gazebo.is_responsive.return_value = True
    monkeypatch.setattr("time.sleep", lambda _: None)

    assert launcher.wait_until_ready(timeout_sec=1.0) is True


def test_wait_until_ready_process_only_requires_explicit_flag(monkeypatch):
    launcher = SimulationLauncher()
    launcher._config = LaunchConfig(
        package="pkg",
        launch_file="sim.launch.py",
        gazebo_startup_delay_sec=0.0,
        allow_process_only_ready=True,
    )
    launcher._process = Mock()
    launcher._process.poll.return_value = None
    launcher._gazebo = Mock()
    launcher._gazebo.is_running.return_value = True
    launcher._gazebo.is_responsive.return_value = False
    monkeypatch.setattr("time.sleep", lambda _: None)

    assert launcher.wait_until_ready(timeout_sec=0.01) is True


def _manager_with_unresponsive_external_sim() -> tuple[SimulationManager, Mock]:
    manager = object.__new__(SimulationManager)
    manager._launcher = None
    manager._current_request = None
    manager._current_hash = ""
    manager._gazebo = Mock()
    manager._gazebo.is_responsive.return_value = False
    manager._gazebo.is_running.return_value = True
    manager._isolation_domain_id = None
    manager._started_by_us = False
    manager._active_users = 0
    manager._lock_handle = None
    manager._lock = threading.Lock()
    return manager, manager._gazebo


def test_manager_does_not_kill_unresponsive_external_processes(monkeypatch):
    manager, gazebo = _manager_with_unresponsive_external_sim()
    release_lock = Mock()
    monkeypatch.setattr(
        simulation_manager, "_acquire_gazebo_lock", lambda: object())
    monkeypatch.setattr(simulation_manager, "_release_gazebo_lock",
                        release_lock)

    result = manager.request(
        SimulationRequest(package="pkg", launch_file="sim.launch.py"),
        require_sim=False,
    )

    assert result is False
    gazebo.kill_all_sim_processes.assert_not_called()
    release_lock.assert_called_once()


def test_manager_global_cleanup_for_external_processes_is_explicit(monkeypatch):
    manager, gazebo = _manager_with_unresponsive_external_sim()
    monkeypatch.setattr(
        simulation_manager, "_acquire_gazebo_lock", lambda: object())
    monkeypatch.setattr(manager, "_start_internal", lambda *args: False)
    monkeypatch.setattr("time.sleep", lambda _: None)

    result = manager.request(
        SimulationRequest(package="pkg", launch_file="sim.launch.py"),
        require_sim=False,
        allow_global_cleanup=True,
    )

    assert result is False
    gazebo.kill_all_sim_processes.assert_called_once()
