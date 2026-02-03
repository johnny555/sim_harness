# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Spin helpers for ROS 2 executors."""

import time


def spin_for_duration(executor, duration_sec: float) -> None:
    """Spin the executor for *duration_sec* seconds."""
    end = time.monotonic() + duration_sec
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=min(0.1, max(0, end - time.monotonic())))


def spin_until_condition(executor, condition, timeout_sec: float) -> bool:
    """Spin until *condition()* returns True or timeout. Returns whether condition was met."""
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False


def spin_until_messages_received(executor, collector, count: int, timeout_sec: float) -> bool:
    """Spin until *collector* has at least *count* messages."""
    return spin_until_condition(executor, lambda: collector.count() >= count, timeout_sec)
