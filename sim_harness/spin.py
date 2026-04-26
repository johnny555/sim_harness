# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Spin helpers for ROS 2 executors."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


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


# -- Sleep-based alternatives (for use when executor spins in background) ---


def wait_for_duration(duration_sec: float) -> None:
    """Sleep for *duration_sec*. Use when executor is spinning in a background thread."""
    time.sleep(duration_sec)


def wait_until_condition(condition, timeout_sec: float) -> bool:
    """Poll *condition()* with sleep until True or timeout."""
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return False


# ---------------------------------------------------------------------------
# Unified executor context
# ---------------------------------------------------------------------------


class ExecutorContext:
    """One-stop context for "spin or sleep" check loops.

    Subsumes the three former helpers (``_acquire_executor``,
    ``_acquire_managed``, ``_managed_executor``). Three modes, picked at
    construction:

    1. **Caller-supplied executor** — the executor was passed in. Assumed to
       already be spinning (e.g. fixture's background MultiThreadedExecutor),
       so ``wait*`` falls through to ``time.sleep`` rather than spinning.
    2. **Managed self-spin** — no executor provided and no service node
       requested. Adds the caller's ``node`` to a fresh
       ``SingleThreadedExecutor`` and spins it. If the node is already in
       another executor, falls back to mode 4.
    3. **Managed temp service node** — ``service_node_prefix`` provided.
       Creates a throwaway node + new executor; ``self.service_node`` points
       at it (used for client creation).
    4. **Unmanaged sleep fallback** — node is already attached to a foreign
       executor we can't see; ``wait*`` sleeps.

    Use as a context manager: ``with ExecutorContext(node) as ec: ec.wait(...)``.
    """

    def __init__(self, node: Node, executor=None, service_node_prefix: str = None):
        self.user_node = node
        self.service_node = node  # default — overridden in temp-node mode
        self.temp_node = None
        self.executor = executor
        self.managed = False  # True iff we own the executor (must clean it up)

        if executor is not None:
            # Mode 1: caller's executor is already spinning somewhere.
            return

        if service_node_prefix is not None:
            # Mode 3: dedicated temp service node + fresh executor.
            unique = f"{service_node_prefix}_{int(time.time() * 1000) % 10000}"
            self.temp_node = rclpy.create_node(unique)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.temp_node)
            self.service_node = self.temp_node
            self.managed = True
            return

        # Mode 2: try to attach caller's node to a managed executor.
        exc = SingleThreadedExecutor()
        try:
            exc.add_node(node)
        except RuntimeError:
            # Mode 4: caller's node is already in another executor. Sleep.
            self.executor = None
            return
        self.executor = exc
        self.managed = True

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        """Spin one iteration if managed; otherwise sleep ``timeout_sec``."""
        if self.managed and self.executor is not None:
            self.executor.spin_once(timeout_sec=timeout_sec)
        else:
            time.sleep(timeout_sec)

    def wait(self, duration_sec: float) -> None:
        """Spin/sleep for ``duration_sec`` regardless of mode."""
        if self.managed and self.executor is not None:
            spin_for_duration(self.executor, duration_sec)
        else:
            time.sleep(duration_sec)

    def wait_until(self, condition, timeout_sec: float) -> bool:
        """Spin or sleep until ``condition()`` is True or ``timeout_sec`` elapses."""
        if self.managed and self.executor is not None:
            return spin_until_condition(self.executor, condition, timeout_sec)
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.05)
        return False

    def cleanup(self, client=None) -> None:
        """Tear down owned resources. Optionally destroys a service client."""
        if client is not None:
            try:
                self.service_node.destroy_client(client)
            except Exception:
                pass
        if not self.managed or self.executor is None:
            return
        if self.temp_node is not None:
            try:
                self.executor.remove_node(self.temp_node)
                self.temp_node.destroy_node()
            except Exception:
                pass
        else:
            # Mode 2: we attached the caller's node; detach it.
            try:
                self.executor.remove_node(self.user_node)
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, tb):
        self.cleanup()
        return False
