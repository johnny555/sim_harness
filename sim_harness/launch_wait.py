# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Launch-time predicate gating for ROS 2 simulation stacks.

Replaces fixed ``TimerAction(period=N)`` delays with real predicates
(topic-is-publishing, service-is-available, lifecycle-is-active) so launch
timing adapts to the host machine instead of hard-coding worst-case
timeouts.

Example::

    from sim_harness.launch_wait import (
        WaitForCondition, topic_publishing, lifecycle_active,
    )

    wait_for_clock = WaitForCondition(
        condition=topic_publishing(
            '/clock', 'rosgraph_msgs/msg/Clock', min_rate_hz=5.0),
        actions=[OpaqueFunction(function=launch_setup)],
        timeout=300.0,
        description='Wait for /clock',
    )

    wait_for_bt = WaitForCondition(
        condition=lifecycle_active('/<namespace>/bt_navigator'),
        actions=[lifecycle_manager_navigation],
        timeout=60.0,
        description='Wait for bt_navigator active',
    )
"""

import asyncio
import os
import shutil
import subprocess
import threading
import time
from typing import Callable, Iterable, List, Optional, Union

from launch.action import Action
from launch.actions import RegisterEventHandler
from launch.event import Event
from launch.event_handler import EventHandler
from launch.events import Shutdown


# ---------------------------------------------------------------------------
# Custom launch event signalling that a WaitForCondition's predicate fired.
# ---------------------------------------------------------------------------


class _ConditionMetEvent(Event):
    """Emitted when a :class:`WaitForCondition` predicate becomes true."""

    name = 'sim_harness.launch_wait.ConditionMet'

    def __init__(self, *, source):
        self.source = source


# ---------------------------------------------------------------------------
# WaitForCondition action
# ---------------------------------------------------------------------------


class WaitForCondition(Action):
    """Launch action that runs *actions* once *condition()* returns ``True``.

    The predicate is polled on a daemon thread at ``poll_rate_hz``.  When it
    returns ``True`` the action
    emits an internal launch event; a registered handler then executes the
    child actions on the launch event loop. Timeout emits
    :class:`launch.events.Shutdown` by default.

    Parameters
    ----------
    condition : Callable[[], bool]
        Predicate polled periodically.  Must be safe to call from a
        background thread.  Exceptions are logged and treated as ``False``.
    actions : Iterable[launch.Action]
        Actions to execute once the condition is met. They also run on timeout
        only when ``on_timeout='proceed'`` is explicitly set.
    timeout : float, default 60.0
        Maximum seconds to wait before giving up.
    poll_rate_hz : float, default 2.0
        Polling frequency.
    on_timeout : {'proceed', 'fail'}, default 'fail'
        ``'fail'`` emits a :class:`launch.events.Shutdown` event.
        ``'proceed'`` emits the event anyway and runs the actions for
        intentional best-effort launch flows.
    description : str, optional
        Human-readable label for log lines.

    Returns (from ``execute``)
    -------
    list[Action]
        A single :class:`RegisterEventHandler` that will fire the child
        actions when the internal event is emitted.
    """

    def __init__(
        self,
        *,
        condition: Callable[[], bool],
        actions: Iterable[Action],
        timeout: float = 60.0,
        poll_rate_hz: float = 2.0,
        on_timeout: str = 'fail',
        description: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        if not callable(condition):
            raise TypeError(
                f'condition must be callable, got {type(condition).__name__}')
        if on_timeout not in ('proceed', 'fail'):
            raise ValueError(
                f"on_timeout must be 'proceed' or 'fail', got {on_timeout!r}")
        if timeout <= 0:
            raise ValueError(f'timeout must be > 0, got {timeout}')
        if poll_rate_hz <= 0:
            raise ValueError(f'poll_rate_hz must be > 0, got {poll_rate_hz}')

        self._condition = condition
        self._actions = list(actions)
        self._timeout = float(timeout)
        self._poll_interval = 1.0 / float(poll_rate_hz)
        self._on_timeout = on_timeout
        self._description = description or 'WaitForCondition'
        self._fired = threading.Event()

    # --- Event handler plumbing --------------------------------------------

    def _matches_self(self, event) -> bool:
        return isinstance(event, _ConditionMetEvent) and event.source is self

    # --- Launch lifecycle --------------------------------------------------

    def execute(self, context) -> List[Action]:
        handler = RegisterEventHandler(EventHandler(
            matcher=self._matches_self,
            entities=self._actions,
        ))

        loop = context.asyncio_loop
        threading.Thread(
            target=self._poll_loop,
            args=(context, loop),
            daemon=True,
            name=f'wait-{self._description}',
        ).start()

        return [handler]

    # --- Polling thread ----------------------------------------------------

    def _poll_loop(self, context, loop) -> None:
        start = time.monotonic()
        next_progress_log = start + 10.0

        while time.monotonic() - start < self._timeout:
            try:
                met = bool(self._condition())
            except Exception as exc:
                met = False
                print(
                    f'[{self._description}] condition raised {type(exc).__name__}: {exc}',
                    flush=True,
                )
            if met:
                elapsed = time.monotonic() - start
                print(
                    f'[{self._description}] met after {elapsed:.1f}s',
                    flush=True,
                )
                self._fire(context, loop, _ConditionMetEvent(source=self))
                return

            now = time.monotonic()
            if now >= next_progress_log:
                elapsed = now - start
                print(
                    f'[{self._description}] still waiting '
                    f'({elapsed:.0f}s / {self._timeout:.0f}s)',
                    flush=True,
                )
                next_progress_log = now + 10.0
            time.sleep(self._poll_interval)

        # Timeout branch
        print(
            f'[{self._description}] TIMEOUT after {self._timeout:.0f}s',
            flush=True,
        )
        if self._on_timeout == 'proceed':
            print(
                f'[{self._description}] on_timeout=proceed -- running '
                f'follow-up actions anyway',
                flush=True,
            )
            self._fire(context, loop, _ConditionMetEvent(source=self))
        else:
            self._fire(
                context, loop,
                Shutdown(reason=f'{self._description} timed out'),
            )

    def _fire(self, context, loop, event) -> None:
        """Schedule *event* emission on the launch event loop (thread-safe)."""
        if self._fired.is_set():
            return
        self._fired.set()

        def schedule():
            # context.emit_event is a coroutine; wrap as a task on this loop.
            loop.create_task(context.emit_event(event))

        loop.call_soon_threadsafe(schedule)


# ---------------------------------------------------------------------------
# Shared launch-side rclpy probe node
# ---------------------------------------------------------------------------

_probe_lock = threading.Lock()
_probe_node = None
_probe_executor = None


def _get_probe_node():
    """Return a process-wide rclpy node used by launch-side condition polls.

    Created lazily on first use; spun on a dedicated daemon thread so that
    service responses and subscription callbacks are processed while the
    polling threads are sleeping.
    """
    global _probe_node, _probe_executor

    with _probe_lock:
        if _probe_node is not None:
            return _probe_node

        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        if not rclpy.ok():
            rclpy.init()

        _probe_node = rclpy.create_node(
            f'sim_harness_launch_probe_{os.getpid()}')
        _probe_executor = SingleThreadedExecutor()
        _probe_executor.add_node(_probe_node)

        t = threading.Thread(
            target=_probe_executor.spin,
            daemon=True,
            name='sim_harness_launch_probe_spin',
        )
        t.start()
        return _probe_node


# ---------------------------------------------------------------------------
# Condition factories
# ---------------------------------------------------------------------------


def topic_publishing(
    topic: str,
    msg_type: Union[str, type],
    min_rate_hz: float = 1.0,
    sample_window_sec: float = 2.0,
) -> Callable[[], bool]:
    """Return a condition that's true once ``topic`` is publishing at rate.

    Parameters
    ----------
    topic : str
        Topic name (e.g. ``'/clock'``).
    msg_type : type or str
        Message class, or a string like ``'rosgraph_msgs/msg/Clock'``.
    min_rate_hz : float, default 1.0
        Required minimum rate in Hz, measured over the rolling sample window.
        Pass ``0`` (or any non-positive value) to fire on the first message.
    sample_window_sec : float, default 2.0
        How long a window (seconds) the rate is averaged over.
    """
    if isinstance(msg_type, str):
        from rosidl_runtime_py.utilities import get_message
        msg_cls = get_message(msg_type)
    else:
        msg_cls = msg_type

    state = {'sub': None, 'timestamps': []}
    lock = threading.Lock()

    def callback(_msg):
        now = time.monotonic()
        with lock:
            ts = state['timestamps']
            ts.append(now)
            # Keep only timestamps inside the window
            cutoff = now - sample_window_sec
            while ts and ts[0] < cutoff:
                ts.pop(0)

    def check() -> bool:
        node = _get_probe_node()
        with lock:
            if state['sub'] is None:
                state['sub'] = node.create_subscription(
                    msg_cls, topic, callback, 10)
                return False
            ts = list(state['timestamps'])
        if min_rate_hz <= 0:
            return len(ts) >= 1
        if len(ts) < 2:
            return False
        span = ts[-1] - ts[0]
        if span <= 0:
            return False
        rate = (len(ts) - 1) / span
        return rate >= min_rate_hz

    return check


def service_available(service_name: str) -> Callable[[], bool]:
    """Return a condition that's true once ``service_name`` is on the graph.

    Uses the ROS graph API (``get_service_names_and_types``) rather than
    creating a client, so it's cheap to poll.
    """
    def check() -> bool:
        node = _get_probe_node()
        names = [name for name, _types in node.get_service_names_and_types()]
        return service_name in names
    return check


def gz_service_available(
    service_name: str,
    *,
    command_timeout_sec: float = 2.0,
) -> Callable[[], bool]:
    """Return a condition that's true once a Gazebo transport service exists.

    This probes Gazebo transport directly via ``gz service -l`` instead of the
    ROS graph. It is intended for launch gates ahead of ``ros_gz_sim create``,
    which depends on Gazebo world discovery being live before any ROS topics
    such as ``/clock`` are useful.
    """

    gz_executable = shutil.which('gz')

    def check() -> bool:
        if not gz_executable:
            return False

        try:
            result = subprocess.run(
                [gz_executable, 'service', '-l'],
                capture_output=True,
                text=True,
                timeout=command_timeout_sec,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return False

        if result.returncode != 0:
            return False

        services = {line.strip() for line in result.stdout.splitlines() if line.strip()}
        return service_name in services

    return check


def gz_world_ready(world_name: str) -> Callable[[], bool]:
    """Return a condition that's true once Gazebo advertises world services."""

    required_services = (
        '/gazebo/worlds',
        f'/world/{world_name}/create_multiple',
    )
    checks = tuple(gz_service_available(name) for name in required_services)

    def check() -> bool:
        return all(predicate() for predicate in checks)

    return check


def lifecycle_active(node_name: str) -> Callable[[], bool]:
    """Return a condition that's true once a lifecycle node reports 'active'.

    ``node_name`` may be absolute (``'/ns/planner_server'``) or bare
    (``'ns/planner_server'``) — a leading slash is added if missing.
    """
    from lifecycle_msgs.srv import GetState

    normalized = node_name if node_name.startswith('/') else '/' + node_name
    service_name = f'{normalized}/get_state'

    state = {'client': None}

    def check() -> bool:
        node = _get_probe_node()
        if state['client'] is None:
            state['client'] = node.create_client(GetState, service_name)
        client = state['client']
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())

        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            return False
        resp = future.result()
        return resp is not None and resp.current_state.label == 'active'

    return check
