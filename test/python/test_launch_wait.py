# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for :mod:`sim_harness.launch_wait`.

Exercises the ``WaitForCondition`` polling loop and timeout branching
without a real launch run — uses a fake context whose ``emit_event`` records
the event and runs the registered matcher/entities synchronously.
"""
import asyncio
import subprocess
import threading
import time

import pytest

from sim_harness.launch_wait import WaitForCondition, _ConditionMetEvent


class _FakeContext:
    """Minimal stand-in for ``launch.LaunchContext``.

    Records every event emitted, and fires the matchers of any
    EventHandlers seen via the entities returned from ``execute``.
    """

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.emitted: list = []
        self._loop_thread = threading.Thread(
            target=self._run_loop, daemon=True)
        self._loop_thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    @property
    def asyncio_loop(self):
        return self.loop

    async def emit_event(self, event):
        self.emitted.append(event)

    def close(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._loop_thread.join(timeout=1.0)


@pytest.fixture
def ctx():
    c = _FakeContext()
    yield c
    c.close()


def _wait_for(pred, timeout=5.0, interval=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(interval)
    return False


def test_condition_met_emits_event(ctx):
    calls = {'n': 0}

    def cond():
        calls['n'] += 1
        return calls['n'] >= 3

    action = WaitForCondition(
        condition=cond, actions=[], timeout=5.0, poll_rate_hz=50.0,
        description='met-test')
    action.execute(ctx)

    assert _wait_for(lambda: len(ctx.emitted) == 1), \
        'expected ConditionMetEvent emission'
    event = ctx.emitted[0]
    assert isinstance(event, _ConditionMetEvent)
    assert event.source is action


def test_timeout_proceed_emits_event(ctx):
    action = WaitForCondition(
        condition=lambda: False, actions=[], timeout=0.2,
        poll_rate_hz=50.0, on_timeout='proceed',
        description='proceed-test')
    action.execute(ctx)

    assert _wait_for(lambda: len(ctx.emitted) == 1, timeout=2.0)
    assert isinstance(ctx.emitted[0], _ConditionMetEvent)


def test_timeout_default_emits_shutdown(ctx):
    from launch.events import Shutdown

    action = WaitForCondition(
        condition=lambda: False, actions=[], timeout=0.2,
        poll_rate_hz=50.0,
        description='fail-test')
    action.execute(ctx)

    assert _wait_for(lambda: len(ctx.emitted) == 1, timeout=2.0)
    assert isinstance(ctx.emitted[0], Shutdown)


def test_condition_exception_treated_as_false(ctx):
    calls = {'n': 0}

    def cond():
        calls['n'] += 1
        if calls['n'] < 3:
            raise RuntimeError('transient')
        return True

    action = WaitForCondition(
        condition=cond, actions=[], timeout=5.0, poll_rate_hz=50.0,
        description='raising-test')
    action.execute(ctx)

    assert _wait_for(lambda: len(ctx.emitted) == 1)
    assert isinstance(ctx.emitted[0], _ConditionMetEvent)


def test_fires_only_once(ctx):
    action = WaitForCondition(
        condition=lambda: True, actions=[], timeout=5.0,
        poll_rate_hz=100.0, description='once-test')
    action.execute(ctx)
    time.sleep(0.3)
    assert len(ctx.emitted) == 1


def test_rejects_invalid_arguments():
    with pytest.raises(TypeError):
        WaitForCondition(condition='not-callable', actions=[])
    with pytest.raises(ValueError):
        WaitForCondition(
            condition=lambda: True, actions=[], on_timeout='bogus')
    with pytest.raises(ValueError):
        WaitForCondition(
            condition=lambda: True, actions=[], timeout=-1)
    with pytest.raises(ValueError):
        WaitForCondition(
            condition=lambda: True, actions=[], poll_rate_hz=0)


def test_topic_publishing_zero_rate_fires_on_first_message(monkeypatch):
    """With ``min_rate_hz=0`` the check should return True after one sample."""
    from sim_harness import launch_wait

    class _FakeSub:
        pass

    class _FakeNode:
        def create_subscription(self, msg_cls, topic, callback, depth):
            _FakeNode.captured_callback = callback
            return _FakeSub()

    fake_node = _FakeNode()
    monkeypatch.setattr(launch_wait, '_get_probe_node', lambda: fake_node)

    check = launch_wait.topic_publishing(
        '/some_topic', msg_type=object, min_rate_hz=0)

    # First call subscribes and returns False (no messages yet)
    assert check() is False

    # Deliver exactly one message; with min_rate_hz=0 that's enough
    _FakeNode.captured_callback(object())
    assert check() is True


def test_topic_publishing_positive_rate_requires_two_messages(monkeypatch):
    """With a positive min_rate_hz, one message is not enough."""
    from sim_harness import launch_wait

    class _FakeNode:
        def create_subscription(self, msg_cls, topic, callback, depth):
            _FakeNode.captured_callback = callback
            return object()

    fake_node = _FakeNode()
    monkeypatch.setattr(launch_wait, '_get_probe_node', lambda: fake_node)

    check = launch_wait.topic_publishing(
        '/some_topic', msg_type=object, min_rate_hz=1.0)
    assert check() is False

    _FakeNode.captured_callback(object())
    assert check() is False  # only one sample, still insufficient


def test_execute_returns_event_handler(ctx):
    from launch.actions import RegisterEventHandler

    action = WaitForCondition(
        condition=lambda: False, actions=[], timeout=0.1,
        poll_rate_hz=50.0, description='handler-test')
    result = action.execute(ctx)
    assert len(result) == 1
    assert isinstance(result[0], RegisterEventHandler)


def test_gz_service_available_detects_service(monkeypatch):
    from sim_harness import launch_wait

    monkeypatch.setattr(launch_wait.shutil, 'which', lambda _name: '/usr/bin/gz')

    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=['gz', 'service', '-l'],
            returncode=0,
            stdout='/gazebo/worlds\n/world/sellafield/create_multiple\n',
            stderr='',
        )

    monkeypatch.setattr(launch_wait.subprocess, 'run', fake_run)

    check = launch_wait.gz_service_available('/gazebo/worlds')
    assert check() is True


def test_gz_service_available_handles_failures(monkeypatch):
    from sim_harness import launch_wait

    monkeypatch.setattr(launch_wait.shutil, 'which', lambda _name: '/usr/bin/gz')

    def fake_run(*args, **kwargs):
        raise subprocess.TimeoutExpired(cmd='gz service -l', timeout=2.0)

    monkeypatch.setattr(launch_wait.subprocess, 'run', fake_run)

    check = launch_wait.gz_service_available('/gazebo/worlds')
    assert check() is False


def test_gz_world_ready_requires_world_and_list_services(monkeypatch):
    from sim_harness import launch_wait

    available = {'/gazebo/worlds', '/world/sellafield/create_multiple'}

    def fake_gz_service_available(name, **_kwargs):
        return lambda: name in available

    monkeypatch.setattr(launch_wait, 'gz_service_available', fake_gz_service_available)

    assert launch_wait.gz_world_ready('sellafield')() is True
    available.remove('/world/sellafield/create_multiple')
    assert launch_wait.gz_world_ready('sellafield')() is False
