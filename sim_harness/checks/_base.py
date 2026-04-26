# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Shared result types and small helpers used by every checks submodule.

Kept private (leading underscore module name) — public callers should import
from :mod:`sim_harness.checks` directly, which re-exports the result types.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple, TypeVar

from rclpy.node import Node

from sim_harness.spin import ExecutorContext

MsgT = TypeVar('MsgT')


# -- Result types ──────────────────────────────────────────────────────────


@dataclass
class ServiceResult:
    available: bool = False
    call_succeeded: bool = False
    response_time_ms: float = 0.0
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.available


@dataclass
class SensorDataResult:
    valid: bool = False
    message_count: int = 0
    publish_rate_hz: float = 0.0
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.valid


@dataclass
class TimingResult:
    within_bounds: bool = False
    measured_rate_hz: float = 0.0
    min_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    avg_latency_ms: float = 0.0
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.within_bounds


@dataclass
class MovementResult:
    success: bool = False
    distance_moved: float = 0.0
    start_position: Tuple[float, float, float] = (0, 0, 0)
    end_position: Tuple[float, float, float] = (0, 0, 0)
    details: str = ""
    ground_truth_distance: Optional[float] = None
    ground_truth_start: Optional[Tuple[float, float, float]] = None
    ground_truth_end: Optional[Tuple[float, float, float]] = None
    odom_error: Optional[float] = None

    @property
    def ok(self) -> bool:
        return self.success


@dataclass
class VelocityResult:
    success: bool = False
    measured_velocity: float = 0.0
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.success


# -- Internal helpers ──────────────────────────────────────────────────────


def _collect(node: Node, topic: str, msg_type, timeout_sec: float,
             qos=10, executor=None) -> list:
    """Subscribe, wait for *timeout_sec*, return collected messages."""
    msgs: list = []
    sub = node.create_subscription(msg_type, topic, msgs.append, qos)
    try:
        with ExecutorContext(node, executor) as ec:
            ec.wait(timeout_sec)
        return msgs
    finally:
        node.destroy_subscription(sub)


def _pos(msg) -> Tuple[float, float, float]:
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


def _dist3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _speed(msg) -> float:
    t = msg.twist.twist.linear
    return math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
