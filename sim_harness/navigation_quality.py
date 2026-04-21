# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0
"""Generic navigation quality tracking for integration tests.

Tracks heading stability, retry detection, distance budget, and collisions
during NavigateToPose maneuvers. Not tied to any specific vehicle or BT.
"""
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Iterable, Mapping, Optional, Union


@dataclass
class NavigationQualityTracker:
    """Accumulates quality metrics during a navigation maneuver.

    Call update() periodically with the robot's pose to accumulate metrics.
    Call assert_quality() at the end to check thresholds.
    """
    # Config
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    max_total_distance: float = 200.0
    spiral_threshold: float = 2 * math.pi     # 360 deg signed
    maneuver_threshold: float = 4 * math.pi   # 720 deg absolute
    retry_close_dist: float = 8.0
    retry_far_dist: float = 12.0

    # State (accumulated during update())
    total_distance: float = 0.0
    _prev_x: float = None
    _prev_y: float = None
    _prev_yaw: float = None
    _current_phase: str = None
    heading_signed: dict = field(default_factory=dict)
    heading_abs: dict = field(default_factory=dict)
    trajectory: list = field(default_factory=list)
    _was_close: bool = False
    retries_detected: int = 0
    collisions: list = field(default_factory=list)

    def update(self, x: float, y: float, yaw: float,
               phase: str, is_reverse: bool = False):
        """Update quality metrics with a new pose sample.

        Args:
            x, y, yaw: Robot pose in map frame.
            phase: Current phase label (e.g. 'Forward', 'Reverse').
            is_reverse: True if currently in reverse phase (for retry detection).
        """
        elapsed = time.monotonic()
        self.trajectory.append((x, y, elapsed, phase))

        # Distance accumulation
        if self._prev_x is not None:
            seg = math.sqrt((x - self._prev_x)**2 + (y - self._prev_y)**2)
            self.total_distance += seg
        self._prev_x, self._prev_y = x, y

        # Heading tracking (for forward loop detection)
        if phase != self._current_phase:
            self._current_phase = phase
            self._prev_yaw = yaw
        elif self._prev_yaw is not None:
            dyaw = yaw - self._prev_yaw
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            if phase not in self.heading_signed:
                self.heading_signed[phase] = 0.0
                self.heading_abs[phase] = 0.0
            self.heading_signed[phase] += dyaw
            self.heading_abs[phase] += abs(dyaw)
            self._prev_yaw = yaw

        # Retry detection (close→far oscillation during reverse)
        if is_reverse:
            d = math.sqrt((x - self.goal_x)**2 + (y - self.goal_y)**2)
            if d < self.retry_close_dist:
                self._was_close = True
            if self._was_close and d > self.retry_far_dist:
                self.retries_detected += 1
                self._was_close = False

    def add_collision(self, source: str, body1: str, body2: str):
        """Record a collision event."""
        entry = (source, body1, body2)
        if entry not in self.collisions:
            self.collisions.append(entry)

    def assert_no_spiral_loops(self):
        """Assert no phase has spiral or excessive heading changes."""
        for phase_name, signed in self.heading_signed.items():
            absolute = self.heading_abs.get(phase_name, 0.0)
            assert abs(signed) < self.spiral_threshold, (
                f"Spiral loop in {phase_name}: "
                f"net heading {math.degrees(signed):.0f}deg >= "
                f"{math.degrees(self.spiral_threshold):.0f}deg")
            assert absolute < self.maneuver_threshold, (
                f"Excessive maneuvering in {phase_name}: "
                f"total heading {math.degrees(absolute):.0f}deg >= "
                f"{math.degrees(self.maneuver_threshold):.0f}deg")

    def assert_no_retries(self):
        """Assert no close→far retry oscillations detected."""
        assert self.retries_detected == 0, (
            f"Reverse had {self.retries_detected} retry(ies) — "
            f"expected clean first-attempt success")

    def assert_distance_budget(self):
        """Assert total distance within budget."""
        assert self.total_distance <= self.max_total_distance, (
            f"Total distance {self.total_distance:.1f}m exceeds "
            f"{self.max_total_distance:.0f}m")

    def assert_no_collisions(self):
        """Assert no obstacle collisions detected."""
        if self.collisions:
            summary = '; '.join(
                f"{e[0]}: {e[1]} <-> {e[2]}" for e in self.collisions)
            assert False, (
                f"Vehicle collided with {len(self.collisions)} object(s): "
                f"{summary}")

    def print_summary(self):
        """Print quality metrics summary."""
        for phase_name in self.heading_signed:
            signed = self.heading_signed[phase_name]
            absolute = self.heading_abs.get(phase_name, 0.0)
            print(f"  {phase_name}: net={math.degrees(signed):.0f}deg, "
                  f"total={math.degrees(absolute):.0f}deg", flush=True)
        print(f"Quality: total distance = {self.total_distance:.1f}m "
              f"(limit: {self.max_total_distance:.0f}m)", flush=True)
        print(f"Quality: retries = {self.retries_detected}", flush=True)
        print(f"Quality: collisions = {len(self.collisions)}", flush=True)

    def cross_track_stats(self):
        """Compute cross-track oscillation stats for reverse trajectory."""
        reverse_pts = [(x, y) for x, y, _, p in self.trajectory
                       if p == 'Reverse']
        if len(reverse_pts) < 3:
            return 0, 0.0

        cross_tracks = []
        for x, y in reverse_pts:
            ct = ((x - self.goal_x) * (-math.sin(self.goal_yaw)) +
                  (y - self.goal_y) * math.cos(self.goal_yaw))
            cross_tracks.append(ct)

        sign_changes = sum(
            1 for i in range(1, len(cross_tracks))
            if cross_tracks[i] * cross_tracks[i-1] < 0)
        max_amplitude = max(abs(ct) for ct in cross_tracks) if cross_tracks else 0.0
        return sign_changes, max_amplitude


PhaseMapper = Union[Mapping[str, str], Callable[[str], Optional[str]]]
CollisionSources = Union[
    Mapping[str, str],                 # {label: topic}
    Iterable[str],                     # [topic, ...] (label derived from topic)
    Iterable[tuple],                   # [(label, topic), ...]
]


def monitor_phases_from_bt_log(
    node,
    topic: str,
    phase_mapper: PhaseMapper,
    *,
    phases_seen: Optional[set] = None,
    on_phase: Optional[Callable[[str, object], None]] = None,
    qos: int = 10,
):
    """Subscribe to a ``nav2_msgs/BehaviorTreeLog`` topic and report phase transitions.

    Args:
        node: ROS2 node to create the subscription on.
        topic: BT log topic (e.g. ``'/robot/behavior_tree_log'``).
        phase_mapper: Either a dict mapping BT node name to phase label,
            or a callable ``(node_name) -> Optional[str]`` (return ``None`` to skip).
        phases_seen: Optional set populated with newly observed phase labels.
        on_phase: Optional callback invoked as ``on_phase(phase_label, event)``
            the first time each phase label is seen.
        qos: Subscription QoS depth (default 10).

    Returns:
        The subscription (keep a reference to prevent GC).
    """
    from nav2_msgs.msg import BehaviorTreeLog

    if callable(phase_mapper):
        lookup = phase_mapper
    else:
        mapping = dict(phase_mapper)
        lookup = mapping.get

    seen = phases_seen if phases_seen is not None else set()

    def bt_log_cb(msg):
        for event in msg.event_log:
            phase = lookup(event.node_name)
            if phase is None:
                continue
            if phase not in seen:
                seen.add(phase)
                if on_phase is not None:
                    on_phase(phase, event)

    return node.create_subscription(BehaviorTreeLog, topic, bt_log_cb, qos)


def _normalize_collision_sources(sources: CollisionSources):
    """Normalize to a list of (label, topic) pairs."""
    if isinstance(sources, Mapping):
        return list(sources.items())
    items = []
    for s in sources:
        if isinstance(s, str):
            label = s.strip('/').split('/')[-1]
            for suffix in ('_contacts', '_contact'):
                if label.endswith(suffix):
                    label = label[: -len(suffix)]
                    break
            items.append((label or s, s))
        else:
            items.append(tuple(s))
    return items


def monitor_collisions(
    node,
    sources: CollisionSources,
    *,
    tracker: Optional[NavigationQualityTracker] = None,
    on_collision: Optional[Callable[[str, str, str], None]] = None,
    ignore_substrings: Iterable[str] = ('ground',),
    qos: int = 10,
):
    """Subscribe to ``ros_gz_interfaces/Contacts`` topics and record collisions.

    Args:
        node: ROS2 node to create subscriptions on.
        sources: Either a dict ``{label: topic}``, a list of topic names,
            or a list of ``(label, topic)`` pairs. When given plain topic
            names, the label is derived from the final path component with
            any trailing ``_contacts`` / ``_contact`` suffix stripped.
        tracker: Optional :class:`NavigationQualityTracker` — new collisions
            are recorded via ``tracker.add_collision``.
        on_collision: Optional callback ``(label, body1, body2)``.
        ignore_substrings: Collision names containing any of these substrings
            (case-insensitive) are skipped. Default filters ``ground`` noise.
        qos: Subscription QoS depth (default 10).

    Returns:
        List of subscriptions (keep references to prevent GC). Sources that
        fail to subscribe are logged via the node's logger and omitted.
    """
    from ros_gz_interfaces.msg import Contacts

    items = _normalize_collision_sources(sources)
    ignore = tuple(sub.lower() for sub in ignore_substrings)

    def make_cb(label):
        def cb(msg):
            for c in msg.contacts:
                n1 = c.collision1.name
                n2 = c.collision2.name
                lower1, lower2 = n1.lower(), n2.lower()
                if any(sub in lower1 or sub in lower2 for sub in ignore):
                    continue
                if tracker is not None:
                    tracker.add_collision(label, n1, n2)
                if on_collision is not None:
                    on_collision(label, n1, n2)
        return cb

    subs = []
    for label, topic in items:
        try:
            subs.append(node.create_subscription(
                Contacts, topic, make_cb(label), qos))
        except Exception as e:
            logger = node.get_logger() if hasattr(node, 'get_logger') else None
            if logger is not None:
                logger.warning(f"Failed to subscribe to {topic}: {e}")
            else:
                print(f"Warning: Failed to subscribe to {topic}: {e}",
                      flush=True)
    return subs
