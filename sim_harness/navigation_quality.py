# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0
"""Generic navigation quality tracking for integration tests.

Tracks heading stability, retry detection, distance budget, and collisions
during NavigateToPose maneuvers. Not tied to any specific vehicle or BT.
"""
import math
import time
from dataclasses import dataclass, field


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
