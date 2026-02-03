# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Hypothesis strategies for generating ROS 2 messages and test parameters.

Provides type-safe strategies for common ROS message types, designed
for property-based testing of robotic systems.  Strategies generate
valid, physically plausible messages -- not random garbage.

Three tiers of use:

  Tier 1 (Recorded Data):
      Use parameter strategies (thresholds, bounds) to test properties
      over already-collected messages.  Cheap -- full Hypothesis power.

  Tier 2 (Scenario-Level):
      Use goal/pose/twist strategies to generate sim scenarios.
      Expensive -- use max_examples=3-5.

  Tier 3 (Same-Sim Variation):
      Use cmd_vel/timing strategies for in-sim parameter sweeps.
      Medium cost -- the sim stays running between examples.

Example::

    from hypothesis import given
    from sim_harness.core.strategies import twist_strategy, navigation_goal_2d
    from sim_harness.core.sim_property import sim_property

    class TestNavigation(SimTestFixture):
        @sim_property(max_examples=5)
        @given(goal=navigation_goal_2d(x_bounds=(-3, 3), y_bounds=(-3, 3)))
        def test_reaches_goals(self, goal):
            result = assert_reaches_goal(self.node, self.executor, goal, 30.0)
            assert result.reached
"""

import math
from typing import Tuple, Optional, List

try:
    from hypothesis import strategies as st
    from hypothesis.strategies import SearchStrategy
    HAS_HYPOTHESIS = True
except ImportError:
    HAS_HYPOTHESIS = False

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from std_msgs.msg import Header


def _require_hypothesis():
    """Raise ImportError if hypothesis is not installed."""
    if not HAS_HYPOTHESIS:
        raise ImportError(
            "hypothesis is required for property-based testing. "
            "Install it with: pip install hypothesis"
        )


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

def point_strategy(
    x_bounds: Tuple[float, float] = (-10.0, 10.0),
    y_bounds: Tuple[float, float] = (-10.0, 10.0),
    z_bounds: Tuple[float, float] = (0.0, 0.0),
) -> 'SearchStrategy[Point]':
    """Generate geometry_msgs/Point within given bounds."""
    _require_hypothesis()
    return st.builds(
        Point,
        x=st.floats(*x_bounds, allow_nan=False, allow_infinity=False),
        y=st.floats(*y_bounds, allow_nan=False, allow_infinity=False),
        z=st.floats(*z_bounds, allow_nan=False, allow_infinity=False),
    )


def vector3_strategy(
    min_val: float = -10.0,
    max_val: float = 10.0,
) -> 'SearchStrategy[Vector3]':
    """Generate geometry_msgs/Vector3 with bounded components."""
    _require_hypothesis()
    component = st.floats(min_val, max_val, allow_nan=False, allow_infinity=False)
    return st.builds(Vector3, x=component, y=component, z=component)


def quaternion_strategy() -> 'SearchStrategy[Quaternion]':
    """
    Generate valid unit quaternions (norm = 1.0).

    Uses the 4-sphere normalization approach to ensure all generated
    quaternions are valid rotations.
    """
    _require_hypothesis()

    @st.composite
    def make_quaternion(draw):
        x = draw(st.floats(-1.0, 1.0, allow_nan=False, allow_infinity=False))
        y = draw(st.floats(-1.0, 1.0, allow_nan=False, allow_infinity=False))
        z = draw(st.floats(-1.0, 1.0, allow_nan=False, allow_infinity=False))
        w = draw(st.floats(-1.0, 1.0, allow_nan=False, allow_infinity=False))

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-10:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        return Quaternion(x=x / norm, y=y / norm, z=z / norm, w=w / norm)

    return make_quaternion()


def yaw_quaternion_strategy(
    yaw_bounds: Tuple[float, float] = (-math.pi, math.pi),
) -> 'SearchStrategy[Quaternion]':
    """
    Generate quaternions representing 2D rotations (yaw only).

    More useful than ``quaternion_strategy`` for ground robots that
    only rotate around the Z axis.
    """
    _require_hypothesis()

    @st.composite
    def make_yaw_quaternion(draw):
        yaw = draw(st.floats(*yaw_bounds, allow_nan=False, allow_infinity=False))
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

    return make_yaw_quaternion()


def pose_strategy(
    x_bounds: Tuple[float, float] = (-10.0, 10.0),
    y_bounds: Tuple[float, float] = (-10.0, 10.0),
    z_bounds: Tuple[float, float] = (0.0, 0.0),
    yaw_only: bool = True,
) -> 'SearchStrategy[Pose]':
    """
    Generate geometry_msgs/Pose messages.

    Args:
        x_bounds: (min, max) for x position
        y_bounds: (min, max) for y position
        z_bounds: (min, max) for z position
        yaw_only: If True (default), only generate yaw rotations.
                  More realistic for ground robots.
    """
    _require_hypothesis()
    pos = point_strategy(x_bounds, y_bounds, z_bounds)
    orient = yaw_quaternion_strategy() if yaw_only else quaternion_strategy()
    return st.builds(Pose, position=pos, orientation=orient)


# ---------------------------------------------------------------------------
# Velocity commands
# ---------------------------------------------------------------------------

def twist_strategy(
    max_linear: float = 1.0,
    max_angular: float = 1.0,
) -> 'SearchStrategy[Twist]':
    """
    Generate geometry_msgs/Twist commands for ground robots.

    Only generates linear.x (forward/backward) and angular.z (yaw)
    -- the two degrees of freedom of a differential-drive robot.

    Args:
        max_linear: Maximum linear velocity (m/s)
        max_angular: Maximum angular velocity (rad/s)
    """
    _require_hypothesis()
    return st.builds(
        Twist,
        linear=st.builds(
            Vector3,
            x=st.floats(-max_linear, max_linear,
                         allow_nan=False, allow_infinity=False),
            y=st.just(0.0),
            z=st.just(0.0),
        ),
        angular=st.builds(
            Vector3,
            x=st.just(0.0),
            y=st.just(0.0),
            z=st.floats(-max_angular, max_angular,
                         allow_nan=False, allow_infinity=False),
        ),
    )


def twist_strategy_3d(
    max_linear: float = 1.0,
    max_angular: float = 1.0,
) -> 'SearchStrategy[Twist]':
    """Generate full 6-DOF Twist commands (drones, underwater vehicles)."""
    _require_hypothesis()
    return st.builds(
        Twist,
        linear=vector3_strategy(-max_linear, max_linear),
        angular=vector3_strategy(-max_angular, max_angular),
    )


# ---------------------------------------------------------------------------
# Navigation goals
# ---------------------------------------------------------------------------

def navigation_goal_2d(
    x_bounds: Tuple[float, float] = (-5.0, 5.0),
    y_bounds: Tuple[float, float] = (-5.0, 5.0),
    frame_id: str = "map",
) -> 'SearchStrategy[PoseStamped]':
    """
    Generate navigation goals for 2D ground robots.

    Creates PoseStamped messages suitable for Nav2 NavigateToPose goals.
    Position is constrained to bounds (typically the sim world limits),
    and orientation is yaw-only.

    Args:
        x_bounds: (min, max) x position in meters
        y_bounds: (min, max) y position in meters
        frame_id: Frame ID for the goal (default: "map")
    """
    _require_hypothesis()

    @st.composite
    def make_goal(draw):
        p = draw(pose_strategy(x_bounds, y_bounds, (0.0, 0.0), yaw_only=True))
        return PoseStamped(header=Header(frame_id=frame_id), pose=p)

    return make_goal()


def waypoints_strategy(
    x_bounds: Tuple[float, float] = (-5.0, 5.0),
    y_bounds: Tuple[float, float] = (-5.0, 5.0),
    min_count: int = 2,
    max_count: int = 5,
    frame_id: str = "map",
) -> 'SearchStrategy[List[PoseStamped]]':
    """
    Generate a list of waypoints for path-following tests.

    Args:
        x_bounds: (min, max) x position
        y_bounds: (min, max) y position
        min_count: Minimum number of waypoints
        max_count: Maximum number of waypoints
        frame_id: Frame ID
    """
    _require_hypothesis()
    goal = navigation_goal_2d(x_bounds, y_bounds, frame_id)
    return st.lists(goal, min_size=min_count, max_size=max_count)


# ---------------------------------------------------------------------------
# Parameter strategies (Tier 1 and Tier 3)
# ---------------------------------------------------------------------------

def duration_strategy(
    min_sec: float = 0.1,
    max_sec: float = 30.0,
) -> 'SearchStrategy[float]':
    """Generate reasonable duration values in seconds."""
    _require_hypothesis()
    return st.floats(min_sec, max_sec, allow_nan=False, allow_infinity=False)


def rate_strategy(
    min_hz: float = 1.0,
    max_hz: float = 100.0,
) -> 'SearchStrategy[float]':
    """Generate publish rate values in Hz."""
    _require_hypothesis()
    return st.floats(min_hz, max_hz, allow_nan=False, allow_infinity=False)


def threshold_strategy(
    min_val: float = 0.0,
    max_val: float = 1.0,
) -> 'SearchStrategy[float]':
    """Generate threshold values (tolerance, confidence, ratio, etc.)."""
    _require_hypothesis()
    return st.floats(min_val, max_val, allow_nan=False, allow_infinity=False)


def angle_strategy(
    min_rad: float = -math.pi,
    max_rad: float = math.pi,
) -> 'SearchStrategy[float]':
    """Generate angle values in radians."""
    _require_hypothesis()
    return st.floats(min_rad, max_rad, allow_nan=False, allow_infinity=False)


def speed_strategy(
    min_speed: float = 0.0,
    max_speed: float = 2.0,
) -> 'SearchStrategy[float]':
    """Generate speed values in m/s."""
    _require_hypothesis()
    return st.floats(min_speed, max_speed, allow_nan=False, allow_infinity=False)


def distance_strategy(
    min_dist: float = 0.0,
    max_dist: float = 50.0,
) -> 'SearchStrategy[float]':
    """Generate distance values in meters."""
    _require_hypothesis()
    return st.floats(min_dist, max_dist, allow_nan=False, allow_infinity=False)
