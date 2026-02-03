# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Hypothesis integration for property-based simulation testing.

Optional module — requires ``pip install hypothesis`` (or
``pip install sim_harness[hypothesis]``).

Provides:

- ``sim_property`` decorator with sim-friendly defaults
- ``PropertyFailure`` for counterexample reporting
- Tier 1 recorded-data helpers (``check_recorded_property``, etc.)
- ROS message strategies (``point_strategy``, ``twist_strategy``, etc.)

Three-tier usage::

    # Tier 1 — cheap: check properties over already-collected data
    check_recorded_property(messages, lambda msg: has_valid_ranges(msg))

    # Tier 2 — expensive: Hypothesis generates full scenarios
    @sim_property(max_examples=3)
    @given(goal=navigation_goal_2d(...))
    def test_reaches_goals(self, goal): ...

    # Tier 3 — medium: vary parameters within a running sim
    @sim_property(max_examples=10)
    @given(cmd=twist_strategy(max_linear=0.5))
    def test_velocity_commands(self, cmd): ...
"""

# Re-export everything from the two implementation modules so users
# have a single import path: ``from sim_harness.core.hypothesis import ...``

from sim_harness.core.sim_property import (  # noqa: F401
    sim_property,
    PropertyFailure,
    check_recorded_property,
    check_recorded_eventually,
    check_recorded_monotonic,
    hypothesis_check_recorded,
)

from sim_harness.core.strategies import (  # noqa: F401
    point_strategy,
    vector3_strategy,
    quaternion_strategy,
    yaw_quaternion_strategy,
    pose_strategy,
    twist_strategy,
    twist_strategy_3d,
    navigation_goal_2d,
    waypoints_strategy,
    duration_strategy,
    rate_strategy,
    threshold_strategy,
    angle_strategy,
    speed_strategy,
    distance_strategy,
)
