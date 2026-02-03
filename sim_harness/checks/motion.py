# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Motion checks — "Does the robot move correctly?"

Validates vehicle movement, velocity, position, orientation, and
stationarity. Includes ground-truth comparison via Gazebo and
composable odometry predicates for custom checks.

Example::

    from sim_harness.checks.motion import (
        assert_vehicle_moved,
        assert_vehicle_stationary,
        odom_velocity_below,
    )

    class TestMotion(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_moves_forward(self):
            result = assert_vehicle_moved(
                self.node, 'robot_01', min_distance=1.0, velocity=0.5)
            assert result.success, result.details

        def test_starts_stationary(self):
            assert assert_vehicle_stationary(self.node, 'robot_01')
"""

# --- Vehicle motion assertions ---
from sim_harness.primitives.vehicle_assertions import (  # noqa: F401
    MovementResult,
    VelocityResult,
    assert_vehicle_moved,
    assert_vehicle_moved_with_ground_truth,
    assert_vehicle_stationary,
    assert_vehicle_velocity,
    assert_vehicle_in_region,
    assert_vehicle_orientation,
)

# --- Composable odometry predicates ---
from sim_harness.core.predicates import (  # noqa: F401
    odom_position_finite,
    odom_velocity_below,
    odom_in_region,
)
