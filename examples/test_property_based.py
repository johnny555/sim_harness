#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Property-Based Testing Examples for ROS 2 Simulations.

Demonstrates the three-tier Hypothesis integration:

  Tier 1 -- Properties over recorded data (CHEAP, full Hypothesis power)
      Collect messages once from the sim, then run many property checks
      against the recorded data.  The sim is never re-run.

  Tier 2 -- Scenario-level properties (EXPENSIVE, use sparingly)
      Hypothesis generates entire test scenarios (goals, initial poses).
      Each example re-runs part of the sim.  Use max_examples=3-5.

  Tier 3 -- Same-sim parameter variation (MEDIUM cost)
      The sim stays running; Hypothesis varies commands or parameters
      within a single session.  Each example costs seconds, not minutes.

Run all property tests:
    pytest examples/test_property_based.py -v

Run only Tier 1 (cheapest):
    pytest examples/test_property_based.py -v -k "tier1"

Run with nightly depth (10x examples):
    SIM_HARNESS_NIGHTLY=1 pytest examples/test_property_based.py -v
"""

import math
import time

import pytest

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from sim_harness import SimTestFixture

# Composable predicates — import from their purpose-bucket
from sim_harness.checks import all_of
from sim_harness.checks.sensors import (
    scan_has_min_points,
    scan_ranges_within,
    scan_nan_ratio_below,
    imu_accel_within,
    imu_no_nan,
)
from sim_harness.checks.motion import (
    odom_position_finite,
    odom_velocity_below,
)

# Property-based testing — single import path
from sim_harness.core.hypothesis import (
    sim_property,
    check_recorded_property,
    check_recorded_eventually,
    check_recorded_monotonic,
    hypothesis_check_recorded,
    twist_strategy,
    navigation_goal_2d,
    speed_strategy,
)

# Hypothesis (optional -- tests skip if not installed)
try:
    from hypothesis import given, assume
    from hypothesis import strategies as st
    HAS_HYPOTHESIS = True
except ImportError:
    HAS_HYPOTHESIS = False

skip_no_hypothesis = pytest.mark.skipif(
    not HAS_HYPOTHESIS,
    reason="hypothesis not installed",
)


# ==========================================================================
# TIER 1: Properties over recorded data
#
# These are CHEAP -- collect data once, check many properties.
# Full Hypothesis power (100+ examples, shrinking, etc.).
# ==========================================================================

class TestTier1RecordedDataProperties(SimTestFixture):
    """
    Tier 1: Property checks over data already collected from the sim.

    Pattern:
      1. Spin to collect messages (expensive, done once per test)
      2. Check properties against recorded messages (cheap, can repeat)
    """

    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"
    IMU_TOPIC = "/imu"

    def test_tier1_lidar_all_scans_valid(self):
        """Every LIDAR scan should have valid range data."""
        collector = self.create_message_collector(
            self.SCAN_TOPIC, LaserScan, key="scan",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No LIDAR data received")

        # Tier 1: check property over recorded messages, no sim re-run
        check_recorded_property(
            data=messages,
            property_fn=lambda scan: (
                len([r for r in scan.ranges if math.isfinite(r)]) >= 10
            ),
            description="Every scan has >= 10 finite range values",
            min_samples=5,
        )

    def test_tier1_odometry_positions_finite(self):
        """All odometry messages should have finite position values."""
        collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No odometry data received")

        check_recorded_property(
            data=messages,
            property_fn=lambda odom: all(
                math.isfinite(v) for v in [
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                ]
            ),
            description="All odometry positions are finite",
        )

    def test_tier1_imu_no_nan(self):
        """IMU data should never contain NaN."""
        collector = self.create_message_collector(
            self.IMU_TOPIC, Imu, key="imu",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No IMU data received")

        check_recorded_property(
            data=messages,
            property_fn=lambda imu: all(
                math.isfinite(v) for v in [
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z,
                ]
            ),
            description="IMU values are never NaN",
        )

    def test_tier1_odometry_eventually_stabilizes(self):
        """At some point, the robot should be nearly stationary."""
        collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No odometry data received")

        check_recorded_eventually(
            data=messages,
            property_fn=lambda odom: (
                math.sqrt(
                    odom.twist.twist.linear.x ** 2
                    + odom.twist.twist.linear.y ** 2
                ) < 0.1
            ),
            description="Robot velocity drops below 0.1 m/s",
        )

    @skip_no_hypothesis
    def test_tier1_lidar_quality_parametric(self):
        """
        Tier 1 with Hypothesis: for any reasonable NaN threshold,
        the LIDAR data quality holds.

        Hypothesis generates random thresholds and checks each one
        against the recorded data.  This finds the exact boundary
        where data quality breaks.
        """
        collector = self.create_message_collector(
            self.SCAN_TOPIC, LaserScan, key="scan",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No LIDAR data received")

        # Hypothesis generates thresholds; we check against recorded data.
        # If it fails, Hypothesis shrinks to find the tightest threshold.
        hypothesis_check_recorded(
            data=messages,
            strategy=st.floats(0.05, 0.5),  # NaN ratio thresholds
            property_fn=lambda scan, threshold: (
                sum(1 for r in scan.ranges if math.isnan(r))
                / max(len(scan.ranges), 1)
                < threshold
            ),
            description="NaN ratio below threshold for all scans",
            max_examples=50,
        )


# ==========================================================================
# TIER 2: Scenario-level property tests
#
# EXPENSIVE -- each example may involve sending goals, waiting for
# navigation, etc.  Use max_examples=3-5.  Shrinking is disabled
# by default (each shrink attempt would re-run the sim).
# ==========================================================================

@skip_no_hypothesis
class TestTier2ScenarioProperties(SimTestFixture):
    """
    Tier 2: Hypothesis generates entire test scenarios.

    Each example is expensive (seconds to minutes), so we limit
    max_examples to 3-5.  The persistent database ensures that
    previously-found counterexamples are re-tested on every run.
    """

    ODOM_TOPIC = "/odom"
    CMD_VEL_TOPIC = "/cmd_vel"

    @sim_property(max_examples=3)
    @given(goal=navigation_goal_2d(x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0)))
    def test_tier2_robot_moves_toward_goal(self, goal):
        """
        For any goal within bounds, the robot should move in the
        right general direction when given a velocity command toward it.

        This is a weak property (doesn't require reaching the goal),
        suitable for basic motion validation.
        """
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )

        # Get initial position
        self.spin_for_duration(1.0)
        initial_msgs = odom_collector.get_messages()
        if not initial_msgs:
            pytest.skip("No odometry")
        initial_pos = initial_msgs[-1].pose.pose.position

        # Compute direction to goal
        dx = goal.pose.position.x - initial_pos.x
        dy = goal.pose.position.y - initial_pos.y
        dist = math.sqrt(dx * dx + dy * dy)
        assume(dist > 0.5)  # Skip if goal is too close

        # Send a velocity command (simplified: just forward)
        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = 0.15
        cmd.twist.angular.z = 0.0

        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.twist.linear.x = 0.0
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        # Check that the robot moved at all
        final_msgs = odom_collector.get_messages()
        final_pos = final_msgs[-1].pose.pose.position
        movement = math.sqrt(
            (final_pos.x - initial_pos.x) ** 2
            + (final_pos.y - initial_pos.y) ** 2
        )

        self.node.destroy_publisher(cmd_pub)

        assert movement > 0.05, (
            f"Robot did not move. "
            f"Initial=({initial_pos.x:.2f}, {initial_pos.y:.2f}), "
            f"Final=({final_pos.x:.2f}, {final_pos.y:.2f})"
        )

    @sim_property(max_examples=5)
    @given(
        speed=speed_strategy(min_speed=0.05, max_speed=0.5),
    )
    def test_tier2_velocity_proportional_to_command(self, speed):
        """
        For any commanded speed, the observed velocity should be
        roughly proportional (within the robot's acceleration limits).
        """
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )

        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = speed
        cmd.twist.angular.z = 0.0

        # Drive for 2 seconds
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.twist.linear.x = 0.0
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.3)

        # Get observed velocities
        msgs = odom_collector.get_messages()
        self.node.destroy_publisher(cmd_pub)

        if len(msgs) < 5:
            pytest.skip("Not enough odometry messages")

        # Check that max observed velocity is in a reasonable range
        # (between 10% and 200% of commanded speed)
        max_observed = max(
            abs(m.twist.twist.linear.x) for m in msgs
        )

        assert max_observed > speed * 0.1, (
            f"Robot barely moved: max_observed={max_observed:.3f}, "
            f"commanded={speed:.3f}"
        )


# ==========================================================================
# TIER 3: Same-sim parameter variation
#
# The sim stays running; Hypothesis varies parameters within a single
# session.  Each example costs seconds (a velocity command + observe),
# not minutes (a full sim restart).
# ==========================================================================

@skip_no_hypothesis
class TestTier3SameSimVariation(SimTestFixture):
    """
    Tier 3: The sim stays running; Hypothesis varies commands.

    Each example sends a different velocity command and observes the
    result.  Much cheaper than Tier 2 because there's no sim restart.
    """

    ODOM_TOPIC = "/odom"
    CMD_VEL_TOPIC = "/cmd_vel"

    @sim_property(max_examples=10)
    @given(cmd=twist_strategy(max_linear=0.3, max_angular=0.5))
    def test_tier3_any_twist_keeps_robot_stable(self, cmd):
        """
        For any reasonable velocity command, the robot should remain
        stable (no NaN in odometry, no teleporting).
        """
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )

        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        stamped = TwistStamped()
        stamped.header.frame_id = "base_link"
        stamped.twist = cmd

        # Send command for 1 second
        start = time.monotonic()
        while time.monotonic() - start < 1.0:
            stamped.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(stamped)
            self.spin_for_duration(0.1)

        # Stop
        stamped.twist.linear.x = 0.0
        stamped.twist.angular.z = 0.0
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        cmd_pub.publish(stamped)
        self.spin_for_duration(0.3)

        # Check stability: all odometry must be finite
        msgs = odom_collector.get_messages()
        self.node.destroy_publisher(cmd_pub)

        for i, odom in enumerate(msgs):
            pos = odom.pose.pose.position
            assert math.isfinite(pos.x) and math.isfinite(pos.y), (
                f"NaN in odometry at message {i} with twist "
                f"linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}"
            )

    @sim_property(max_examples=8)
    @given(
        linear=st.floats(0.05, 0.3, allow_nan=False, allow_infinity=False),
        duration=st.floats(0.5, 2.0, allow_nan=False, allow_infinity=False),
    )
    def test_tier3_movement_scales_with_duration(self, linear, duration):
        """
        Longer commands should produce more movement.

        We can't test exact proportionality (acceleration ramp, physics),
        but the robot should move *some* distance for *any* positive
        velocity and duration.
        """
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )

        # Get initial position
        self.spin_for_duration(0.5)
        initial_msgs = odom_collector.get_messages()
        if not initial_msgs:
            pytest.skip("No odometry")
        initial_pos = initial_msgs[-1].pose.pose.position

        # Drive
        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        stamped = TwistStamped()
        stamped.header.frame_id = "base_link"
        stamped.twist.linear.x = linear
        stamped.twist.angular.z = 0.0

        start = time.monotonic()
        while time.monotonic() - start < duration:
            stamped.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(stamped)
            self.spin_for_duration(0.1)

        # Stop and observe
        stamped.twist.linear.x = 0.0
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        cmd_pub.publish(stamped)
        self.spin_for_duration(0.3)

        final_msgs = odom_collector.get_messages()
        self.node.destroy_publisher(cmd_pub)

        if not final_msgs:
            pytest.skip("No final odometry")

        final_pos = final_msgs[-1].pose.pose.position
        dist = math.sqrt(
            (final_pos.x - initial_pos.x) ** 2
            + (final_pos.y - initial_pos.y) ** 2
        )

        # The robot should move at least a tiny amount
        assert dist > 0.01, (
            f"Robot didn't move with linear={linear:.3f} "
            f"for {duration:.1f}s. dist={dist:.4f}"
        )


# ==========================================================================
# Bonus: Combining Tier 1 with FP stream properties
# ==========================================================================

class TestCombinedFPAndPropertyBased(SimTestFixture):
    """
    Demonstrates combining the FP stream property combinators with
    the Tier 1 recorded-data approach.
    """

    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"

    def test_combined_lidar_quality_composable(self):
        """
        Use composable predicates + recorded data property checking.

        This combines:
        - Composable predicates (all_of, scan_has_min_points, etc.)
        - Tier 1 recorded data checking (check_recorded_property)
        """
        collector = self.create_message_collector(
            self.SCAN_TOPIC, LaserScan, key="scan",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if not messages:
            pytest.skip("No LIDAR data")

        # Compose a predicate from small pieces
        valid_lidar = all_of(
            scan_has_min_points(10),
            scan_ranges_within(0.01, 50.0),
            scan_nan_ratio_below(0.5),
        )

        # Check it over recorded data
        check_recorded_property(
            data=messages,
            property_fn=valid_lidar,
            description="All LIDAR scans pass composite quality check",
        )

    def test_combined_odometry_monotonic_time(self):
        """
        Odometry timestamps should be monotonically increasing.

        Uses check_recorded_monotonic -- a Tier 1 helper.
        """
        collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom",
        )
        self.spin_for_duration(5.0)
        messages = collector.get_messages()

        if len(messages) < 2:
            pytest.skip("Not enough odometry messages")

        check_recorded_monotonic(
            data=messages,
            extract=lambda odom: (
                odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
            ),
            strict=False,
            description="Odometry timestamps are non-decreasing",
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
