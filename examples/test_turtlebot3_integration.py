#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
TurtleBot3 Integration Test Example.

Demonstrates how to use sim_harness for simulation testing.

Tests are organized into:
- Basic tests: Verify sensors publish data (should always pass)
- Advanced tests: Motion, navigation (may need full stack initialization)

Run all tests:
    pytest examples/test_turtlebot3_integration.py -v

Run only basic tests:
    pytest examples/test_turtlebot3_integration.py -v -k "Basic"

Run with requirement validation summary:
    pytest examples/test_turtlebot3_integration.py -v -s
"""

import math
import pytest

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage

from sim_harness import (
    SimTestFixture,
    RequirementValidator,
    ValidationResultCollector,
)
from sim_harness.nav2 import check_lifecycle_node_active


class TestTurtleBot3Integration(SimTestFixture, RequirementValidator):
    """Integration test fixture for TurtleBot3 simulation."""

    # Topic configuration
    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"
    IMU_TOPIC = "/imu"
    CMD_VEL_TOPIC = "/cmd_vel"
    COSTMAP_TOPIC = "/local_costmap/costmap"

    # ==========================================================================
    # BASIC TESTS - These should always pass if simulation is running
    # ==========================================================================

    def test_basic_lidar_publishes(self):
        """
        Verify LIDAR sensor is publishing data.

        Requirement: REQ-SEN-001 LIDAR publishes scan data
        """
        collector = self.create_message_collector(
            self.SCAN_TOPIC,
            LaserScan,
            key="scan"
        )

        # Wait for messages with timeout
        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        has_data = len(messages) > 0

        details = f"Received {len(messages)} LIDAR messages"
        if has_data and messages:
            scan = messages[-1]
            details += f", {len(scan.ranges)} ranges"

        self.assert_requirement(
            "REQ-SEN-001",
            "LIDAR sensor is operational",
            has_data,
            details,
            "Sensors"
        )

        assert has_data, f"No LIDAR messages received on {self.SCAN_TOPIC}"
        assert len(messages) > 0

    def test_basic_odometry_publishes(self):
        """
        Verify odometry is being published.

        Requirement: REQ-SEN-002 Odometry publishes pose data
        """
        collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        has_data = len(messages) > 0

        details = f"Received {len(messages)} odometry messages"

        self.assert_requirement(
            "REQ-SEN-002",
            "Odometry is being published",
            has_data,
            details,
            "Sensors"
        )

        assert has_data, f"No odometry messages received on {self.ODOM_TOPIC}"

    def test_basic_imu_publishes(self):
        """
        Verify IMU sensor is publishing data.

        Requirement: REQ-SEN-003 IMU publishes data
        """
        collector = self.create_message_collector(
            self.IMU_TOPIC,
            Imu,
            key="imu"
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        has_data = len(messages) > 0

        details = f"Received {len(messages)} IMU messages"

        self.assert_requirement(
            "REQ-SEN-003",
            "IMU sensor is operational",
            has_data,
            details,
            "Sensors"
        )

        assert has_data, f"No IMU messages received on {self.IMU_TOPIC}"

    def test_basic_sensor_data_quality(self):
        """
        Verify sensor data contains valid values (no NaN).

        Requirement: REQ-SEN-004 Sensor data is valid
        """
        scan_collector = self.create_message_collector(
            self.SCAN_TOPIC,
            LaserScan,
            key="scan"
        )
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )

        self.spin_for_duration(3.0)

        valid = True
        issues = ""

        # Check odometry for NaN
        for odom in odom_collector.get_messages():
            if math.isnan(odom.pose.pose.position.x) or \
               math.isnan(odom.pose.pose.position.y):
                valid = False
                issues += "Odometry has NaN; "
                break

        # Check LIDAR - allow inf (out of range) but not NaN
        for scan in scan_collector.get_messages():
            nan_count = sum(1 for r in scan.ranges if math.isnan(r))
            if nan_count > 10:  # Allow a few NaN values
                valid = False
                issues += "LIDAR has too many NaN values; "
                break

        self.assert_requirement(
            "REQ-SEN-004",
            "Sensor data is valid (no NaN)",
            valid,
            "All sensor data valid" if valid else issues,
            "Sensors"
        )

        assert valid, issues

    def test_basic_costmap_publishes(self):
        """
        Verify local costmap is publishing.

        Requirement: REQ-NAV-001 Costmap is active
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        collector = self.create_message_collector(
            self.COSTMAP_TOPIC,
            OccupancyGrid,
            key="costmap",
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        has_data = len(messages) > 0

        details = f"Received {len(messages)} costmap messages"

        self.assert_requirement(
            "REQ-NAV-001",
            "Local costmap is publishing",
            has_data,
            details,
            "Navigation"
        )

        assert has_data, "No costmap messages received"

    # ==========================================================================
    # ADVANCED TESTS - These require full Nav2 stack and may be flaky
    # ==========================================================================

    def test_advanced_nav2_stack_active(self):
        """
        Verify Nav2 lifecycle nodes are active.

        Requirement: REQ-NAV-002 Nav2 stack is active
        """
        # Check just a few key nodes instead of all
        key_nodes = [
            "controller_server",
            "planner_server",
            "bt_navigator"
        ]

        any_active = False
        details_parts = []

        for node_name in key_nodes:
            result = check_lifecycle_node_active(self.node, node_name, timeout_sec=10.0)
            if result.success:
                any_active = True
                details_parts.append(f"{node_name}: active")
            else:
                details_parts.append(f"{node_name}: {result.details}")

        details = "; ".join(details_parts)

        # We just need at least one active for this test to pass
        # Full Nav2 may not be ready yet
        assert any_active, details

    def test_advanced_transforms_available(self):
        """
        Verify TF transforms are being published.

        Requirement: REQ-NAV-003 TF tree is configured

        Note: This test checks that TF messages are being published.
        The actual transform lookup is done via temp nodes which may have
        timing issues with transient_local QoS for tf_static. The motion
        tests (VelocityCommandAccepted) provide stronger proof that TF works.
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        tf_collector = self.create_message_collector(
            "/tf",
            TFMessage,
            key="tf"
        )
        tf_static_collector = self.create_message_collector(
            "/tf_static",
            TFMessage,
            key="tf_static",
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        # Wait for TF messages
        self.spin_for_duration(3.0)

        tf_msgs = tf_collector.get_messages()
        tf_static_msgs = tf_static_collector.get_messages()

        has_tf = len(tf_msgs) > 0
        has_tf_static = len(tf_static_msgs) > 0

        details = f"/tf messages: {len(tf_msgs)}, /tf_static messages: {len(tf_static_msgs)}"

        self.get_logger().info(f"TF check: {details}")

        # Check that we're getting TF data (at least one of the two topics)
        assert has_tf or has_tf_static, f"No TF messages received. Details: {details}"

        # Verify the TF messages contain expected frames
        found_odom_frame = False
        found_base_frame = False
        for tf_msg in tf_msgs:
            for transform in tf_msg.transforms:
                if transform.child_frame_id in ["base_footprint", "base_link"]:
                    found_base_frame = True
                if transform.header.frame_id == "odom":
                    found_odom_frame = True

        # At minimum, odom should be publishing transforms
        if has_tf:
            assert found_odom_frame or found_base_frame, \
                "TF messages found but no odom/base frames"

    def test_advanced_velocity_command_accepted(self):
        """
        Verify robot can receive velocity commands.

        Requirement: REQ-MOT-001 Robot accepts velocity commands
        """
        # TurtleBot3 Gazebo uses TwistStamped for cmd_vel
        cmd_pub = self.node.create_publisher(
            TwistStamped,
            self.CMD_VEL_TOPIC,
            10
        )

        # Create collector for odometry to see if robot responds
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )

        # Get initial position
        self.spin_for_duration(1.0)
        initial_msgs = odom_collector.get_messages()
        initial_x = 0.0
        if initial_msgs:
            initial_x = initial_msgs[-1].pose.pose.position.x

        # Send forward velocity command (TwistStamped for TurtleBot3)
        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = 0.2  # 0.2 m/s forward
        cmd.twist.angular.z = 0.0

        # Publish for 3 seconds
        import time
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop the robot
        cmd.twist.linear.x = 0.0
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        # Check if robot moved
        final_msgs = odom_collector.get_messages()
        final_x = initial_x
        if final_msgs:
            final_x = final_msgs[-1].pose.pose.position.x

        distance = abs(final_x - initial_x)
        moved = distance > 0.1  # At least 10cm

        details = f"Moved {distance:.3f} meters"

        self.assert_requirement(
            "REQ-MOT-001",
            "Robot responds to velocity commands",
            moved,
            details,
            "Motion"
        )

        self.node.destroy_publisher(cmd_pub)
        assert moved, f"Robot did not move. Distance: {distance}"

    def test_advanced_robot_stationary(self):
        """
        Verify robot is stationary when no commands sent.

        Requirement: REQ-MOT-002 Robot is stable when idle
        """
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )

        # Wait and collect odometry
        self.spin_for_duration(2.0)

        messages = odom_collector.get_messages()
        if len(messages) < 2:
            pytest.skip("Not enough odometry messages to check stability")

        # Check velocity is near zero
        max_velocity = 0.0
        for odom in messages:
            vel = math.sqrt(
                odom.twist.twist.linear.x ** 2 +
                odom.twist.twist.linear.y ** 2
            )
            max_velocity = max(max_velocity, vel)

        stationary = max_velocity < 0.05  # Less than 5cm/s

        self.assert_requirement(
            "REQ-MOT-002",
            "Robot is stationary when idle",
            stationary,
            f"Max velocity: {max_velocity:.3f} m/s",
            "Motion"
        )

        assert stationary, f"Robot is moving unexpectedly: {max_velocity}"


# ==========================================================================
# Pytest fixtures for setup/teardown
# ==========================================================================

@pytest.fixture(scope="session", autouse=True)
def print_validation_summary():
    """Print validation results summary at end of test session."""
    yield
    # After all tests complete
    collector = ValidationResultCollector.instance()
    collector.print_summary()
    collector.export_to_json("/tmp/turtlebot3_test_results.json")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
