#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
TurtleBot3 Integration Test with Automatic Simulation Control.

This example demonstrates how to use SimulationTestFixture to automatically
start and stop the simulation during tests.

The test framework will:
1. Start TurtleBot3 Gazebo simulation before tests
2. Wait for simulation to be ready
3. Run all tests
4. Stop simulation after tests complete

Run with:
    pytest examples/test_turtlebot3_with_sim_control.py -v -s

Or via colcon:
    colcon test --packages-select sim_harness --pytest-args="-m integration"

Prerequisites:
    sudo apt install ros-jazzy-turtlebot3-gazebo
"""

import math
import pytest

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from sim_harness import (
    SimulationTestFixture,
    ValidationResultCollector,
)


@pytest.mark.integration
class TestTurtleBot3WithSimControl(SimulationTestFixture):
    """
    Integration tests with automatic simulation control.

    The simulation is automatically started before tests and stopped after.
    """

    # Configure simulation launch
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'
    LAUNCH_ARGS = {'use_sim_time': 'true'}
    ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}

    # Timing configuration
    STARTUP_TIMEOUT = 60.0
    GAZEBO_STARTUP_DELAY = 10.0

    # If simulation can't start, skip tests (don't fail)
    REQUIRE_SIM = True

    # Topic configuration
    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"
    IMU_TOPIC = "/imu"
    CMD_VEL_TOPIC = "/cmd_vel"

    def test_lidar_publishes(self):
        """Verify LIDAR sensor is publishing data after sim starts."""
        collector = self.create_message_collector(
            self.SCAN_TOPIC,
            LaserScan,
            key="scan"
        )

        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        assert len(messages) > 0, f"No LIDAR messages received on {self.SCAN_TOPIC}"

        # Validate data
        scan = messages[-1]
        assert len(scan.ranges) > 100, "LIDAR should have > 100 ranges"

        self.validate_requirement(
            "REQ-SEN-001",
            "LIDAR publishes valid data",
            True,
            f"Received {len(messages)} messages, {len(scan.ranges)} ranges",
            "Sensors"
        )

    def test_odometry_publishes(self):
        """Verify odometry is being published."""
        collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert len(messages) > 0, f"No odometry messages received on {self.ODOM_TOPIC}"

        # Check for valid data (no NaN)
        odom = messages[-1]
        assert not math.isnan(odom.pose.pose.position.x), "Odometry X is NaN"
        assert not math.isnan(odom.pose.pose.position.y), "Odometry Y is NaN"

        self.validate_requirement(
            "REQ-SEN-002",
            "Odometry publishes valid poses",
            True,
            f"Position: ({odom.pose.pose.position.x:.2f}, {odom.pose.pose.position.y:.2f})",
            "Sensors"
        )

    def test_robot_responds_to_commands(self):
        """Verify robot moves when commanded."""
        # Get initial position
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry,
            key="odom"
        )
        self.spin_for_duration(1.0)

        initial_msgs = odom_collector.get_messages()
        if not initial_msgs:
            pytest.skip("No initial odometry")

        initial_x = initial_msgs[-1].pose.pose.position.x

        # Send velocity commands
        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)

        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = 0.2

        import time
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.twist.linear.x = 0.0
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        # Check movement
        final_msgs = odom_collector.get_messages()
        final_x = final_msgs[-1].pose.pose.position.x
        distance = abs(final_x - initial_x)

        moved = distance > 0.1

        self.validate_requirement(
            "REQ-MOT-001",
            "Robot responds to velocity commands",
            moved,
            f"Moved {distance:.3f}m",
            "Motion"
        )

        self.node.destroy_publisher(cmd_pub)
        assert moved, f"Robot didn't move enough: {distance:.3f}m"


@pytest.mark.integration
@pytest.mark.requires_sim
class TestTurtleBot3UseExisting(SimulationTestFixture):
    """
    Alternative: Use existing simulation (don't start/stop).

    Use this if you want to start the simulation manually or from
    a separate terminal.
    """

    USE_EXISTING_SIM = True  # Don't start simulation, assume it's running
    REQUIRE_SIM = True  # Skip if sim not running

    SCAN_TOPIC = "/scan"

    def test_lidar_with_existing_sim(self):
        """Test that works with pre-started simulation."""
        collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert len(messages) > 0, "No LIDAR messages - is simulation running?"


# Print summary at end
@pytest.fixture(scope="session", autouse=True)
def print_summary():
    """Print validation summary at end of session."""
    yield
    collector = ValidationResultCollector.instance()
    collector.print_summary()
    collector.export_to_json("/tmp/turtlebot3_sim_control_results.json")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
