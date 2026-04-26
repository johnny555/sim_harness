#!/usr/bin/env python3
# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
TurtleBot3 integration test with automatic simulation control.

Demonstrates:
- ``SimTestFixture`` with ``LAUNCH_PACKAGE`` / ``LAUNCH_FILE`` to start/stop
  the sim around the test session.
- ``@pytest.mark.requirement(...)`` to attach Jama requirement IDs which
  the ``sim_harness`` pytest plugin exports to xlsx via ``--jama-xlsx``.

Run with::

    pytest examples/test_turtlebot3_with_sim_control.py -v -s
    pytest examples/test_turtlebot3_with_sim_control.py --jama-xlsx /tmp/tb3.xlsx

Prerequisites::

    sudo apt install ros-jazzy-turtlebot3-gazebo
"""

import math
import time

import pytest

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from sim_harness import SimTestFixture


@pytest.mark.integration
class TestTurtleBot3WithSimControl(SimTestFixture):
    """Integration tests with automatic simulation control."""

    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'
    LAUNCH_ARGS = {'use_sim_time': 'true'}
    ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}
    STARTUP_TIMEOUT = 60.0
    GAZEBO_STARTUP_DELAY = 10.0
    REQUIRE_SIM = True

    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"
    CMD_VEL_TOPIC = "/cmd_vel"

    @pytest.mark.requirement(
        "REQ-SEN-001", "LIDAR publishes valid data", category="Sensors",
    )
    def test_lidar_publishes(self):
        """LIDAR sensor must publish reasonably-sized scans."""
        collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan, key="scan")
        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        assert messages, f"No LIDAR messages on {self.SCAN_TOPIC}"
        assert len(messages[-1].ranges) > 100, (
            f"LIDAR has only {len(messages[-1].ranges)} ranges (need >100)"
        )

    @pytest.mark.requirement(
        "REQ-SEN-002", "Odometry publishes valid poses", category="Sensors",
    )
    def test_odometry_publishes(self):
        """Odometry must publish poses with finite x/y."""
        collector = self.create_message_collector(self.ODOM_TOPIC, Odometry, key="odom")
        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert messages, f"No odometry messages on {self.ODOM_TOPIC}"
        odom = messages[-1]
        assert not math.isnan(odom.pose.pose.position.x), "odometry x is NaN"
        assert not math.isnan(odom.pose.pose.position.y), "odometry y is NaN"

    @pytest.mark.requirement(
        "REQ-MOT-001", "Robot responds to velocity commands", category="Motion",
    )
    def test_robot_responds_to_commands(self):
        """Driving forward for 3 s must move the robot at least 10 cm."""
        odom_collector = self.create_message_collector(self.ODOM_TOPIC, Odometry, key="odom")
        self.spin_for_duration(1.0)

        initial_msgs = odom_collector.get_messages()
        if not initial_msgs:
            pytest.skip("no initial odometry")
        initial_x = initial_msgs[-1].pose.pose.position.x

        cmd_pub = self.node.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        try:
            cmd = TwistStamped()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = 0.2

            t0 = time.monotonic()
            while time.monotonic() - t0 < 3.0:
                cmd.header.stamp = self.node.get_clock().now().to_msg()
                cmd_pub.publish(cmd)
                self.spin_for_duration(0.1)

            cmd.twist.linear.x = 0.0
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.5)
        finally:
            self.node.destroy_publisher(cmd_pub)

        final_msgs = odom_collector.get_messages()
        final_x = final_msgs[-1].pose.pose.position.x
        distance = abs(final_x - initial_x)
        assert distance > 0.1, f"robot moved only {distance:.3f}m"


@pytest.mark.integration
@pytest.mark.requires_sim
class TestTurtleBot3UseExisting(SimTestFixture):
    """Use an already-running simulation. Useful when starting Gazebo manually
    or from another terminal."""

    USE_EXISTING_SIM = True
    REQUIRE_SIM = True
    SCAN_TOPIC = "/scan"

    def test_lidar_with_existing_sim(self):
        collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
        self.spin_for_duration(3.0)
        messages = collector.get_messages()
        assert messages, "No LIDAR messages — is the simulation running?"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
