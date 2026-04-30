# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""TurtleBot3 integration tests using the Python sim_harness fixture."""

import math
import time

import pytest
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from sim_harness import SimTestFixture


def _missing_turtlebot3_packages() -> list[str]:
    required = [
        "turtlebot3_gazebo",
        "turtlebot3_description",
    ]
    missing = []
    for package in required:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            missing.append(package)
    return missing


_MISSING_TURTLEBOT3_PACKAGES = _missing_turtlebot3_packages()


@pytest.mark.skipif(
    bool(_MISSING_TURTLEBOT3_PACKAGES),
    reason=(
        "TurtleBot3 integration prerequisites missing: "
        + ", ".join(_MISSING_TURTLEBOT3_PACKAGES)
    ),
)
class TestTurtleBot3Integration(SimTestFixture):
    """Launch TurtleBot3 Gazebo and validate the basic ROS surface."""

    LAUNCH_PACKAGE = "turtlebot3_gazebo"
    LAUNCH_FILE = "turtlebot3_world.launch.py"
    LAUNCH_ARGS = {"use_sim_time": "true"}
    ENV_VARS = {"TURTLEBOT3_MODEL": "waffle"}
    WORLD = "turtlebot3_world"
    ROBOT_MODEL = "waffle"
    STARTUP_TIMEOUT = 120.0
    GAZEBO_STARTUP_DELAY = 5.0
    REQUIRE_SIM = True

    SCAN_TOPIC = "/scan"
    ODOM_TOPIC = "/odom"
    IMU_TOPIC = "/imu"
    CMD_VEL_TOPIC = "/cmd_vel"

    READINESS_TOPICS = [
        (SCAN_TOPIC, LaserScan, 1.0),
        (ODOM_TOPIC, Odometry, 1.0),
    ]
    READINESS_TIMEOUT = 60.0
    READINESS_MAX_ATTEMPTS = 2

    def test_lidar_publishes(self):
        collector = self.create_message_collector(
            self.SCAN_TOPIC, LaserScan, key="scan")

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert messages, f"No LIDAR messages received on {self.SCAN_TOPIC}"
        assert len(messages[-1].ranges) > 100

    def test_odometry_publishes_finite_pose(self):
        collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom")

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert messages, f"No odometry messages received on {self.ODOM_TOPIC}"
        pose = messages[-1].pose.pose.position
        assert math.isfinite(pose.x)
        assert math.isfinite(pose.y)

    def test_imu_publishes(self):
        collector = self.create_message_collector(
            self.IMU_TOPIC, Imu, key="imu")

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert messages, f"No IMU messages received on {self.IMU_TOPIC}"

    def test_robot_responds_to_velocity_commands(self):
        odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry, key="odom")
        self.spin_for_duration(1.0)

        initial_messages = odom_collector.get_messages()
        if not initial_messages:
            pytest.skip("No initial odometry available")
        initial_x = initial_messages[-1].pose.pose.position.x

        cmd_pub = self.node.create_publisher(Twist, self.CMD_VEL_TOPIC, 10)
        try:
            cmd = Twist()
            cmd.linear.x = 0.2

            start = time.monotonic()
            while time.monotonic() - start < 3.0:
                cmd_pub.publish(cmd)
                self.spin_for_duration(0.1)

            cmd.linear.x = 0.0
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.5)
        finally:
            self.node.destroy_publisher(cmd_pub)

        final_messages = odom_collector.get_messages()
        assert final_messages, "No final odometry available"
        final_x = final_messages[-1].pose.pose.position.x
        assert abs(final_x - initial_x) > 0.05
