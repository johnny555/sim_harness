# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""GTest runner launch file for C++ test binaries.

This launch file runs a C++ test binary with proper ROS 2 isolation,
without launching a simulator. Useful for:
- Unit tests that don't need a simulator
- Tests using recorded data (rosbag replay)
- Tests with mock publishers

Note: This file is named 'gtest_runner' (not 'test_runner') to avoid
being picked up by test discovery, since it requires arguments.

Usage:
    ros2 launch sim_harness gtest_runner.launch.py \
        package:=my_package \
        test_binary:=my_test \
        domain_id:=42
"""

import os
import random
import unittest

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import launch_testing
from launch_testing.actions import ReadyToTest


def get_unique_domain_id():
    """Generate a unique ROS_DOMAIN_ID for test isolation."""
    return str(random.randint(1, 232))


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch configurations."""
    package = LaunchConfiguration("package").perform(context)
    test_binary = LaunchConfiguration("test_binary").perform(context)
    domain_id = LaunchConfiguration("domain_id").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    gtest_filter = LaunchConfiguration("gtest_filter").perform(context)

    # Use provided domain_id or generate one
    if domain_id == "auto":
        domain_id = get_unique_domain_id()

    # Build test command
    cmd = [
        PathJoinSubstitution([
            FindPackageShare(package),
            "lib",
            package,
            test_binary,
        ]).perform(context),
    ]

    # Add GTest output
    if output_dir:
        output_file = os.path.join(output_dir, f"{test_binary}_results.xml")
        cmd.append(f"--gtest_output=xml:{output_file}")

    # Add GTest filter if specified
    if gtest_filter:
        cmd.append(f"--gtest_filter={gtest_filter}")

    test_process = ExecuteProcess(
        cmd=cmd,
        name="test_runner",
        output="screen",
        env={
            "ROS_DOMAIN_ID": domain_id,
        },
    )

    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
        test_process,
    ]


def generate_test_description():
    """Generate the test launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "package",
            description="Package containing the test binary",
        ),
        DeclareLaunchArgument(
            "test_binary",
            description="Name of the GTest binary to run",
        ),
        DeclareLaunchArgument(
            "domain_id",
            default_value="auto",
            description="ROS_DOMAIN_ID for isolation (or 'auto' to generate)",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/tmp",
            description="Directory for test result XML files",
        ),
        DeclareLaunchArgument(
            "gtest_filter",
            default_value="",
            description="GTest filter pattern (e.g., 'TestSuite.TestName')",
        ),
        OpaqueFunction(function=launch_setup),
        ReadyToTest(),
    ])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Post-shutdown test to verify test results."""

    def test_exit_code(self, proc_info):
        """Verify the test process exited successfully."""
        launch_testing.asserts.assertExitCodes(proc_info, [0])
