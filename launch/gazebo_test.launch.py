# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Generic Gazebo test launcher for integration tests.

This launch file provides a template for running C++ GTest binaries with
a Gazebo simulation. It handles:
- Test isolation via ROS_DOMAIN_ID and GZ_PARTITION
- Gazebo simulator lifecycle
- Test binary execution
- Cleanup on shutdown

Usage in consumer packages:
    from sim_harness.launch.gazebo_test import generate_gazebo_test_description

    def generate_test_description():
        return generate_gazebo_test_description(
            world_file='path/to/world.sdf',
            test_binary='my_test_binary',
            package='my_package',
        )
"""

import os
import random
import unittest

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

import launch_testing


def get_unique_domain_id():
    """Generate a unique ROS_DOMAIN_ID for test isolation."""
    # Use random ID between 1-232 to avoid conflicts
    return str(random.randint(1, 232))


def get_gz_partition(domain_id: str) -> str:
    """Generate GZ_PARTITION from domain ID for Gazebo isolation."""
    return f"gz_test_{domain_id}"


def generate_gazebo_test_description(
    world_file: str,
    test_binary: str,
    package: str,
    gazebo_args: str = "",
    startup_timeout: float = 30.0,
    test_timeout: float = 300.0,
    extra_env: dict = None,
):
    """Generate a LaunchDescription for Gazebo-based integration tests.

    Args:
        world_file: Path to the SDF world file
        test_binary: Name of the GTest binary to run
        package: Package containing the test binary
        gazebo_args: Additional arguments for Gazebo
        startup_timeout: Seconds to wait for Gazebo startup
        test_timeout: Maximum test duration in seconds
        extra_env: Additional environment variables

    Returns:
        LaunchDescription with Gazebo and test execution
    """
    # Generate isolation IDs
    domain_id = get_unique_domain_id()
    gz_partition = get_gz_partition(domain_id)

    # Environment setup for isolation
    env_actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
    ]

    # Add any extra environment variables
    if extra_env:
        for key, value in extra_env.items():
            env_actions.append(SetEnvironmentVariable(key, value))

    # Gazebo launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": f"-r {world_file} {gazebo_args}",
        }.items(),
    )

    # Test binary execution (delayed to allow Gazebo startup)
    test_binary_path = PathJoinSubstitution([
        FindPackageShare(package),
        "lib",
        package,
        test_binary,
    ])

    test_process = ExecuteProcess(
        cmd=[test_binary_path, "--gtest_output=xml:/tmp/test_results.xml"],
        name="test_runner",
        output="screen",
        env={
            "ROS_DOMAIN_ID": domain_id,
            "GZ_PARTITION": gz_partition,
        },
    )

    # Delay test start to allow Gazebo to initialize
    delayed_test = TimerAction(
        period=startup_timeout,
        actions=[test_process],
    )

    return LaunchDescription([
        *env_actions,
        gz_sim_launch,
        delayed_test,
        ReadyToTest(),
    ]), {"test_process": test_process}


def generate_test_description():
    """Default test description for standalone testing."""
    # This is a template - consumer packages should call
    # generate_gazebo_test_description() with their parameters

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_file",
            description="Path to SDF world file",
        ),
        DeclareLaunchArgument(
            "test_binary",
            description="Name of the GTest binary",
        ),
        DeclareLaunchArgument(
            "package",
            description="Package containing the test binary",
        ),
    ])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Post-shutdown test to verify test results."""

    def test_exit_code(self, proc_info, test_process):
        """Verify the test process exited successfully."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [0],
            process=test_process,
        )
