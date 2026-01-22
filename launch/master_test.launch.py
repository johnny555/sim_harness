# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Master test launch file for running launch_testing tests directly.

This launch file can run any launch test file with proper isolation,
making it the standard entry point for test execution.

Usage:
    # Run a specific test by path
    ros2 launch sim_harness master_test.launch.py \
        test_file:=src/my_package/test/integration/test_robot_navigation.py

    # Run with specific timeout
    ros2 launch sim_harness master_test.launch.py \
        test_file:=path/to/test.py timeout:=600

    # Run with specific domain ID for isolation
    ros2 launch sim_harness master_test.launch.py \
        test_file:=path/to/test.py domain_id:=42

    # Pass arguments to the test
    ros2 launch sim_harness master_test.launch.py \
        test_file:=path/to/test.py test_args:="vehicle_id:=test_vehicle"

Note: This launch file uses OpaqueFunction to dynamically import and
execute the test's generate_test_description(). This allows running
any launch test without modification.
"""

import importlib.util
import os
import random
import sys
import unittest
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

import launch_testing
from launch_testing.actions import ReadyToTest


def get_unique_domain_id() -> str:
    """Generate a unique ROS_DOMAIN_ID for test isolation."""
    return str(random.randint(1, 232))


def get_gz_partition(domain_id: str) -> str:
    """Generate GZ_PARTITION from domain ID."""
    return f"test_partition_{domain_id}"


def load_test_module(test_path: Path):
    """Dynamically load a test module and return its generate_test_description."""
    spec = importlib.util.spec_from_file_location("test_module", test_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load test module from {test_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules["test_module"] = module
    spec.loader.exec_module(module)

    if not hasattr(module, 'generate_test_description'):
        raise AttributeError(
            f"Test file {test_path} does not have generate_test_description()"
        )

    return module.generate_test_description


def find_workspace_root() -> Path:
    """Find workspace root by looking for src/ directory."""
    cwd = Path.cwd()

    if (cwd / 'src').is_dir():
        return cwd

    for parent in cwd.parents:
        if (parent / 'src').is_dir():
            return parent

    return cwd


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch configurations and load the test."""
    test_file = LaunchConfiguration("test_file").perform(context)
    domain_id = LaunchConfiguration("domain_id").perform(context)
    test_args_str = LaunchConfiguration("test_args").perform(context)

    # Resolve test file path
    test_path = Path(test_file)
    if not test_path.is_absolute():
        workspace_root = find_workspace_root()
        test_path = workspace_root / test_path

    if not test_path.exists():
        raise FileNotFoundError(f"Test file not found: {test_path}")

    # Get or generate domain ID
    if domain_id == "auto":
        domain_id = get_unique_domain_id()

    gz_partition = get_gz_partition(domain_id)

    # Parse test arguments
    test_args = {}
    if test_args_str:
        for arg in test_args_str.split():
            if ':=' in arg:
                key, value = arg.split(':=', 1)
                test_args[key] = value

    # Environment setup for isolation
    env_actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
        LogInfo(msg=f"Running test: {test_path.name}"),
        LogInfo(msg=f"Domain ID: {domain_id}, GZ_PARTITION: {gz_partition}"),
    ]

    # Load and execute the test's generate_test_description
    try:
        generate_test_description = load_test_module(test_path)

        # Get the test's launch description
        test_ld = generate_test_description()

        # If it returns a tuple (ld, context), extract just the ld
        if isinstance(test_ld, tuple):
            test_ld = test_ld[0]

        # Include the test's entities
        # We need to extract the entities and add them to our description
        return env_actions + list(test_ld.entities)

    except Exception as e:
        # Return error info if test loading fails
        return [
            LogInfo(msg=f"ERROR: Failed to load test: {e}"),
        ]


def generate_launch_description():
    """Generate the master test launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "test_file",
            description="Path to the launch test file to run (relative to workspace or absolute)",
        ),
        DeclareLaunchArgument(
            "domain_id",
            default_value="auto",
            description="ROS_DOMAIN_ID for test isolation (or 'auto' to generate)",
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value="300",
            description="Test timeout in seconds",
        ),
        DeclareLaunchArgument(
            "test_args",
            default_value="",
            description="Additional arguments to pass to the test (space-separated key:=value)",
        ),
        OpaqueFunction(function=launch_setup),
    ])


# For use with launch_test command
generate_test_description = generate_launch_description
