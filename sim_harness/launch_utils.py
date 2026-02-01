# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Shared utilities for sim_harness launch files.

Consolidates common functions used across multiple launch files for
test isolation, environment setup, and simulator configuration.

Usage in launch files:
    from sim_harness.launch_utils import (
        get_unique_domain_id,
        get_gz_partition,
        create_isolation_env_actions,
    )
"""

import random

from launch.actions import LogInfo, SetEnvironmentVariable


def get_unique_domain_id() -> str:
    """Generate a unique ROS_DOMAIN_ID for test isolation.

    Returns a random ID between 1 and 232 to avoid conflicts with
    other ROS 2 processes on the same machine.

    Returns:
        String representation of the domain ID.
    """
    return str(random.randint(1, 232))


def get_gz_partition(domain_id: str) -> str:
    """Generate GZ_PARTITION from domain ID for Gazebo topic isolation.

    Each test gets its own Gazebo partition so that topic traffic from
    concurrent simulations does not interfere.

    Args:
        domain_id: The ROS_DOMAIN_ID string.

    Returns:
        A partition name string like ``gz_test_42``.
    """
    return f"gz_test_{domain_id}"


def create_isolation_env_actions(
    domain_id: str,
    gz_partition: str,
    *,
    localhost_only: bool = False,
    unbuffered_python: bool = False,
):
    """Create launch actions that set environment variables for test isolation.

    Args:
        domain_id: ROS_DOMAIN_ID value.
        gz_partition: GZ_PARTITION value.
        localhost_only: If True, set ROS_LOCALHOST_ONLY=1.
        unbuffered_python: If True, set PYTHONUNBUFFERED=1.

    Returns:
        List of SetEnvironmentVariable actions.
    """
    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
    ]
    if localhost_only:
        actions.append(SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"))
    if unbuffered_python:
        actions.append(SetEnvironmentVariable("PYTHONUNBUFFERED", "1"))
    return actions
