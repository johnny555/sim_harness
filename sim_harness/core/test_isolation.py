# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test isolation utilities for ROS 2 tests.

Prevents crosstalk between concurrent tests by assigning unique
ROS_DOMAIN_ID values and Gazebo partitions.
"""

import os
import random
from dataclasses import dataclass
from typing import Optional


@dataclass
class TestIsolationConfig:
    """Configuration for test isolation."""

    domain_id: int
    """ROS 2 domain ID (0-232)."""

    gz_partition: str
    """Gazebo partition name for simulation isolation."""


def get_test_isolation_config() -> TestIsolationConfig:
    """
    Get test isolation configuration.

    Reads ROS_DOMAIN_ID from environment and maps it to a test-safe
    range (100-199) to prevent collision with production systems.

    Returns:
        TestIsolationConfig with domain ID and Gazebo partition
    """
    # Read domain ID from environment, default to 0
    env_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    try:
        domain_id = int(env_domain)
    except ValueError:
        domain_id = 0

    # Map to test isolation range (100-199)
    test_domain_id = 100 + (domain_id % 100)

    # Generate Gazebo partition name
    gz_partition = f"gz_test_{test_domain_id}"

    return TestIsolationConfig(
        domain_id=test_domain_id,
        gz_partition=gz_partition
    )


def apply_test_isolation(config: Optional[TestIsolationConfig] = None) -> TestIsolationConfig:
    """
    Apply test isolation by setting environment variables.

    Sets ROS_DOMAIN_ID and GZ_PARTITION environment variables to
    isolate the test from other ROS 2 systems.

    Args:
        config: Optional config to use (generates new if None)

    Returns:
        The applied TestIsolationConfig
    """
    if config is None:
        config = get_test_isolation_config()

    os.environ['ROS_DOMAIN_ID'] = str(config.domain_id)
    os.environ['GZ_PARTITION'] = config.gz_partition

    return config


def generate_test_node_name(
    base_name: str = "test_node",
    domain_id: Optional[int] = None
) -> str:
    """
    Generate a unique node name for test isolation.

    Combines base name with domain suffix and random number to
    prevent node name collisions.

    Args:
        base_name: Base node name
        domain_id: Optional domain ID (reads from env if None)

    Returns:
        Unique node name like "test_node_d100_1234"
    """
    if domain_id is None:
        config = get_test_isolation_config()
        domain_id = config.domain_id

    random_suffix = random.randint(1000, 9999)
    return f"{base_name}_d{domain_id}_{random_suffix}"
