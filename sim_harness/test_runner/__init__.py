# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test runner module for discovering and executing launch tests.

This module provides a standardized way to run launch_testing tests
without going through colcon test.

Usage:
    # List all tests
    ros2 run sim_harness list_tests

    # Run a specific test
    ros2 run sim_harness run_test path/to/test.py

    # Run all tests
    ros2 run sim_harness run_tests
"""

from sim_harness.test_runner.test_registry import (
    TestRegistry,
    TestInfo,
    discover_tests,
)
from sim_harness.test_runner.runner import (
    TestRunner,
    TestResult,
    run_test,
    run_tests,
)

__all__ = [
    'TestRegistry',
    'TestInfo',
    'discover_tests',
    'TestRunner',
    'TestResult',
    'run_test',
    'run_tests',
]
