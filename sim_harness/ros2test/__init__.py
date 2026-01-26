# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
ros2 test CLI command for sim_harness.

Provides a unified interface for running simulation tests via the ros2 CLI:

    ros2 test list                    # List all tests
    ros2 test list -p my_package      # Filter by package
    ros2 test list -v                 # Verbose with paths

    ros2 test run                     # Run all tests
    ros2 test run test_navigation    # Run specific test
    ros2 test run -p my_package       # Run package tests
    ros2 test run --pattern "nav*"    # Pattern filter
    ros2 test run -v -x               # Verbose, stop on failure
    ros2 test run --no-sim            # Unit tests only

    ros2 test failed                  # Rerun failed tests
"""

__all__ = []
