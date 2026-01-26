# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test executors for different test types.

Provides an abstraction layer for executing different types of tests
(launch tests, GTest, pytest) with consistent interfaces.
"""

import subprocess
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, Dict, Any

from sim_harness.test_runner.test_registry import TestInfo, TestType


class TestExecutor(ABC):
    """Abstract base class for test execution strategies."""

    @abstractmethod
    def build_command(self, test: TestInfo, output_file: Path) -> List[str]:
        """Build command line for test execution.

        Args:
            test: TestInfo for the test to run.
            output_file: Path for JUnit XML output.

        Returns:
            List of command-line arguments.
        """
        pass

    @abstractmethod
    def get_output_filename(self, test: TestInfo) -> str:
        """Get the output filename for test results.

        Args:
            test: TestInfo for the test.

        Returns:
            Filename for the test results.
        """
        pass

    def get_extra_env(self, test: TestInfo) -> Dict[str, str]:
        """Get additional environment variables for the test.

        Args:
            test: TestInfo for the test.

        Returns:
            Dictionary of extra environment variables.
        """
        return {}


class LaunchTestExecutor(TestExecutor):
    """Executor for launch_testing Python tests."""

    def build_command(self, test: TestInfo, output_file: Path) -> List[str]:
        """Build launch_testing command.

        Args:
            test: TestInfo for the test to run.
            output_file: Path for JUnit XML output.

        Returns:
            List of command-line arguments.
        """
        return [
            sys.executable, '-m', 'launch_testing.launch_test',
            str(test.path),
            '--junit-xml', str(output_file),
        ]

    def get_output_filename(self, test: TestInfo) -> str:
        """Get output filename for launch test results."""
        return f"{test.name}_results.xml"


class GTestExecutor(TestExecutor):
    """Executor for GTest/rtest C++ tests."""

    def build_command(self, test: TestInfo, output_file: Path) -> List[str]:
        """Build GTest command.

        Args:
            test: TestInfo for the test to run.
            output_file: Path for JUnit XML output.

        Returns:
            List of command-line arguments.
        """
        return [
            str(test.path),
            f'--gtest_output=xml:{output_file}',
            '--gtest_color=yes',
        ]

    def get_output_filename(self, test: TestInfo) -> str:
        """Get output filename for GTest results."""
        return f"{test.name}_gtest_results.xml"

    def get_extra_env(self, test: TestInfo) -> Dict[str, str]:
        """Get extra environment for GTest execution.

        GTest tests may need specific environment settings.
        """
        return {
            # Ensure GTest doesn't use color codes when capturing output
            'GTEST_COLOR': 'yes',
        }


class PytestExecutor(TestExecutor):
    """Executor for pytest Python tests (future use)."""

    def build_command(self, test: TestInfo, output_file: Path) -> List[str]:
        """Build pytest command.

        Args:
            test: TestInfo for the test to run.
            output_file: Path for JUnit XML output.

        Returns:
            List of command-line arguments.
        """
        return [
            sys.executable, '-m', 'pytest',
            str(test.path),
            f'--junit-xml={output_file}',
            '-v',
        ]

    def get_output_filename(self, test: TestInfo) -> str:
        """Get output filename for pytest results."""
        return f"{test.name}_pytest_results.xml"


# Registry of executors by test type
_EXECUTORS: Dict[TestType, TestExecutor] = {
    TestType.LAUNCH_TEST: LaunchTestExecutor(),
    TestType.GTEST: GTestExecutor(),
    TestType.PYTEST: PytestExecutor(),
}


def get_executor(test: TestInfo) -> TestExecutor:
    """Get the appropriate executor for a test.

    Factory function that returns the correct executor based on
    the test's type.

    Args:
        test: TestInfo for the test.

    Returns:
        TestExecutor instance for the test type.

    Raises:
        ValueError: If no executor is registered for the test type.
    """
    executor = _EXECUTORS.get(test.test_type)
    if executor is None:
        # Default to launch test executor for backward compatibility
        return _EXECUTORS[TestType.LAUNCH_TEST]
    return executor


def register_executor(test_type: TestType, executor: TestExecutor) -> None:
    """Register a custom executor for a test type.

    Allows extending the executor system with custom executors.

    Args:
        test_type: The test type to register for.
        executor: The executor instance to use.
    """
    _EXECUTORS[test_type] = executor
