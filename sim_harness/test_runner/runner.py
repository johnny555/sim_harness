# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Test runner for executing launch_testing tests directly.

This module provides functionality to run launch tests without colcon,
using the launch_test command directly with proper isolation.
"""

import os
import random
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import List, Optional, Dict, Any

from sim_harness.test_runner.test_registry import TestRegistry, TestInfo


class TestStatus(Enum):
    """Status of a test execution."""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"
    ERROR = "error"
    TIMEOUT = "timeout"


@dataclass
class TestResult:
    """Result of a test execution."""
    test: TestInfo
    status: TestStatus
    duration: float = 0.0
    return_code: int = 0
    output: str = ""
    error: str = ""
    domain_id: int = 0
    timestamp: datetime = field(default_factory=datetime.now)

    @property
    def passed(self) -> bool:
        return self.status == TestStatus.PASSED

    def summary(self) -> str:
        """Return a one-line summary."""
        icon = {
            TestStatus.PASSED: "✓",
            TestStatus.FAILED: "✗",
            TestStatus.ERROR: "!",
            TestStatus.TIMEOUT: "⏱",
            TestStatus.SKIPPED: "-",
        }.get(self.status, "?")
        return f"{icon} {self.test.name} ({self.duration:.1f}s)"


class TestRunner:
    """Runner for executing launch tests."""

    def __init__(self,
                 workspace_root: Optional[Path] = None,
                 verbose: bool = False,
                 timeout: int = 300,
                 domain_id: Optional[int] = None,
                 output_dir: Optional[Path] = None):
        """Initialize the test runner.

        Args:
            workspace_root: Root of the ROS2 workspace.
            verbose: Enable verbose output.
            timeout: Default timeout in seconds.
            domain_id: Specific ROS_DOMAIN_ID to use, or None for auto.
            output_dir: Directory for test results (XML, logs).
        """
        self.registry = TestRegistry(workspace_root)
        self.workspace_root = self.registry.workspace_root
        self.verbose = verbose
        self.default_timeout = timeout
        self.fixed_domain_id = domain_id
        self.output_dir = output_dir or Path('/tmp/launch_test_results')
        self.results: List[TestResult] = []

    def _get_domain_id(self) -> int:
        """Get a ROS_DOMAIN_ID for test isolation."""
        if self.fixed_domain_id is not None:
            return self.fixed_domain_id
        return random.randint(1, 232)

    def _get_gz_partition(self, domain_id: int) -> str:
        """Get Gazebo partition for isolation."""
        return f"test_partition_{domain_id}"

    def _build_env(self, domain_id: int) -> Dict[str, str]:
        """Build environment variables for test isolation."""
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(domain_id)
        env['GZ_PARTITION'] = self._get_gz_partition(domain_id)
        env['PYTHONUNBUFFERED'] = '1'

        # Ensure isolated discovery
        env['ROS_LOCALHOST_ONLY'] = '1'

        return env

    def run_test(self,
                 test: TestInfo,
                 timeout: Optional[int] = None,
                 extra_args: Optional[List[str]] = None) -> TestResult:
        """Run a single test.

        Args:
            test: TestInfo for the test to run.
            timeout: Timeout in seconds (overrides default).
            extra_args: Additional arguments to pass to launch_test.

        Returns:
            TestResult with execution details.
        """
        timeout = timeout or test.timeout or self.default_timeout
        domain_id = self._get_domain_id()
        env = self._build_env(domain_id)

        # Ensure output directory exists
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Build command
        cmd = [
            sys.executable, '-m', 'launch_testing.launch_test',
            str(test.path),
        ]

        # Add output file
        result_file = self.output_dir / f"{test.name}_results.xml"
        cmd.extend(['--junit-xml', str(result_file)])

        # Add extra args
        if extra_args:
            cmd.extend(extra_args)

        if self.verbose:
            print(f"\n{'='*60}")
            print(f"Running: {test.name}")
            print(f"Path: {test.path}")
            print(f"Domain ID: {domain_id}")
            print(f"Command: {' '.join(cmd)}")
            print(f"{'='*60}\n")

        start_time = datetime.now()

        try:
            proc = subprocess.run(
                cmd,
                env=env,
                cwd=self.workspace_root,
                capture_output=not self.verbose,
                text=True,
                timeout=timeout,
            )

            duration = (datetime.now() - start_time).total_seconds()

            status = TestStatus.PASSED if proc.returncode == 0 else TestStatus.FAILED

            result = TestResult(
                test=test,
                status=status,
                duration=duration,
                return_code=proc.returncode,
                output=proc.stdout if not self.verbose else "",
                error=proc.stderr if not self.verbose else "",
                domain_id=domain_id,
            )

        except subprocess.TimeoutExpired:
            duration = timeout
            result = TestResult(
                test=test,
                status=TestStatus.TIMEOUT,
                duration=duration,
                return_code=-1,
                error=f"Test timed out after {timeout}s",
                domain_id=domain_id,
            )

        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult(
                test=test,
                status=TestStatus.ERROR,
                duration=duration,
                return_code=-1,
                error=str(e),
                domain_id=domain_id,
            )

        self.results.append(result)
        return result

    def run_tests(self,
                  tests: Optional[List[TestInfo]] = None,
                  packages: Optional[List[str]] = None,
                  pattern: Optional[str] = None,
                  stop_on_failure: bool = False) -> List[TestResult]:
        """Run multiple tests.

        Args:
            tests: Specific tests to run. If None, discovers tests.
            packages: Filter by package names (if discovering).
            pattern: Glob pattern to filter tests.
            stop_on_failure: Stop after first failure.

        Returns:
            List of TestResults.
        """
        if tests is None:
            tests = self.registry.discover(packages=packages, pattern=pattern)

        if not tests:
            print("No tests found to run.")
            return []

        print(f"\nRunning {len(tests)} test(s)...\n")

        for i, test in enumerate(tests, 1):
            print(f"[{i}/{len(tests)}] {test.name}...", end=' ', flush=True)

            result = self.run_test(test)

            # Print inline result
            if not self.verbose:
                if result.passed:
                    print(f"PASSED ({result.duration:.1f}s)")
                else:
                    print(f"FAILED ({result.duration:.1f}s)")

            if stop_on_failure and not result.passed:
                print("\nStopping due to test failure.")
                break

        return self.results

    def summary(self) -> str:
        """Generate a summary of all test results."""
        if not self.results:
            return "No tests were run."

        passed = sum(1 for r in self.results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in self.results if r.status == TestStatus.FAILED)
        errors = sum(1 for r in self.results if r.status == TestStatus.ERROR)
        timeouts = sum(1 for r in self.results if r.status == TestStatus.TIMEOUT)
        total_time = sum(r.duration for r in self.results)

        lines = [
            "",
            "=" * 60,
            "TEST SUMMARY",
            "=" * 60,
            f"Total:    {len(self.results)}",
            f"Passed:   {passed}",
            f"Failed:   {failed}",
            f"Errors:   {errors}",
            f"Timeouts: {timeouts}",
            f"Duration: {total_time:.1f}s",
            "",
        ]

        # List failures
        failures = [r for r in self.results if r.status != TestStatus.PASSED]
        if failures:
            lines.append("FAILURES:")
            for result in failures:
                lines.append(f"  {result.summary()}")
                if result.error and not self.verbose:
                    # Show first line of error
                    err_line = result.error.strip().split('\n')[0][:80]
                    lines.append(f"    {err_line}")
            lines.append("")

        success = passed == len(self.results)
        lines.append("RESULT: " + ("PASSED" if success else "FAILED"))

        return '\n'.join(lines)


def run_test(test_path: str,
             verbose: bool = False,
             timeout: int = 300,
             **kwargs) -> TestResult:
    """Convenience function to run a single test by path.

    Args:
        test_path: Path to the test file.
        verbose: Enable verbose output.
        timeout: Timeout in seconds.
        **kwargs: Additional arguments for TestRunner.

    Returns:
        TestResult.
    """
    runner = TestRunner(verbose=verbose, timeout=timeout, **kwargs)

    # Create TestInfo from path
    path = Path(test_path)
    if not path.is_absolute():
        path = runner.workspace_root / path

    test = TestInfo(name=path.stem, path=path)
    return runner.run_test(test)


def run_tests(packages: Optional[List[str]] = None,
              pattern: Optional[str] = None,
              verbose: bool = False,
              timeout: int = 300,
              stop_on_failure: bool = False,
              **kwargs) -> List[TestResult]:
    """Convenience function to discover and run tests.

    Args:
        packages: Filter by package names.
        pattern: Glob pattern to filter tests.
        verbose: Enable verbose output.
        timeout: Timeout in seconds.
        stop_on_failure: Stop after first failure.
        **kwargs: Additional arguments for TestRunner.

    Returns:
        List of TestResults.
    """
    runner = TestRunner(verbose=verbose, timeout=timeout, **kwargs)
    results = runner.run_tests(packages=packages, pattern=pattern,
                               stop_on_failure=stop_on_failure)
    print(runner.summary())
    return results
