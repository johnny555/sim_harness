# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Command-line interface for the test runner.

Provides commands:
- list_tests: Discover and list all launch tests
- run_test: Run a single test
- run_tests: Run all or filtered tests
"""

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from sim_harness.test_runner.test_registry import TestRegistry
from sim_harness.test_runner.runner import TestRunner, TestStatus


def list_tests_main(args: Optional[List[str]] = None) -> int:
    """Entry point for list_tests command."""
    parser = argparse.ArgumentParser(
        description="List all launch tests in the workspace",
        prog="ros2 run sim_harness list_tests",
    )
    parser.add_argument(
        "-p", "--packages",
        nargs="+",
        help="Filter by package names",
    )
    parser.add_argument(
        "--pattern",
        help="Glob pattern to filter tests (e.g., 'test_nav*')",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed test information",
    )
    parser.add_argument(
        "-w", "--workspace",
        type=Path,
        help="Workspace root directory",
    )

    parsed = parser.parse_args(args)

    registry = TestRegistry(workspace_root=parsed.workspace)
    tests = registry.discover(packages=parsed.packages, pattern=parsed.pattern)

    if not tests:
        print("No launch tests found.")
        return 0

    print(registry.list_tests(packages=parsed.packages, verbose=parsed.verbose))
    return 0


def run_test_main(args: Optional[List[str]] = None) -> int:
    """Entry point for run_test command."""
    parser = argparse.ArgumentParser(
        description="Run a single launch test",
        prog="ros2 run sim_harness run_test",
    )
    parser.add_argument(
        "test",
        help="Test name or path to run",
    )
    parser.add_argument(
        "-t", "--timeout",
        type=int,
        default=300,
        help="Timeout in seconds (default: 300)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "-d", "--domain-id",
        type=int,
        help="Specific ROS_DOMAIN_ID to use",
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=Path,
        help="Directory for test results",
    )
    parser.add_argument(
        "-w", "--workspace",
        type=Path,
        help="Workspace root directory",
    )
    parser.add_argument(
        "extra_args",
        nargs="*",
        help="Additional arguments to pass to the test",
    )

    parsed = parser.parse_args(args)

    runner = TestRunner(
        workspace_root=parsed.workspace,
        verbose=parsed.verbose,
        timeout=parsed.timeout,
        domain_id=parsed.domain_id,
        output_dir=parsed.output_dir,
    )

    # Find the test
    registry = runner.registry
    tests = registry.discover()

    test_info = registry.get_test(parsed.test)

    if test_info is None:
        # Try as a direct path
        test_path = Path(parsed.test)
        if not test_path.is_absolute():
            test_path = runner.workspace_root / test_path

        if test_path.exists():
            from sim_harness.test_runner.test_registry import TestInfo
            test_info = TestInfo(name=test_path.stem, path=test_path)
        else:
            print(f"Test not found: {parsed.test}")
            print("\nAvailable tests:")
            for t in tests:
                print(f"  {t.name}")
            return 1

    result = runner.run_test(test_info, extra_args=parsed.extra_args)

    print(runner.summary())

    return 0 if result.passed else 1


def run_tests_main(args: Optional[List[str]] = None) -> int:
    """Entry point for run_tests command."""
    parser = argparse.ArgumentParser(
        description="Run multiple launch tests",
        prog="ros2 run sim_harness run_tests",
    )
    parser.add_argument(
        "-p", "--packages",
        nargs="+",
        help="Filter by package names",
    )
    parser.add_argument(
        "--pattern",
        help="Glob pattern to filter tests",
    )
    parser.add_argument(
        "-t", "--timeout",
        type=int,
        default=300,
        help="Timeout per test in seconds (default: 300)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "-x", "--stop-on-failure",
        action="store_true",
        help="Stop after first test failure",
    )
    parser.add_argument(
        "-d", "--domain-id",
        type=int,
        help="Specific ROS_DOMAIN_ID to use (same for all tests)",
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=Path,
        help="Directory for test results",
    )
    parser.add_argument(
        "-w", "--workspace",
        type=Path,
        help="Workspace root directory",
    )
    parser.add_argument(
        "-l", "--list-only",
        action="store_true",
        help="Only list tests without running",
    )

    parsed = parser.parse_args(args)

    runner = TestRunner(
        workspace_root=parsed.workspace,
        verbose=parsed.verbose,
        timeout=parsed.timeout,
        domain_id=parsed.domain_id,
        output_dir=parsed.output_dir,
    )

    tests = runner.registry.discover(packages=parsed.packages, pattern=parsed.pattern)

    if not tests:
        print("No tests found matching criteria.")
        return 0

    if parsed.list_only:
        print(f"Would run {len(tests)} test(s):")
        for t in tests:
            print(f"  {t.name} ({t.package})")
        return 0

    results = runner.run_tests(
        tests=tests,
        stop_on_failure=parsed.stop_on_failure,
    )

    print(runner.summary())

    # Return non-zero if any test failed
    failed = sum(1 for r in results if r.status != TestStatus.PASSED)
    return min(failed, 1)


if __name__ == "__main__":
    # Allow running module directly for testing
    if len(sys.argv) > 1 and sys.argv[1] == "list":
        sys.exit(list_tests_main(sys.argv[2:]))
    elif len(sys.argv) > 1 and sys.argv[1] == "run":
        sys.exit(run_test_main(sys.argv[2:]))
    else:
        sys.exit(run_tests_main(sys.argv[1:]))
