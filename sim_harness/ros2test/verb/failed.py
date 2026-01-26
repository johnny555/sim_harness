# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
FailedVerb - Implementation of 'ros2 test failed' command.

Reruns tests that failed in the previous test run.
"""

import json
from pathlib import Path
from typing import List

from sim_harness.ros2test.verb import VerbExtension
from sim_harness.ros2test.output.console import Console
from sim_harness.test_runner.test_registry import TestInfo, TestType
from sim_harness.test_runner.runner import TestRunner, TestStatus
from sim_harness.simulator.simulation_launcher import kill_all_gazebo


# Path where failed tests are stored
FAILED_TESTS_FILE = Path('/tmp/sim_harness_failed_tests.json')


class FailedVerb(VerbExtension):
    """Rerun failed tests from previous run."""

    NAME = 'failed'
    DESCRIPTION = 'Rerun failed tests'

    def add_arguments(self, parser, cli_name):
        """Add verb-specific arguments."""
        parser.add_argument(
            '-v', '--verbose',
            action='store_true',
            help='Enable verbose output'
        )
        parser.add_argument(
            '-x', '--stop-on-failure',
            action='store_true',
            help='Stop after first test failure'
        )
        parser.add_argument(
            '-t', '--timeout',
            type=int,
            default=300,
            metavar='SEC',
            help='Timeout per test in seconds (default: 300)'
        )
        parser.add_argument(
            '-d', '--domain-id',
            type=int,
            metavar='ID',
            help='Specific ROS_DOMAIN_ID to use'
        )
        parser.add_argument(
            '-o', '--output-dir',
            type=Path,
            metavar='DIR',
            help='Directory for JUnit XML results'
        )
        parser.add_argument(
            '--no-cleanup',
            action='store_true',
            help='Skip Gazebo cleanup before/after tests'
        )
        parser.add_argument(
            '-l', '--list-only',
            action='store_true',
            help='List failed tests without running'
        )
        parser.add_argument(
            '--clear',
            action='store_true',
            help='Clear the failed tests list'
        )

    def main(self, *, args):
        """Execute the failed command."""
        use_color = not getattr(args, 'no_color', False)
        console = Console(color=use_color)

        # Clear option
        if args.clear:
            self._clear_failed_tests(console)
            return 0

        # Load failed tests
        failed_data = self._load_failed_tests(console)
        if not failed_data:
            console.info("No failed tests to rerun.")
            console.dim("Run 'ros2 test run' first to generate a failed tests list.")
            return 0

        workspace_root = Path(failed_data.get('workspace', '.'))
        test_data = failed_data.get('tests', [])

        if not test_data:
            console.info("No failed tests to rerun.")
            return 0

        # Convert to TestInfo objects
        tests = []
        for t in test_data:
            path = Path(t['path'])
            if not path.exists():
                continue

            # Determine test type from saved data or infer from path
            test_type = TestType.LAUNCH_TEST
            if t.get('test_type'):
                try:
                    test_type = TestType[t['test_type'].upper()]
                except (KeyError, AttributeError):
                    pass
            elif path.suffix == '' or 'gtest' in t['name'] or 'rtest' in t['name']:
                # Likely a C++ executable
                test_type = TestType.GTEST

            tests.append(TestInfo(
                name=t['name'],
                path=path,
                package=t.get('package'),
                test_type=test_type
            ))

        if not tests:
            console.warning("Failed test files no longer exist.")
            return 1

        # List only mode
        if args.list_only:
            console.info(f"Failed tests from previous run ({len(tests)}):")
            for test in tests:
                pkg = f"[{test.package}] " if test.package else ""
                type_badge = f"[{test.test_type.name.lower()}] " if test.test_type else ""
                console.print(f"  {type_badge}{pkg}{test.name}")
            return 0

        # Clean up Gazebo
        if not args.no_cleanup:
            console.dim("Cleaning up stale Gazebo processes...")
            kill_all_gazebo()

        # Create runner
        runner = TestRunner(
            workspace_root=workspace_root,
            verbose=args.verbose,
            timeout=args.timeout,
            domain_id=args.domain_id,
            output_dir=args.output_dir,
        )

        # Run failed tests
        console.print()
        console.header(f"Rerunning {len(tests)} failed test(s)...")
        console.print()

        results = []
        for i, test in enumerate(tests, 1):
            console.test_running(test.name, i, len(tests))

            result = runner.run_test(test)
            results.append(result)

            # Display result
            if not args.verbose:
                if result.status == TestStatus.PASSED:
                    console.success(f"PASSED ({result.duration:.1f}s)")
                elif result.status == TestStatus.TIMEOUT:
                    console.warning(f"TIMEOUT ({result.duration:.1f}s)")
                else:
                    console.error(f"FAILED ({result.duration:.1f}s)")

            if args.stop_on_failure and result.status != TestStatus.PASSED:
                console.warning("\nStopping due to test failure.")
                break

        # Clean up
        if not args.no_cleanup:
            console.dim("Cleaning up Gazebo processes...")
            kill_all_gazebo()

        # Print summary
        self._print_summary(results, console)

        # Update failed tests file (only keep still-failing tests)
        still_failed = [
            {
                'name': r.test.name,
                'path': str(r.test.path),
                'package': r.test.package,
                'test_type': r.test.test_type.name.lower() if r.test.test_type else None,
            }
            for r in results
            if r.status != TestStatus.PASSED
        ]

        if still_failed:
            self._save_failed_tests(workspace_root, still_failed)
        else:
            # All tests passed, clear the file
            self._clear_failed_tests(console, quiet=True)

        # Return exit code
        failed = sum(1 for r in results if r.status != TestStatus.PASSED)
        return min(failed, 1)

    def _load_failed_tests(self, console: Console) -> dict:
        """Load failed tests from file."""
        if not FAILED_TESTS_FILE.exists():
            return {}

        try:
            with open(FAILED_TESTS_FILE) as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            console.warning(f"Could not read failed tests file: {e}")
            return {}

    def _save_failed_tests(self, workspace_root: Path, tests: List[dict]) -> None:
        """Save updated failed tests list."""
        data = {
            'workspace': str(workspace_root),
            'tests': tests
        }
        try:
            with open(FAILED_TESTS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass

    def _clear_failed_tests(self, console: Console, quiet: bool = False) -> None:
        """Clear the failed tests file."""
        try:
            if FAILED_TESTS_FILE.exists():
                FAILED_TESTS_FILE.unlink()
                if not quiet:
                    console.info("Cleared failed tests list.")
            elif not quiet:
                console.info("No failed tests list to clear.")
        except OSError as e:
            console.warning(f"Could not clear failed tests: {e}")

    def _print_summary(self, results, console: Console) -> None:
        """Print test results summary."""
        if not results:
            return

        passed = sum(1 for r in results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in results if r.status == TestStatus.FAILED)
        errors = sum(1 for r in results if r.status == TestStatus.ERROR)
        timeouts = sum(1 for r in results if r.status == TestStatus.TIMEOUT)
        total_time = sum(r.duration for r in results)

        console.summary(
            total=len(results),
            passed=passed,
            failed=failed,
            errors=errors,
            timeouts=timeouts,
            duration=total_time
        )

        # List still-failing tests
        failures = [
            (r.test.name, r.error)
            for r in results
            if r.status != TestStatus.PASSED
        ]
        console.list_failures(failures, show_details=True)
