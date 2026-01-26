# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
RunVerb - Implementation of 'ros2 test run' command.

Runs launch tests with proper isolation and Gazebo cleanup.
"""

import json
from pathlib import Path
from typing import List, Optional

from sim_harness.ros2test.verb import VerbExtension
from sim_harness.ros2test.output.console import Console
from sim_harness.test_runner.test_registry import TestRegistry, TestInfo, TestType
from sim_harness.test_runner.runner import TestRunner, TestResult, TestStatus
from sim_harness.simulator.simulation_launcher import kill_all_gazebo


# Path to store failed tests for rerun
FAILED_TESTS_FILE = Path('/tmp/sim_harness_failed_tests.json')


class RunVerb(VerbExtension):
    """Run launch tests."""

    NAME = 'run'
    DESCRIPTION = 'Run tests'

    def add_arguments(self, parser, cli_name):
        """Add verb-specific arguments."""
        parser.add_argument(
            'tests',
            nargs='*',
            metavar='TEST',
            help='Specific test name(s) or path(s) to run'
        )
        parser.add_argument(
            '-p', '--packages',
            nargs='+',
            metavar='PKG',
            help='Filter by package name(s)'
        )
        parser.add_argument(
            '--pattern',
            metavar='PATTERN',
            help="Glob pattern to filter tests (e.g., 'test_nav*')"
        )
        parser.add_argument(
            '-v', '--verbose',
            action='store_true',
            help='Enable verbose output (show test stdout/stderr)'
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
            help='Specific ROS_DOMAIN_ID to use (random if not set)'
        )
        parser.add_argument(
            '-o', '--output-dir',
            type=Path,
            metavar='DIR',
            help='Directory for JUnit XML results'
        )
        parser.add_argument(
            '-w', '--workspace',
            type=Path,
            metavar='DIR',
            help='Workspace root directory'
        )
        parser.add_argument(
            '--no-sim',
            action='store_true',
            help='Skip simulation tests (run unit tests only)'
        )
        parser.add_argument(
            '--no-cleanup',
            action='store_true',
            help='Skip Gazebo cleanup before/after tests'
        )
        parser.add_argument(
            '-l', '--list-only',
            action='store_true',
            help='List tests that would be run without executing'
        )
        parser.add_argument(
            '--junit-xml',
            type=Path,
            metavar='FILE',
            help='Path for combined JUnit XML output'
        )
        # Test type filtering
        parser.add_argument(
            '--unit',
            action='store_true',
            help='Run only unit tests (C++ GTest/rtest)'
        )
        parser.add_argument(
            '--integration',
            action='store_true',
            help='Run only integration tests (Python launch tests)'
        )
        parser.add_argument(
            '--type',
            choices=['launch', 'gtest', 'all'],
            default='all',
            metavar='TYPE',
            help='Filter by test type: launch, gtest, or all (default: all)'
        )

    def main(self, *, args):
        """Execute the run command."""
        use_color = not getattr(args, 'no_color', False)
        console = Console(color=use_color)

        # Determine test types to discover
        test_types = self._get_test_types(args)

        # Clean up Gazebo before running tests (only for launch tests)
        run_launch_tests = test_types is None or TestType.LAUNCH_TEST in test_types
        if not args.no_cleanup and not args.no_sim and run_launch_tests:
            console.dim("Cleaning up stale Gazebo processes...")
            kill_all_gazebo()

        # Create runner
        runner = TestRunner(
            workspace_root=args.workspace,
            verbose=args.verbose,
            timeout=args.timeout,
            domain_id=args.domain_id,
            output_dir=args.output_dir,
        )

        # Get tests to run
        tests = self._get_tests(runner.registry, args, console, test_types)

        if not tests:
            console.warning("No tests found matching criteria.")
            return 0

        # Filter simulation tests if --no-sim
        if args.no_sim:
            tests = [t for t in tests if 'sim' not in t.markers and 'gazebo' not in t.markers]
            if not tests:
                console.warning("No non-simulation tests found.")
                return 0

        # List only mode
        if args.list_only:
            console.info(f"Would run {len(tests)} test(s):")
            for test in tests:
                pkg = f"[{test.package}] " if test.package else ""
                type_badge = f"[{test.test_type.name.lower()}] " if test.test_type else ""
                console.print(f"  {type_badge}{pkg}{test.name}")
            return 0

        # Run the tests
        console.print()
        console.header(f"Running {len(tests)} test(s)...")
        console.print()

        results = self._run_tests(runner, tests, args, console)

        # Clean up Gazebo after tests
        if not args.no_cleanup and not args.no_sim:
            console.dim("Cleaning up Gazebo processes...")
            kill_all_gazebo()

        # Print summary
        self._print_summary(results, console)

        # Save failed tests for 'ros2 test failed'
        self._save_failed_tests(results, runner.workspace_root)

        # Write combined JUnit XML if requested
        if args.junit_xml:
            self._write_junit_xml(results, args.junit_xml, console)

        # Return exit code
        failed = sum(1 for r in results if r.status != TestStatus.PASSED)
        return min(failed, 1)

    def _get_test_types(self, args) -> Optional[List[TestType]]:
        """Determine test types based on command-line arguments.

        Args:
            args: Parsed command-line arguments.

        Returns:
            List of TestType values to discover, or None for all types.
        """
        # --unit flag takes precedence
        if args.unit:
            return [TestType.GTEST]

        # --integration flag
        if args.integration:
            return [TestType.LAUNCH_TEST]

        # --type argument
        if args.type == 'launch':
            return [TestType.LAUNCH_TEST]
        elif args.type == 'gtest':
            return [TestType.GTEST]

        # Default: all types
        return None

    def _get_tests(
        self,
        registry: TestRegistry,
        args,
        console: Console,
        test_types: Optional[List[TestType]] = None
    ) -> List[TestInfo]:
        """Get tests to run based on arguments.

        Args:
            registry: The test registry.
            args: Parsed command-line arguments.
            console: Console for output.
            test_types: Optional list of test types to filter by.

        Returns:
            List of TestInfo objects for tests to run.
        """
        # If specific tests provided, find them
        if args.tests:
            tests = []
            registry.discover(test_types=test_types)  # Populate registry
            for test_arg in args.tests:
                test = registry.get_test(test_arg)
                if test:
                    tests.append(test)
                else:
                    # Try as direct path
                    test_path = Path(test_arg)
                    if not test_path.is_absolute():
                        test_path = registry.workspace_root / test_arg
                    if test_path.exists():
                        tests.append(TestInfo(
                            name=test_path.stem,
                            path=test_path,
                            package=registry._get_package_name(test_path)
                        ))
                    else:
                        console.warning(f"Test not found: {test_arg}")
            return tests

        # Otherwise discover tests with filters
        return registry.discover(
            packages=args.packages,
            pattern=args.pattern,
            test_types=test_types
        )

    def _run_tests(
        self,
        runner: TestRunner,
        tests: List[TestInfo],
        args,
        console: Console
    ) -> List[TestResult]:
        """Run tests and display progress."""
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

            # Stop on failure if requested
            if args.stop_on_failure and result.status != TestStatus.PASSED:
                console.warning("\nStopping due to test failure.")
                break

        return results

    def _print_summary(self, results: List[TestResult], console: Console) -> None:
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

        # List failures
        failures = [
            (r.test.name, r.error)
            for r in results
            if r.status != TestStatus.PASSED
        ]
        console.list_failures(failures, show_details=True)

    def _save_failed_tests(
        self,
        results: List[TestResult],
        workspace_root: Path
    ) -> None:
        """Save failed test paths for rerun."""
        failed = [
            {
                'name': r.test.name,
                'path': str(r.test.path),
                'package': r.test.package,
                'test_type': r.test.test_type.name.lower() if r.test.test_type else None,
            }
            for r in results
            if r.status != TestStatus.PASSED
        ]

        data = {
            'workspace': str(workspace_root),
            'tests': failed
        }

        try:
            with open(FAILED_TESTS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass  # Ignore if we can't write

    def _write_junit_xml(
        self,
        results: List[TestResult],
        output_path: Path,
        console: Console
    ) -> None:
        """Write combined JUnit XML output."""
        try:
            from xml.etree.ElementTree import Element, SubElement, tostring
            from xml.dom.minidom import parseString

            # Create testsuite element
            testsuite = Element('testsuite')
            testsuite.set('name', 'ros2_test')
            testsuite.set('tests', str(len(results)))
            testsuite.set('failures', str(sum(1 for r in results if r.status == TestStatus.FAILED)))
            testsuite.set('errors', str(sum(1 for r in results if r.status == TestStatus.ERROR)))
            testsuite.set('time', str(sum(r.duration for r in results)))

            for result in results:
                testcase = SubElement(testsuite, 'testcase')
                testcase.set('name', result.test.name)
                testcase.set('classname', result.test.package or 'unknown')
                testcase.set('time', str(result.duration))

                if result.status == TestStatus.FAILED:
                    failure = SubElement(testcase, 'failure')
                    failure.set('message', 'Test failed')
                    failure.text = result.error or result.output
                elif result.status == TestStatus.ERROR:
                    error = SubElement(testcase, 'error')
                    error.set('message', 'Test error')
                    error.text = result.error
                elif result.status == TestStatus.TIMEOUT:
                    error = SubElement(testcase, 'error')
                    error.set('message', 'Test timeout')
                    error.text = f"Test timed out after {result.duration}s"

            # Write formatted XML
            xml_str = parseString(tostring(testsuite)).toprettyxml(indent='  ')
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, 'w') as f:
                f.write(xml_str)

            console.dim(f"JUnit XML written to: {output_path}")

        except Exception as e:
            console.warning(f"Failed to write JUnit XML: {e}")
