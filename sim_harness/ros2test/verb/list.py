# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
ListVerb - Implementation of 'ros2 test list' command.

Lists all discovered launch tests in the workspace.
"""

from pathlib import Path
from typing import Dict, List

from sim_harness.ros2test.verb import VerbExtension
from sim_harness.ros2test.output.console import Console
from sim_harness.test_runner.test_registry import TestRegistry, TestInfo, TestType


class ListVerb(VerbExtension):
    """List all tests in the workspace."""

    NAME = 'list'
    DESCRIPTION = 'List available tests'

    def add_arguments(self, parser, cli_name):
        """Add verb-specific arguments."""
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
            help='Show detailed test information including paths'
        )
        parser.add_argument(
            '-w', '--workspace',
            type=Path,
            metavar='DIR',
            help='Workspace root directory'
        )
        parser.add_argument(
            '--json',
            action='store_true',
            help='Output in JSON format'
        )
        # Test type filtering
        parser.add_argument(
            '--unit',
            action='store_true',
            help='List only unit tests (C++ GTest/rtest)'
        )
        parser.add_argument(
            '--integration',
            action='store_true',
            help='List only integration tests (Python launch tests)'
        )
        parser.add_argument(
            '--type',
            choices=['launch', 'gtest', 'all'],
            default='all',
            metavar='TYPE',
            help='Filter by test type: launch, gtest, or all (default: all)'
        )

    def main(self, *, args):
        """Execute the list command."""
        use_color = not getattr(args, 'no_color', False)
        console = Console(color=use_color)

        # Determine test types based on arguments
        test_types = self._get_test_types(args)

        # Create registry and discover tests
        registry = TestRegistry(workspace_root=args.workspace)
        tests = registry.discover(
            packages=args.packages,
            pattern=args.pattern,
            test_types=test_types
        )

        if not tests:
            console.warning("No tests found.")
            return 0

        # Output JSON if requested
        if args.json:
            import json
            output = []
            for test in tests:
                output.append({
                    'name': test.name,
                    'path': str(test.path),
                    'package': test.package,
                    'description': test.description,
                    'markers': test.markers,
                    'test_type': test.test_type.name.lower() if test.test_type else None,
                })
            console.print(json.dumps(output, indent=2))
            return 0

        # Group tests by package
        by_package: Dict[str, List[TestInfo]] = {}
        for test in tests:
            pkg = test.package or "unknown"
            if pkg not in by_package:
                by_package[pkg] = []
            by_package[pkg].append(test)

        # Print header
        console.test_list_header(len(tests))

        # Print tests grouped by package
        for pkg in sorted(by_package.keys()):
            console.test_list_package(pkg)
            for test in sorted(by_package[pkg], key=lambda t: t.name):
                # Add test type badge
                type_badge = self._get_type_badge(test.test_type)
                display_name = f"{type_badge} {test.name}"

                console.test_list_item(
                    name=display_name,
                    path=str(test.path) if args.verbose else None,
                    description=test.description if args.verbose else None,
                    markers=test.markers if args.verbose and test.markers else None,
                    verbose=args.verbose
                )

        return 0

    def _get_test_types(self, args):
        """Determine test types based on command-line arguments."""
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

    def _get_type_badge(self, test_type):
        """Get display badge for test type."""
        badges = {
            TestType.LAUNCH_TEST: '[launch]',
            TestType.GTEST: '[gtest]',
            TestType.PYTEST: '[pytest]',
        }
        return badges.get(test_type, '[?]')
