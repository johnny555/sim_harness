# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
TestCommand - Main entry point for 'ros2 test' command.

This module provides the ros2cli extension for running sim_harness tests.
"""

from ros2cli.command import CommandExtension, add_subparsers_on_demand


class TestCommand(CommandExtension):
    """Run and manage sim_harness tests."""

    def add_arguments(self, parser, cli_name):
        """Add command arguments."""
        parser.add_argument(
            '--no-color',
            action='store_true',
            default=False,
            help='Disable colored output'
        )
        self._subparser = parser
        add_subparsers_on_demand(
            parser,
            cli_name,
            '_verb',
            'ros2test.verb',
            required=False
        )

    def main(self, *, parser, args):
        """Execute the command."""
        if not hasattr(args, '_verb'):
            # No verb specified, show help
            parser.print_help()
            return 0
        extension = getattr(args, '_verb')
        return extension.main(args=args)
