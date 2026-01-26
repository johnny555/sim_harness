# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Verb extensions for the ros2 test command.

Provides the base class for verb implementations (list, run, failed).
"""

from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION, satisfies_version


class VerbExtension:
    """
    Base class for ros2 test verb extensions.

    Subclasses must implement:
    - add_arguments(parser): Add verb-specific arguments
    - main(*, args): Execute the verb
    """

    NAME: str = ''
    DESCRIPTION: str = ''

    def __init__(self):
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        """Add verb-specific arguments to the parser."""
        pass

    def main(self, *, args):
        """Execute the verb. Returns exit code (0 = success)."""
        raise NotImplementedError()
