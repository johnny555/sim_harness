# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Console output utilities for ros2 test CLI.

Provides colored output and formatted test result display.
"""

import sys
from typing import List, Optional


class Console:
    """
    Colored console output for test results.

    Provides formatted output with optional color support for terminal output.
    """

    # ANSI color codes
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    CYAN = '\033[36m'
    GRAY = '\033[90m'

    def __init__(self, color: bool = True, file=None):
        """
        Initialize console output.

        Args:
            color: Whether to use colored output.
            file: Output file (defaults to sys.stdout).
        """
        self._file = file or sys.stdout
        self._use_color = color and self._supports_color()

    def _supports_color(self) -> bool:
        """Check if the output supports color."""
        if not hasattr(self._file, 'isatty'):
            return False
        return self._file.isatty()

    def _colorize(self, text: str, color: str) -> str:
        """Apply color to text if color is enabled."""
        if self._use_color:
            return f"{color}{text}{self.RESET}"
        return text

    def print(self, message: str = '', end: str = '\n') -> None:
        """Print a message."""
        print(message, end=end, file=self._file, flush=True)

    def info(self, message: str) -> None:
        """Print an info message."""
        self.print(message)

    def success(self, message: str) -> None:
        """Print a success message in green."""
        self.print(self._colorize(message, self.GREEN))

    def error(self, message: str) -> None:
        """Print an error message in red."""
        self.print(self._colorize(message, self.RED))

    def warning(self, message: str) -> None:
        """Print a warning message in yellow."""
        self.print(self._colorize(message, self.YELLOW))

    def header(self, message: str) -> None:
        """Print a bold header."""
        self.print(self._colorize(message, self.BOLD))

    def dim(self, message: str) -> None:
        """Print a dimmed message."""
        self.print(self._colorize(message, self.GRAY))

    def test_pass(self, test_name: str, duration: float) -> None:
        """Print a passing test result."""
        icon = self._colorize('PASS', self.GREEN)
        self.print(f"  {icon}  {test_name} ({duration:.1f}s)")

    def test_fail(self, test_name: str, duration: float) -> None:
        """Print a failing test result."""
        icon = self._colorize('FAIL', self.RED)
        self.print(f"  {icon}  {test_name} ({duration:.1f}s)")

    def test_error(self, test_name: str, duration: float) -> None:
        """Print a test error result."""
        icon = self._colorize('ERROR', self.RED + self.BOLD)
        self.print(f"  {icon} {test_name} ({duration:.1f}s)")

    def test_timeout(self, test_name: str, duration: float) -> None:
        """Print a test timeout result."""
        icon = self._colorize('TIMEOUT', self.YELLOW)
        self.print(f"  {icon} {test_name} ({duration:.1f}s)")

    def test_skip(self, test_name: str) -> None:
        """Print a skipped test result."""
        icon = self._colorize('SKIP', self.GRAY)
        self.print(f"  {icon} {test_name}")

    def test_running(self, test_name: str, index: int, total: int) -> None:
        """Print a test running indicator."""
        self.print(f"[{index}/{total}] Running {test_name}...", end=' ')

    def divider(self, char: str = '=', width: int = 60) -> None:
        """Print a divider line."""
        self.print(char * width)

    def summary(
        self,
        total: int,
        passed: int,
        failed: int,
        errors: int,
        timeouts: int,
        duration: float
    ) -> None:
        """Print test summary."""
        self.print()
        self.divider()
        self.header("TEST SUMMARY")
        self.divider()

        self.print(f"Total:    {total}")

        if passed > 0:
            self.print(f"Passed:   {self._colorize(str(passed), self.GREEN)}")
        else:
            self.print(f"Passed:   {passed}")

        if failed > 0:
            self.print(f"Failed:   {self._colorize(str(failed), self.RED)}")
        else:
            self.print(f"Failed:   {failed}")

        if errors > 0:
            self.print(f"Errors:   {self._colorize(str(errors), self.RED)}")
        else:
            self.print(f"Errors:   {errors}")

        if timeouts > 0:
            self.print(f"Timeouts: {self._colorize(str(timeouts), self.YELLOW)}")
        else:
            self.print(f"Timeouts: {timeouts}")

        self.print(f"Duration: {duration:.1f}s")
        self.print()

        # Overall result
        if failed == 0 and errors == 0 and timeouts == 0:
            self.success("RESULT: PASSED")
        else:
            self.error("RESULT: FAILED")

    def list_failures(
        self,
        failures: List[tuple],
        show_details: bool = False
    ) -> None:
        """
        Print list of failed tests.

        Args:
            failures: List of (name, error_message) tuples.
            show_details: Whether to show error details.
        """
        if not failures:
            return

        self.print()
        self.error("FAILURES:")
        for name, error in failures:
            self.print(f"  - {name}")
            if show_details and error:
                # Show first line of error
                err_line = error.strip().split('\n')[0][:80]
                self.dim(f"      {err_line}")

    def test_list_header(self, count: int) -> None:
        """Print header for test listing."""
        self.print(f"Found {count} launch test(s):")

    def test_list_package(self, package: str) -> None:
        """Print package header in test listing."""
        self.print()
        self.header(f"[{package}]")

    def test_list_item(
        self,
        name: str,
        path: Optional[str] = None,
        description: Optional[str] = None,
        markers: Optional[List[str]] = None,
        verbose: bool = False
    ) -> None:
        """Print a test item in the list."""
        if verbose:
            self.print(f"  {name}")
            if path:
                self.dim(f"    Path: {path}")
            if description:
                self.dim(f"    Desc: {description}")
            if markers:
                self.dim(f"    Markers: {', '.join(markers)}")
        else:
            desc = f" - {description}" if description else ""
            self.print(f"  {name}{desc}")
