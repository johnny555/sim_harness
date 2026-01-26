# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
C++ test registry for discovering GTest/rtest test executables.

Scans the workspace build/ and install/ directories for C++ test
executables that can be run with GTest command-line options.
"""

import os
import re
import subprocess
from pathlib import Path
from typing import List, Optional

from sim_harness.test_runner.test_registry import TestInfo, TestType


class CppTestRegistry:
    """Discovers GTest/rtest C++ test executables."""

    # Patterns for test binary names
    TEST_PATTERNS = ['test_*', '*_test', '*_gtest', '*_rtest']

    # Directories to search within workspace
    SEARCH_DIRS = ['build', 'install']

    # Directories to skip during search
    SKIP_DIRS = {
        '.git', '.venv', '__pycache__', 'CMakeFiles', 'cmake',
        'share', 'include', 'python', 'Testing',
    }

    def __init__(self, workspace_root: Path):
        """Initialize the C++ test registry.

        Args:
            workspace_root: Root of the ROS2 workspace.
        """
        self.workspace_root = workspace_root

    def discover(self, packages: Optional[List[str]] = None) -> List[TestInfo]:
        """Discover C++ test executables.

        Args:
            packages: List of package names to search. If None, searches all.

        Returns:
            List of TestInfo objects for discovered C++ tests.
        """
        tests = []
        for search_dir in self._get_search_dirs():
            if not search_dir.exists():
                continue
            for binary in self._find_test_binaries(search_dir):
                if self._is_gtest_binary(binary):
                    info = self._extract_test_info(binary)
                    if packages is None or info.package in packages:
                        tests.append(info)
        return tests

    def _get_search_dirs(self) -> List[Path]:
        """Get directories to search for test binaries."""
        dirs = []
        for search_dir in self.SEARCH_DIRS:
            path = self.workspace_root / search_dir
            if path.exists():
                dirs.append(path)
        return dirs

    def _find_test_binaries(self, directory: Path) -> List[Path]:
        """Find executable files matching test patterns.

        Args:
            directory: Directory to search.

        Returns:
            List of paths to potential test binaries.
        """
        binaries = []

        for root, dirs, files in os.walk(directory):
            root_path = Path(root)

            # Filter out directories to skip
            dirs[:] = [d for d in dirs if d not in self.SKIP_DIRS]

            for filename in files:
                file_path = root_path / filename

                # Check if matches test patterns
                if not self._matches_test_pattern(filename):
                    continue

                # Check if executable
                if not self._is_executable(file_path):
                    continue

                binaries.append(file_path)

        return binaries

    def _matches_test_pattern(self, filename: str) -> bool:
        """Check if filename matches any test pattern.

        Args:
            filename: Name of the file to check.

        Returns:
            True if filename matches a test pattern.
        """
        for pattern in self.TEST_PATTERNS:
            if pattern.startswith('*'):
                if filename.endswith(pattern[1:]):
                    return True
            elif pattern.endswith('*'):
                if filename.startswith(pattern[:-1]):
                    return True
            elif filename == pattern:
                return True
        return False

    def _is_executable(self, path: Path) -> bool:
        """Check if a file is an executable binary.

        Args:
            path: Path to the file.

        Returns:
            True if the file is executable.
        """
        if not path.is_file():
            return False

        # Check execute permission
        if not os.access(path, os.X_OK):
            return False

        # Check if it's a binary (ELF on Linux)
        try:
            with open(path, 'rb') as f:
                header = f.read(4)
                # ELF magic number
                if header == b'\x7fELF':
                    return True
        except (OSError, IOError):
            pass

        return False

    def _is_gtest_binary(self, path: Path) -> bool:
        """Check if binary is a GTest executable.

        Verifies by running with --gtest_list_tests flag.

        Args:
            path: Path to the binary.

        Returns:
            True if binary supports GTest command-line options.
        """
        try:
            result = subprocess.run(
                [str(path), '--gtest_list_tests'],
                capture_output=True,
                text=True,
                timeout=5,
                env={**os.environ, 'ROS_DOMAIN_ID': '254'}
            )
            # GTest returns 0 and outputs test list, or specific error
            # Check for typical GTest output patterns
            if result.returncode == 0:
                return True
            # Some GTest binaries return non-zero but still have test output
            if 'Running main()' in result.stdout or result.stdout.strip():
                return True
        except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError):
            pass
        return False

    def _extract_test_info(self, binary: Path) -> TestInfo:
        """Extract test metadata from binary.

        Args:
            binary: Path to the test binary.

        Returns:
            TestInfo object with test metadata.
        """
        # Determine if it's rtest based on naming
        is_rtest = '_rtest' in binary.name or binary.name.startswith('rtest_')
        markers = ['cpp', 'unit']
        if is_rtest:
            markers.append('rtest')
        else:
            markers.append('gtest')

        return TestInfo(
            name=binary.stem,
            path=binary,
            package=self._get_package_name(binary),
            description=self._get_test_description(binary),
            markers=markers,
            test_type=TestType.GTEST,
        )

    def _get_package_name(self, binary: Path) -> Optional[str]:
        """Extract package name from binary path.

        Args:
            binary: Path to the test binary.

        Returns:
            Package name if determinable, None otherwise.
        """
        # Try to extract from path structure
        # Common patterns:
        #   build/<package>/test_*
        #   install/lib/<package>/test_*

        parts = binary.parts
        try:
            # Check for build/<package> pattern
            if 'build' in parts:
                build_idx = parts.index('build')
                if build_idx + 1 < len(parts):
                    return parts[build_idx + 1]

            # Check for install/lib/<package> pattern
            if 'lib' in parts:
                lib_idx = parts.index('lib')
                if lib_idx + 1 < len(parts):
                    return parts[lib_idx + 1]
        except (ValueError, IndexError):
            pass

        return None

    def _get_test_description(self, binary: Path) -> str:
        """Get a description for the test binary.

        Args:
            binary: Path to the test binary.

        Returns:
            Description string.
        """
        # Try to get test count from --gtest_list_tests
        try:
            result = subprocess.run(
                [str(binary), '--gtest_list_tests'],
                capture_output=True,
                text=True,
                timeout=5,
                env={**os.environ, 'ROS_DOMAIN_ID': '254'}
            )
            if result.returncode == 0:
                # Count test cases
                lines = result.stdout.strip().split('\n')
                test_count = sum(1 for line in lines if line.startswith('  '))
                if test_count > 0:
                    return f"C++ test with {test_count} test case(s)"
        except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError):
            pass

        return "C++ test executable"

    def list_test_cases(self, binary: Path) -> List[str]:
        """Get test case names from binary.

        Args:
            binary: Path to the test binary.

        Returns:
            List of test case names in TestSuite.TestCase format.
        """
        test_cases = []
        try:
            result = subprocess.run(
                [str(binary), '--gtest_list_tests'],
                capture_output=True,
                text=True,
                timeout=10,
                env={**os.environ, 'ROS_DOMAIN_ID': '254'}
            )
            if result.returncode == 0:
                # Parse output format:
                # TestSuite.
                #   TestCase1
                #   TestCase2
                current_suite = ""
                for line in result.stdout.split('\n'):
                    if line and not line.startswith(' '):
                        # Test suite line (ends with .)
                        current_suite = line.rstrip('.')
                    elif line.startswith('  '):
                        # Test case line
                        test_name = line.strip()
                        if current_suite:
                            test_cases.append(f"{current_suite}.{test_name}")
                        else:
                            test_cases.append(test_name)
        except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError):
            pass
        return test_cases
