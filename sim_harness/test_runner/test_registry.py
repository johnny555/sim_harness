# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Test registry for discovering launch_testing tests in the workspace.

Scans the workspace src/ directory for Python files containing
`generate_test_description()` functions, which is the marker for
launch_testing tests.
"""

import ast
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Dict


@dataclass
class TestInfo:
    """Information about a discovered test."""

    name: str
    path: Path
    package: Optional[str] = None
    description: str = ""
    markers: List[str] = field(default_factory=list)
    timeout: int = 300  # Default timeout in seconds

    def __str__(self) -> str:
        pkg = f"[{self.package}]" if self.package else ""
        return f"{pkg} {self.name}: {self.path}"

    @property
    def short_name(self) -> str:
        """Return short name without extension."""
        return self.path.stem


class TestRegistry:
    """Registry for discovering and managing launch tests."""

    # Directories to skip during discovery
    SKIP_DIRS = {
        '.git', '.venv', 'venv', '__pycache__', 'build', 'install', 'log',
        'node_modules', '.mypy_cache', '.pytest_cache', 'site-packages',
        'dist-packages', '.tox', 'eggs', '*.egg-info',
    }

    # Patterns that indicate a test file
    TEST_PATTERNS = [
        'test_*.py',
        '*_test.py',
        '*_launch_test.py',
    ]

    def __init__(self, workspace_root: Optional[Path] = None):
        """Initialize the registry.

        Args:
            workspace_root: Root of the ROS2 workspace. If None, attempts
                           to find it from current directory.
        """
        self.workspace_root = workspace_root or self._find_workspace_root()
        self.tests: Dict[str, TestInfo] = {}

    def _find_workspace_root(self) -> Path:
        """Find workspace root by looking for src/ directory."""
        cwd = Path.cwd()

        # Check if we're in a workspace
        if (cwd / 'src').is_dir():
            return cwd

        # Walk up to find workspace
        for parent in cwd.parents:
            if (parent / 'src').is_dir():
                return parent

        # Fallback to current directory
        return cwd

    def _should_skip_dir(self, dir_path: Path) -> bool:
        """Check if directory should be skipped."""
        name = dir_path.name
        for skip in self.SKIP_DIRS:
            if skip.startswith('*'):
                if name.endswith(skip[1:]):
                    return True
            elif name == skip:
                return True
        return False

    def _get_package_name(self, test_path: Path) -> Optional[str]:
        """Extract package name from test path.

        Looks for package.xml in parent directories.
        """
        for parent in test_path.parents:
            package_xml = parent / 'package.xml'
            if package_xml.exists():
                try:
                    with open(package_xml) as f:
                        content = f.read()
                        # Simple XML parsing for package name
                        import re
                        match = re.search(r'<name>([^<]+)</name>', content)
                        if match:
                            return match.group(1)
                except Exception:
                    pass
                # Fallback to directory name
                return parent.name
        return None

    def _is_launch_test(self, file_path: Path) -> bool:
        """Check if file contains generate_test_description()."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()

            # Quick check before full parsing
            if 'generate_test_description' not in content:
                return False

            # Parse AST to verify it's a proper function definition
            tree = ast.parse(content)
            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef):
                    if node.name == 'generate_test_description':
                        return True

        except (SyntaxError, UnicodeDecodeError, OSError):
            pass

        return False

    def _extract_test_info(self, file_path: Path) -> Optional[TestInfo]:
        """Extract test information from a test file."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()

            tree = ast.parse(content)

            # Extract docstring
            description = ""
            if (tree.body and isinstance(tree.body[0], ast.Expr) and
                isinstance(tree.body[0].value, ast.Constant)):
                description = tree.body[0].value.value.split('\n')[0].strip()

            # Extract markers (pytest.mark.*)
            markers = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute):
                    if (hasattr(node, 'attr') and
                        isinstance(node.value, ast.Attribute)):
                        if (hasattr(node.value, 'attr') and
                            node.value.attr == 'mark'):
                            markers.append(node.attr)

            return TestInfo(
                name=file_path.stem,
                path=file_path,
                package=self._get_package_name(file_path),
                description=description,
                markers=markers,
            )

        except Exception:
            return TestInfo(
                name=file_path.stem,
                path=file_path,
                package=self._get_package_name(file_path),
            )

    def discover(self,
                 packages: Optional[List[str]] = None,
                 pattern: Optional[str] = None) -> List[TestInfo]:
        """Discover all launch tests in the workspace.

        Args:
            packages: List of package names to search. If None, searches all.
            pattern: Glob pattern to filter tests (e.g., "test_nav*")

        Returns:
            List of TestInfo objects for discovered tests.
        """
        self.tests.clear()
        src_dir = self.workspace_root / 'src'

        if not src_dir.exists():
            return []

        # Walk the source directory
        for root, dirs, files in os.walk(src_dir):
            root_path = Path(root)

            # Filter out directories to skip
            dirs[:] = [d for d in dirs if not self._should_skip_dir(root_path / d)]

            for file in files:
                if not file.endswith('.py'):
                    continue

                # Check if file matches test patterns
                is_test_file = any(
                    file.startswith(p.replace('*', '')) or
                    file.endswith(p.replace('*', ''))
                    for p in ['test_', '_test.py', '_launch_test.py']
                )

                if not is_test_file:
                    continue

                file_path = root_path / file

                # Apply pattern filter
                if pattern and not file_path.match(pattern):
                    continue

                # Check if it's a launch test
                if not self._is_launch_test(file_path):
                    continue

                # Extract test info
                test_info = self._extract_test_info(file_path)
                if test_info:
                    # Apply package filter
                    if packages and test_info.package not in packages:
                        continue

                    # Use relative path as key
                    rel_path = file_path.relative_to(self.workspace_root)
                    self.tests[str(rel_path)] = test_info

        return list(self.tests.values())

    def get_test(self, name_or_path: str) -> Optional[TestInfo]:
        """Get a test by name or path.

        Args:
            name_or_path: Test name (e.g., "test_robot_navigation") or
                         relative path (e.g., "src/my_package/test/test_robot_navigation.py")

        Returns:
            TestInfo if found, None otherwise.
        """
        # Try direct path match
        if name_or_path in self.tests:
            return self.tests[name_or_path]

        # Try matching by name
        for key, info in self.tests.items():
            if info.name == name_or_path or info.short_name == name_or_path:
                return info

        # Try partial path match
        for key, info in self.tests.items():
            if name_or_path in key:
                return info

        return None

    def list_tests(self,
                   packages: Optional[List[str]] = None,
                   verbose: bool = False) -> str:
        """List all tests in a formatted string.

        Args:
            packages: Filter by package names.
            verbose: Include full paths and descriptions.

        Returns:
            Formatted string listing all tests.
        """
        tests = self.discover(packages=packages)

        if not tests:
            return "No launch tests found."

        # Group by package
        by_package: Dict[str, List[TestInfo]] = {}
        for test in tests:
            pkg = test.package or "unknown"
            if pkg not in by_package:
                by_package[pkg] = []
            by_package[pkg].append(test)

        lines = [f"Found {len(tests)} launch test(s):\n"]

        for pkg in sorted(by_package.keys()):
            lines.append(f"\n[{pkg}]")
            for test in sorted(by_package[pkg], key=lambda t: t.name):
                if verbose:
                    lines.append(f"  {test.name}")
                    lines.append(f"    Path: {test.path}")
                    if test.description:
                        lines.append(f"    Desc: {test.description}")
                    if test.markers:
                        lines.append(f"    Markers: {', '.join(test.markers)}")
                else:
                    desc = f" - {test.description}" if test.description else ""
                    lines.append(f"  {test.name}{desc}")

        return '\n'.join(lines)


def discover_tests(workspace_root: Optional[Path] = None,
                   packages: Optional[List[str]] = None,
                   pattern: Optional[str] = None) -> List[TestInfo]:
    """Convenience function to discover tests.

    Args:
        workspace_root: Root of the ROS2 workspace.
        packages: List of package names to search.
        pattern: Glob pattern to filter tests.

    Returns:
        List of TestInfo objects.
    """
    registry = TestRegistry(workspace_root)
    return registry.discover(packages=packages, pattern=pattern)
