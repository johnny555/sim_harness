#!/usr/bin/env python3
"""
Format colcon test results in a pytest-style summary.

Parses JUnit/xunit XML files and displays a clean PASS/FAIL
summary grouped by package.

Usage:
    python3 format_test_results.py [build_dir]
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from dataclasses import dataclass
from typing import List


# Terminal colors
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

    @classmethod
    def disable(cls):
        """Disable colors (for non-TTY output)."""
        cls.GREEN = cls.RED = cls.YELLOW = cls.CYAN = cls.RESET = cls.BOLD = ''


@dataclass
class TestResult:
    name: str
    package: str
    tests: int
    passed: int
    failures: int
    errors: int
    skipped: int

    @property
    def failed(self) -> int:
        return self.failures + self.errors

    @property
    def status(self) -> str:
        if self.failed > 0:
            return 'FAIL'
        elif self.skipped == self.tests:
            return 'SKIP'
        return 'PASS'


def parse_junit_xml(xml_file: Path) -> dict:
    """Parse a JUnit XML file and return aggregated stats."""
    tree = ET.parse(xml_file)
    root = tree.getroot()

    tests = failures = errors = skipped = 0

    # Handle both <testsuites> and <testsuite> root elements
    if root.tag == 'testsuites':
        suites = root.findall('testsuite')
    elif root.tag == 'testsuite':
        suites = [root]
    else:
        return None

    for suite in suites:
        tests += int(suite.get('tests', 0))
        failures += int(suite.get('failures', 0))
        errors += int(suite.get('errors', 0))
        skipped += int(suite.get('skipped', suite.get('skip', 0)))

    return {
        'tests': tests,
        'failures': failures,
        'errors': errors,
        'skipped': skipped,
        'passed': tests - failures - errors - skipped,
    }


def find_test_results(build_dir: Path) -> List[TestResult]:
    """Find and parse all JUnit XML result files."""
    results = []

    for xml_file in build_dir.rglob('*.xml'):
        # Only look in test_results directories
        if 'test_results' not in str(xml_file):
            continue

        try:
            stats = parse_junit_xml(xml_file)
            if stats is None or stats['tests'] == 0:
                continue

            # Extract package name from path
            parts = xml_file.relative_to(build_dir).parts
            package = parts[0] if parts else 'unknown'

            # Clean up test name from filename
            name = xml_file.stem
            for prefix in ['test_', 'scripts_validation_', 'test_integration_']:
                if name.startswith(prefix):
                    name = name[len(prefix):]
            name = name.replace('.xunit', '').replace('.gtest', '')

            results.append(TestResult(
                name=name,
                package=package,
                **stats,
            ))
        except Exception:
            pass

    return results


def print_results(results: List[TestResult]) -> int:
    """Print formatted results. Returns total failure count."""
    if not results:
        print("No test results found")
        return 0

    # Sort by package, then name
    results.sort(key=lambda r: (r.package, r.name))

    current_pkg = None
    total_passed = 0
    total_failed = 0
    total_skipped = 0
    failed_tests = []

    for r in results:
        # Print package header
        if r.package != current_pkg:
            current_pkg = r.package
            print(f"\n{Colors.CYAN}{Colors.BOLD}{current_pkg}{Colors.RESET}")

        total_passed += r.passed
        total_failed += r.failed
        total_skipped += r.skipped

        # Format status
        if r.status == 'FAIL':
            status = f"{Colors.RED}FAIL{Colors.RESET}"
            failed_tests.append(f"{r.package}/{r.name}")
        elif r.status == 'SKIP':
            status = f"{Colors.YELLOW}SKIP{Colors.RESET}"
        else:
            status = f"{Colors.GREEN}PASS{Colors.RESET}"

        # Show counts for non-clean results
        if r.failed > 0 or r.skipped > 0:
            details = f" ({r.passed}/{r.tests} passed"
            if r.failed > 0:
                details += f", {r.failed} failed"
            if r.skipped > 0:
                details += f", {r.skipped} skipped"
            details += ")"
        else:
            details = ""

        print(f"  [{status}] {r.name}{details}")

    # Print summary
    print(f"\n{'='*50}")

    if total_failed == 0:
        print(f"{Colors.GREEN}{Colors.BOLD}ALL TESTS PASSED{Colors.RESET}")
    else:
        print(f"{Colors.RED}{Colors.BOLD}TESTS FAILED{Colors.RESET}")

    print(f"{Colors.GREEN}{total_passed} passed{Colors.RESET}, "
          f"{Colors.RED}{total_failed} failed{Colors.RESET}, "
          f"{Colors.YELLOW}{total_skipped} skipped{Colors.RESET}")

    if failed_tests:
        print(f"\n{Colors.RED}Failed:{Colors.RESET}")
        for t in failed_tests[:10]:
            print(f"  - {t}")
        if len(failed_tests) > 10:
            print(f"  ... and {len(failed_tests) - 10} more")

    return total_failed


def main():
    # Disable colors if not a TTY
    if not sys.stdout.isatty():
        Colors.disable()

    build_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('build')

    if not build_dir.exists():
        print(f"Build directory not found: {build_dir}")
        return 1

    results = find_test_results(build_dir)
    failures = print_results(results)

    return 1 if failures > 0 else 0


if __name__ == '__main__':
    sys.exit(main())
