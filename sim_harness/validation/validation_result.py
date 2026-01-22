# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Validation result storage and reporting.

Provides requirements traceability by mapping test results back to
specification requirements.
"""

import json
import threading
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import List, Optional, Tuple


@dataclass
class ValidationResult:
    """
    Stores the result of a single requirement validation.

    Used for requirements traceability - mapping test results back to
    specification requirements.
    """

    requirement_id: str
    """Requirement ID (e.g., "REQ-001", "Matt-SYSRQ-52")."""

    description: str
    """Human-readable description of the requirement."""

    passed: bool
    """Whether the requirement was satisfied."""

    details: str = ""
    """Additional details about the validation."""

    timestamp: str = ""
    """ISO 8601 timestamp of when the validation occurred."""

    test_file: str = ""
    """Source file where the test was defined."""

    test_method: str = ""
    """Test method name."""

    category: str = ""
    """Category for grouping (e.g., "Sensors", "Navigation")."""

    @staticmethod
    def create(
        requirement_id: str,
        description: str,
        passed: bool,
        details: str = "",
        category: str = "",
        test_file: str = "",
        test_method: str = ""
    ) -> 'ValidationResult':
        """
        Create a ValidationResult with auto-generated timestamp.

        Args:
            requirement_id: Requirement ID
            description: Description
            passed: Whether passed
            details: Optional details
            category: Optional category
            test_file: Optional test file path
            test_method: Optional test method name

        Returns:
            ValidationResult with current timestamp
        """
        return ValidationResult(
            requirement_id=requirement_id,
            description=description,
            passed=passed,
            details=details,
            timestamp=datetime.now().isoformat(),
            test_file=test_file,
            test_method=test_method,
            category=category
        )


class ValidationResultCollector:
    """
    Singleton collector for validation results.

    Accumulates validation results across all tests and provides
    export functionality for traceability reports.
    """

    _instance: Optional['ValidationResultCollector'] = None
    _lock = threading.Lock()

    def __new__(cls) -> 'ValidationResultCollector':
        """Ensure singleton pattern."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._results: List[ValidationResult] = []
                    cls._instance._results_lock = threading.Lock()
        return cls._instance

    @classmethod
    def instance(cls) -> 'ValidationResultCollector':
        """
        Get the singleton instance.

        Returns:
            Reference to the global collector
        """
        return cls()

    def add_result(self, result: ValidationResult) -> None:
        """
        Add a validation result.

        Thread-safe.

        Args:
            result: The result to add
        """
        with self._results_lock:
            self._results.append(result)

    def clear(self) -> None:
        """Clear all collected results. Thread-safe."""
        with self._results_lock:
            self._results.clear()

    def get_results(self) -> List[ValidationResult]:
        """
        Get all collected results.

        Thread-safe.

        Returns:
            Copy of all results
        """
        with self._results_lock:
            return list(self._results)

    def get_counts(self) -> Tuple[int, int]:
        """
        Get count of passed/failed results.

        Returns:
            Tuple of (passed_count, failed_count)
        """
        with self._results_lock:
            passed = sum(1 for r in self._results if r.passed)
            failed = sum(1 for r in self._results if not r.passed)
            return passed, failed

    def export_to_json(self, filepath: str) -> bool:
        """
        Export results to a JSON file.

        JSON format:
            {
                "timestamp": "2024-01-15T10:30:00",
                "summary": {"total": 10, "passed": 8, "failed": 2},
                "results": [...]
            }

        Args:
            filepath: Path to write JSON file

        Returns:
            True if export succeeded
        """
        try:
            with self._results_lock:
                # Calculate counts inline to avoid deadlock (don't call get_counts())
                passed = sum(1 for r in self._results if r.passed)
                failed = sum(1 for r in self._results if not r.passed)
                data = {
                    "timestamp": datetime.now().isoformat(),
                    "summary": {
                        "total": len(self._results),
                        "passed": passed,
                        "failed": failed
                    },
                    "results": [asdict(r) for r in self._results]
                }

            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)

            return True
        except Exception as e:
            print(f"Failed to export results: {e}")
            return False

    def print_summary(self) -> None:
        """
        Print a summary to stdout.

        Includes pass/fail counts and list of failed requirements.
        Uses ANSI color codes for visibility.
        """
        # ANSI color codes
        GREEN = '\033[92m'
        RED = '\033[91m'
        YELLOW = '\033[93m'
        BOLD = '\033[1m'
        RESET = '\033[0m'

        passed, failed = self.get_counts()
        total = passed + failed

        print()
        print(f"{BOLD}{'=' * 60}{RESET}")
        print(f"{BOLD}VALIDATION RESULTS SUMMARY{RESET}")
        print(f"{'=' * 60}")
        print(f"Total: {total}")
        print(f"{GREEN}Passed: {passed}{RESET}")
        print(f"{RED}Failed: {failed}{RESET}")
        print()

        # List failed requirements
        with self._results_lock:
            failed_results = [r for r in self._results if not r.passed]

        if failed_results:
            print(f"{RED}{BOLD}Failed Requirements:{RESET}")
            for r in failed_results:
                print(f"  {RED}[FAIL]{RESET} {r.requirement_id}: {r.description}")
                if r.details:
                    print(f"         {YELLOW}Details: {r.details}{RESET}")
            print()

        print(f"{'=' * 60}")
