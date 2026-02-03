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


class ValidationScope:
    """
    Scoped validation result collection.

    Each test suite or test session gets its own scope, replacing the need
    for a process-global singleton. Scopes can optionally propagate results
    to a parent scope for hierarchical collection.

    Usage as a context manager::

        with ValidationScope("turtlebot3_nav") as scope:
            scope.add(ValidationResult.create("REQ-001", "Nav works", True))
            scope.export_to_json("results.json")

    Usage as a pytest fixture::

        @pytest.fixture
        def validation_scope(request):
            scope = ValidationScope(request.node.name)
            yield scope
    """

    def __init__(self, name: str, parent: Optional['ValidationScope'] = None):
        self.name = name
        self.parent = parent
        self._results: List[ValidationResult] = []
        self._lock = threading.Lock()

    def add(self, result: ValidationResult) -> None:
        """Add a result. Thread-safe. Propagates to parent if present."""
        with self._lock:
            self._results.append(result)
        if self.parent is not None:
            self.parent.add(result)

    # Alias for backward compatibility with code that calls add_result()
    add_result = add

    def clear(self) -> None:
        """Clear all results in this scope. Thread-safe."""
        with self._lock:
            self._results.clear()

    def get_results(self) -> List[ValidationResult]:
        """Get a copy of all results. Thread-safe."""
        with self._lock:
            return list(self._results)

    def get_counts(self) -> Tuple[int, int]:
        """Get (passed_count, failed_count). Thread-safe."""
        with self._lock:
            passed = sum(1 for r in self._results if r.passed)
            failed = sum(1 for r in self._results if not r.passed)
            return passed, failed

    def export_to_json(self, filepath: str) -> bool:
        """Export results to a JSON file. Returns True on success."""
        try:
            with self._lock:
                passed = sum(1 for r in self._results if r.passed)
                failed = sum(1 for r in self._results if not r.passed)
                data = {
                    "scope": self.name,
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
        """Print a summary to stdout with ANSI colors."""
        GREEN = '\033[92m'
        RED = '\033[91m'
        YELLOW = '\033[93m'
        BOLD = '\033[1m'
        RESET = '\033[0m'

        passed, failed = self.get_counts()
        total = passed + failed

        print()
        print(f"{BOLD}{'=' * 60}{RESET}")
        print(f"{BOLD}VALIDATION RESULTS SUMMARY ({self.name}){RESET}")
        print(f"{'=' * 60}")
        print(f"Total: {total}")
        print(f"{GREEN}Passed: {passed}{RESET}")
        print(f"{RED}Failed: {failed}{RESET}")
        print()

        with self._lock:
            failed_results = [r for r in self._results if not r.passed]

        if failed_results:
            print(f"{RED}{BOLD}Failed Requirements:{RESET}")
            for r in failed_results:
                print(f"  {RED}[FAIL]{RESET} {r.requirement_id}: {r.description}")
                if r.details:
                    print(f"         {YELLOW}Details: {r.details}{RESET}")
            print()

        print(f"{'=' * 60}")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


class ValidationResultCollector:
    """
    Backward-compatible collector that delegates to a thread-local
    ValidationScope.

    New code should prefer using ``ValidationScope`` directly. This class
    exists so that existing code calling ``ValidationResultCollector.instance()``
    continues to work, but results are now scoped per-thread by default
    rather than being truly process-global.

    To set a custom scope for the current thread::

        scope = ValidationScope("my_suite")
        ValidationResultCollector.set_scope(scope)
    """

    _thread_local = threading.local()
    _lock = threading.Lock()

    @classmethod
    def set_scope(cls, scope: ValidationScope) -> None:
        """Set the ValidationScope for the current thread."""
        cls._thread_local.scope = scope

    @classmethod
    def _get_scope(cls) -> ValidationScope:
        """Get or create the scope for the current thread."""
        if not hasattr(cls._thread_local, 'scope'):
            cls._thread_local.scope = ValidationScope("default")
        return cls._thread_local.scope

    @classmethod
    def instance(cls) -> 'ValidationResultCollector':
        """
        Get the collector instance (backward-compatible API).

        Returns a collector that delegates to the current thread's scope.
        """
        # Return a singleton wrapper that delegates to the thread-local scope
        if not hasattr(cls, '_wrapper'):
            with cls._lock:
                if not hasattr(cls, '_wrapper'):
                    cls._wrapper = cls.__new__(cls)
        return cls._wrapper

    def add_result(self, result: ValidationResult) -> None:
        """Add a validation result to the current thread's scope."""
        self._get_scope().add(result)

    def clear(self) -> None:
        """Clear results in the current thread's scope."""
        self._get_scope().clear()

    def get_results(self) -> List[ValidationResult]:
        """Get results from the current thread's scope."""
        return self._get_scope().get_results()

    def get_counts(self) -> Tuple[int, int]:
        """Get (passed, failed) counts from the current thread's scope."""
        return self._get_scope().get_counts()

    def export_to_json(self, filepath: str) -> bool:
        """Export results from the current thread's scope to JSON."""
        return self._get_scope().export_to_json(filepath)

    def print_summary(self) -> None:
        """Print summary of the current thread's scope."""
        self._get_scope().print_summary()
