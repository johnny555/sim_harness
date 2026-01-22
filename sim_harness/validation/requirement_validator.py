# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Requirement validation mixin for test classes.

Provides methods to validate requirements and track results.
"""

import inspect
import os
from typing import Optional

from sim_harness.validation.validation_result import (
    ValidationResult,
    ValidationResultCollector,
)


class RequirementValidator:
    """
    Mixin class for requirement-based test validation.

    Add this as a base class to your test fixture to enable
    requirement tracking:

        class TestMyRobot(SimTestFixture, RequirementValidator):
            def test_sensor(self):
                result = check_sensor()
                self.validate_requirement(
                    "REQ-001",
                    "Sensor publishes data",
                    result.valid,
                    result.details,
                    "Sensors"
                )
    """

    def validate_requirement(
        self,
        requirement_id: str,
        description: str,
        passed: bool,
        details: str = "",
        category: str = ""
    ) -> None:
        """
        Validate a requirement (non-fatal).

        Records the result and prints status, but does not fail the test.
        Use assert_requirement() for fatal assertions.

        Args:
            requirement_id: Requirement ID (e.g., "REQ-001")
            description: Human-readable description
            passed: Whether the requirement was satisfied
            details: Optional additional details
            category: Optional category for grouping
        """
        # Get caller info for traceability
        frame = inspect.currentframe()
        caller_frame = frame.f_back if frame else None

        test_file = ""
        test_method = ""
        if caller_frame:
            test_file = os.path.basename(caller_frame.f_code.co_filename)
            test_method = caller_frame.f_code.co_name

        result = ValidationResult.create(
            requirement_id=requirement_id,
            description=description,
            passed=passed,
            details=details,
            category=category,
            test_file=test_file,
            test_method=test_method
        )

        # Add to collector
        ValidationResultCollector.instance().add_result(result)

        # Print status
        status = "\033[92m[PASS]\033[0m" if passed else "\033[91m[FAIL]\033[0m"
        print(f"{status} {requirement_id}: {description}")
        if details and not passed:
            print(f"       Details: {details}")

    def assert_requirement(
        self,
        requirement_id: str,
        description: str,
        passed: bool,
        details: str = "",
        category: str = ""
    ) -> None:
        """
        Assert a requirement (fatal with pytest assert).

        Records the result and raises AssertionError if failed.

        Args:
            requirement_id: Requirement ID (e.g., "REQ-001")
            description: Human-readable description
            passed: Whether the requirement was satisfied
            details: Optional additional details
            category: Optional category for grouping

        Raises:
            AssertionError: If the requirement was not satisfied
        """
        self.validate_requirement(
            requirement_id, description, passed, details, category
        )
        assert passed, f"{requirement_id}: {description} - {details}"

    def assert_requirement_fatal(
        self,
        requirement_id: str,
        description: str,
        passed: bool,
        details: str = "",
        category: str = ""
    ) -> None:
        """
        Assert a requirement with pytest.fail (stops test immediately).

        Records the result and calls pytest.fail() if failed,
        stopping the test immediately.

        Args:
            requirement_id: Requirement ID
            description: Human-readable description
            passed: Whether the requirement was satisfied
            details: Optional additional details
            category: Optional category for grouping
        """
        import pytest

        self.validate_requirement(
            requirement_id, description, passed, details, category
        )
        if not passed:
            pytest.fail(f"{requirement_id}: {description} - {details}")
