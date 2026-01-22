#!/usr/bin/env python3
# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for validation result module."""

import json
import os
import tempfile
import pytest

from sim_harness.validation.validation_result import (
    ValidationResult,
    ValidationResultCollector,
)


class TestValidationResult:
    """Tests for ValidationResult dataclass."""

    def test_create_with_defaults(self):
        """Test creating ValidationResult with default values."""
        result = ValidationResult.create(
            requirement_id="REQ-001",
            description="Test requirement",
            passed=True
        )

        assert result.requirement_id == "REQ-001"
        assert result.description == "Test requirement"
        assert result.passed is True
        assert result.details == ""
        assert result.timestamp != ""  # Should have auto-generated timestamp
        assert result.category == ""

    def test_create_with_all_fields(self):
        """Test creating ValidationResult with all fields."""
        result = ValidationResult.create(
            requirement_id="REQ-002",
            description="Full test",
            passed=False,
            details="Failed because of X",
            category="Sensors",
            test_file="test_example.py",
            test_method="test_something"
        )

        assert result.requirement_id == "REQ-002"
        assert result.passed is False
        assert result.details == "Failed because of X"
        assert result.category == "Sensors"
        assert result.test_file == "test_example.py"
        assert result.test_method == "test_something"


class TestValidationResultCollector:
    """Tests for ValidationResultCollector singleton."""

    def setup_method(self):
        """Clear collector before each test."""
        ValidationResultCollector.instance().clear()

    def test_singleton(self):
        """Test that collector is a singleton."""
        collector1 = ValidationResultCollector.instance()
        collector2 = ValidationResultCollector.instance()
        assert collector1 is collector2

    def test_add_and_get_results(self):
        """Test adding and retrieving results."""
        collector = ValidationResultCollector.instance()

        result1 = ValidationResult.create("REQ-001", "Test 1", True)
        result2 = ValidationResult.create("REQ-002", "Test 2", False)

        collector.add_result(result1)
        collector.add_result(result2)

        results = collector.get_results()
        assert len(results) == 2
        assert results[0].requirement_id == "REQ-001"
        assert results[1].requirement_id == "REQ-002"

    def test_get_counts(self):
        """Test getting pass/fail counts."""
        collector = ValidationResultCollector.instance()

        collector.add_result(ValidationResult.create("REQ-001", "Pass 1", True))
        collector.add_result(ValidationResult.create("REQ-002", "Pass 2", True))
        collector.add_result(ValidationResult.create("REQ-003", "Fail 1", False))

        passed, failed = collector.get_counts()
        assert passed == 2
        assert failed == 1

    def test_clear(self):
        """Test clearing results."""
        collector = ValidationResultCollector.instance()

        collector.add_result(ValidationResult.create("REQ-001", "Test", True))
        assert len(collector.get_results()) == 1

        collector.clear()
        assert len(collector.get_results()) == 0

    def test_export_to_json(self):
        """Test exporting results to JSON file."""
        collector = ValidationResultCollector.instance()

        collector.add_result(ValidationResult.create("REQ-001", "Test 1", True, "OK"))
        collector.add_result(ValidationResult.create("REQ-002", "Test 2", False, "Failed"))

        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            filepath = f.name

        try:
            success = collector.export_to_json(filepath)
            assert success

            with open(filepath, 'r') as f:
                data = json.load(f)

            assert "timestamp" in data
            assert data["summary"]["total"] == 2
            assert data["summary"]["passed"] == 1
            assert data["summary"]["failed"] == 1
            assert len(data["results"]) == 2
        finally:
            os.unlink(filepath)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
