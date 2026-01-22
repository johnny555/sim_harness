# Requirement Tracing

sim_harness supports mapping tests to requirements for traceability and compliance reporting.

## Basic Usage

Use `assert_requirement` to record requirement validation:

```python
from sim_harness import SimTestFixture

class TestSensors(SimTestFixture):
    def test_lidar_operational(self):
        collector = self.create_message_collector("/scan", LaserScan)
        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        passed = len(messages) > 0

        self.assert_requirement(
            req_id="REQ-SEN-001",
            description="LIDAR sensor publishes scan data",
            passed=passed,
            details=f"Received {len(messages)} messages",
            category="Sensors"
        )

        assert passed, "LIDAR not publishing"
```

## Methods

### assert_requirement

Records the validation **and** asserts (test fails if `passed=False`):

```python
self.assert_requirement(
    req_id="REQ-001",
    description="System shall do X",
    passed=True,
    details="X was done successfully",
    category="Functional"
)
```

### validate_requirement

Records the validation **without** asserting (test continues regardless):

```python
self.validate_requirement(
    req_id="REQ-001",
    description="System shall do X",
    passed=result,
    details="Details here",
    category="Functional"
)

# Test continues even if result was False
# Useful for collecting all results before final assertion
```

## Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `req_id` | str | Unique requirement identifier (e.g., "REQ-SEN-001") |
| `description` | str | Human-readable requirement description |
| `passed` | bool | Whether the requirement was satisfied |
| `details` | str | Additional context or measurements |
| `category` | str | Category for grouping (e.g., "Sensors", "Navigation") |

## Validation Summary

At the end of your test session, print a summary:

```python
import pytest
from sim_harness import ValidationResultCollector

@pytest.fixture(scope="session", autouse=True)
def print_validation_summary():
    yield  # Run all tests first
    collector = ValidationResultCollector.instance()
    collector.print_summary()
```

Output:

```
================================================================================
REQUIREMENT VALIDATION SUMMARY
================================================================================

Category: Sensors
--------------------------------------------------------------------------------
[PASS] REQ-SEN-001: LIDAR sensor publishes scan data
       Details: Received 150 messages
[PASS] REQ-SEN-002: Odometry publishes pose data
       Details: Received 300 messages
[FAIL] REQ-SEN-003: IMU publishes at 100Hz
       Details: Measured rate: 85Hz

Category: Navigation
--------------------------------------------------------------------------------
[PASS] REQ-NAV-001: Robot can navigate to goal
       Details: Reached goal in 12.3s

================================================================================
SUMMARY: 3 passed, 1 failed, 0 skipped
================================================================================
```

## JSON Export

Export results for CI/CD or documentation:

```python
@pytest.fixture(scope="session", autouse=True)
def export_results():
    yield
    collector = ValidationResultCollector.instance()
    collector.export_to_json("/tmp/validation_results.json")
```

JSON format:

```json
{
  "timestamp": "2026-01-22T10:30:00Z",
  "summary": {
    "total": 4,
    "passed": 3,
    "failed": 1,
    "skipped": 0
  },
  "results": [
    {
      "req_id": "REQ-SEN-001",
      "description": "LIDAR sensor publishes scan data",
      "passed": true,
      "details": "Received 150 messages",
      "category": "Sensors",
      "test_name": "test_lidar_operational",
      "timestamp": "2026-01-22T10:29:45Z"
    }
  ]
}
```

## Organizing Requirements

### By Category

```python
class TestSensors(SimTestFixture):
    """All sensor-related requirements."""

    def test_lidar(self):
        self.assert_requirement("REQ-SEN-001", ..., category="Sensors")

    def test_imu(self):
        self.assert_requirement("REQ-SEN-002", ..., category="Sensors")


class TestNavigation(SimTestFixture):
    """All navigation-related requirements."""

    def test_pathfinding(self):
        self.assert_requirement("REQ-NAV-001", ..., category="Navigation")
```

### Multiple Requirements per Test

```python
def test_sensor_integration(self):
    """Test that validates multiple requirements."""
    # Check LIDAR
    scan_ok = self.check_lidar()
    self.validate_requirement(
        "REQ-SEN-001", "LIDAR operational", scan_ok, "", "Sensors"
    )

    # Check odometry
    odom_ok = self.check_odometry()
    self.validate_requirement(
        "REQ-SEN-002", "Odometry operational", odom_ok, "", "Sensors"
    )

    # Check fusion
    fusion_ok = scan_ok and odom_ok
    self.validate_requirement(
        "REQ-SEN-010", "Sensor fusion possible", fusion_ok, "", "Sensors"
    )

    # Final assertion
    assert fusion_ok, "Sensor integration failed"
```

## CI/CD Integration

### GitHub Actions Example

```yaml
- name: Run tests with requirement tracing
  run: |
    pytest tests/ -v --tb=short
    cat /tmp/validation_results.json

- name: Upload validation report
  uses: actions/upload-artifact@v3
  with:
    name: validation-report
    path: /tmp/validation_results.json
```

### Fail on Missing Requirements

```python
@pytest.fixture(scope="session", autouse=True)
def check_all_requirements():
    yield

    collector = ValidationResultCollector.instance()
    results = collector.get_results()

    # Define required requirements
    required = {"REQ-SEN-001", "REQ-SEN-002", "REQ-NAV-001"}
    validated = {r.req_id for r in results}

    missing = required - validated
    if missing:
        pytest.fail(f"Missing requirement validations: {missing}")
```

## Best Practices

!!! tip "Use consistent ID format"
    Adopt a consistent format: `REQ-{CATEGORY}-{NUMBER}`
    - `REQ-SEN-001` for sensors
    - `REQ-NAV-001` for navigation
    - `REQ-SAF-001` for safety

!!! tip "Keep descriptions requirement-focused"
    Write descriptions as requirements, not test descriptions:
    - Good: "LIDAR sensor shall publish scan data at 10Hz"
    - Bad: "Test that LIDAR publishes"

!!! tip "Include measurements in details"
    Make details useful for debugging:
    - Good: "Measured rate: 9.8Hz (required: 10Hz +/- 1Hz)"
    - Bad: "Rate OK"

!!! warning "Don't skip requirements"
    If a requirement can't be tested, mark it explicitly:
    ```python
    self.validate_requirement(
        "REQ-SAF-001",
        "Emergency stop within 100ms",
        passed=None,  # or False
        details="SKIPPED: Requires hardware testing",
        category="Safety"
    )
    ```
