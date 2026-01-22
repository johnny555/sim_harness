# Validation API

Classes for requirement validation and result collection.

## ValidationResult

A single requirement validation result.

```python
from sim_harness.validation import ValidationResult

result = ValidationResult(
    req_id="REQ-SEN-001",
    description="LIDAR sensor publishes scan data",
    passed=True,
    details="Received 150 messages in 5 seconds",
    category="Sensors",
    test_name="test_lidar_publishes"
)
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `req_id` | str | Unique requirement identifier |
| `description` | str | Human-readable description |
| `passed` | bool | Whether validation passed |
| `details` | str | Additional context/measurements |
| `category` | str | Category for grouping |
| `test_name` | str | Name of the test that validated |
| `timestamp` | datetime | When validation occurred |

### Methods

#### to_dict()

Convert to dictionary for JSON serialization.

```python
data = result.to_dict()
# {
#     "req_id": "REQ-SEN-001",
#     "description": "LIDAR sensor publishes scan data",
#     "passed": True,
#     "details": "Received 150 messages",
#     "category": "Sensors",
#     "test_name": "test_lidar_publishes",
#     "timestamp": "2026-01-22T10:30:00Z"
# }
```

**Returns:** `dict`

---

## ValidationResultCollector

Singleton collector for all validation results in a test session.

```python
from sim_harness import ValidationResultCollector

collector = ValidationResultCollector.instance()
```

### instance()

Get the singleton instance.

```python
collector = ValidationResultCollector.instance()
```

**Returns:** `ValidationResultCollector`

---

### add_result(result)

Add a validation result.

```python
result = ValidationResult(
    req_id="REQ-001",
    description="Test requirement",
    passed=True,
    details="Details here",
    category="Category"
)
collector.add_result(result)
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `result` | ValidationResult | The result to add |

---

### get_results()

Get all collected results.

```python
results = collector.get_results()
for r in results:
    status = "PASS" if r.passed else "FAIL"
    print(f"[{status}] {r.req_id}: {r.description}")
```

**Returns:** `List[ValidationResult]`

---

### get_results_by_category(category)

Get results filtered by category.

```python
sensor_results = collector.get_results_by_category("Sensors")
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `category` | str | Category to filter by |

**Returns:** `List[ValidationResult]`

---

### get_summary()

Get summary statistics.

```python
summary = collector.get_summary()
# {
#     "total": 10,
#     "passed": 8,
#     "failed": 2,
#     "skipped": 0,
#     "pass_rate": 0.8
# }
```

**Returns:** `dict`

---

### print_summary()

Print a formatted summary to stdout.

```python
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
[FAIL] REQ-SEN-002: IMU publishes at 100Hz
       Details: Measured rate: 85Hz

================================================================================
SUMMARY: 8 passed, 2 failed, 0 skipped
================================================================================
```

---

### export_to_json(path)

Export all results to a JSON file.

```python
collector.export_to_json("/tmp/validation_results.json")
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | str | Output file path |

JSON structure:
```json
{
  "timestamp": "2026-01-22T10:30:00Z",
  "summary": {
    "total": 10,
    "passed": 8,
    "failed": 2,
    "skipped": 0
  },
  "results": [
    {
      "req_id": "REQ-SEN-001",
      "description": "LIDAR sensor publishes scan data",
      "passed": true,
      "details": "Received 150 messages",
      "category": "Sensors",
      "test_name": "test_lidar_publishes",
      "timestamp": "2026-01-22T10:29:45Z"
    }
  ]
}
```

---

### export_to_csv(path)

Export results to a CSV file.

```python
collector.export_to_csv("/tmp/validation_results.csv")
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | str | Output file path |

---

### clear()

Clear all collected results.

```python
collector.clear()
```

---

## RequirementValidator

Helper class for validating requirements within tests.

```python
from sim_harness.validation import RequirementValidator

validator = RequirementValidator(collector)
```

### validate(req_id, description, passed, details, category)

Validate a requirement and add to collector.

```python
validator.validate(
    req_id="REQ-001",
    description="System does X",
    passed=True,
    details="X completed successfully",
    category="Functional"
)
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `req_id` | str | Requirement identifier |
| `description` | str | Requirement description |
| `passed` | bool | Whether validation passed |
| `details` | str | Additional details |
| `category` | str | Category for grouping |

---

### assert_requirement(req_id, description, passed, details, category)

Validate and assert (raises AssertionError if failed).

```python
validator.assert_requirement(
    req_id="REQ-001",
    description="System does X",
    passed=result,  # If False, raises AssertionError
    details="Details",
    category="Functional"
)
```

**Parameters:** Same as `validate()`

**Raises:** `AssertionError` if `passed` is False
