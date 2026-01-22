# Core API

Core classes and functions for sim_harness.

## Test Fixtures

### SimTestFixture

Base test fixture for ROS 2 simulation tests.

```python
from sim_harness import SimTestFixture

class TestMyRobot(SimTestFixture):
    def test_something(self):
        pass
```

**Class Attributes:**

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `NODE_NAME` | str | `"test_node"` | Name for the ROS 2 node |

**Properties:**

| Property | Type | Description |
|----------|------|-------------|
| `node` | `rclpy.node.Node` | The ROS 2 node instance |

**Methods:**

#### get_logger()

Get the node's logger.

```python
self.get_logger().info("Test starting")
```

#### spin_for_duration(seconds)

Spin the ROS node for the specified duration.

```python
self.spin_for_duration(5.0)  # Spin for 5 seconds
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `seconds` | float | Duration to spin in seconds |

#### create_message_collector(topic, msg_type, ...)

Create a message collector for a topic.

```python
collector = self.create_message_collector(
    "/scan",
    LaserScan,
    key="scan",
    max_messages=100,
    qos_profile=qos
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topic` | str | required | Topic name |
| `msg_type` | type | required | Message type class |
| `key` | str | None | Key for later retrieval |
| `max_messages` | int | None | Max buffer size |
| `qos_profile` | QoSProfile | None | Custom QoS |

**Returns:** `MessageCollector`

#### get_collector(key)

Retrieve a message collector by key.

```python
collector = self.get_collector("scan")
```

#### assert_requirement(...)

Record and assert a requirement validation.

```python
self.assert_requirement(
    req_id="REQ-001",
    description="System does X",
    passed=True,
    details="X completed",
    category="Functional"
)
```

#### validate_requirement(...)

Record a requirement validation without asserting.

```python
self.validate_requirement(
    req_id="REQ-001",
    description="System does X",
    passed=result,
    details="Details",
    category="Functional"
)
```

---

### SimulationTestFixture

Test fixture with automatic simulation lifecycle management.

```python
from sim_harness import SimulationTestFixture

class TestWithSim(SimulationTestFixture):
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'

    def test_something(self):
        pass
```

**Class Attributes:**

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `LAUNCH_PACKAGE` | str | required | ROS 2 package with launch file |
| `LAUNCH_FILE` | str | required | Launch file name |
| `LAUNCH_ARGS` | dict | `{}` | Launch arguments |
| `ENV_VARS` | dict | `{}` | Environment variables |
| `STARTUP_TIMEOUT` | float | `60.0` | Max startup wait (seconds) |
| `GAZEBO_STARTUP_DELAY` | float | `5.0` | Post-Gazebo delay (seconds) |
| `REQUIRE_SIM` | bool | `True` | Fail if sim can't start |
| `USE_EXISTING_SIM` | bool | `False` | Use running simulation |

Inherits all methods from `SimTestFixture`.

---

## Message Collection

### MessageCollector

Collects messages from a ROS 2 topic.

```python
from sim_harness.core import MessageCollector

collector = MessageCollector(node, "/scan", LaserScan)
# ... spin node ...
messages = collector.get_messages()
```

**Methods:**

#### get_messages()

Get all collected messages.

```python
messages = collector.get_messages()
for msg in messages:
    print(msg.header.stamp)
```

**Returns:** `List[MsgType]`

#### get_latest()

Get the most recent message.

```python
latest = collector.get_latest()
if latest:
    print(latest.ranges)
```

**Returns:** `Optional[MsgType]`

#### message_count()

Get the number of collected messages.

```python
count = collector.message_count()
```

**Returns:** `int`

#### clear()

Clear all collected messages.

```python
collector.clear()
```

#### wait_for_messages(count, timeout)

Wait for a minimum number of messages.

```python
success = collector.wait_for_messages(count=5, timeout=10.0)
```

**Returns:** `bool`

---

## Validation

### ValidationResultCollector

Singleton collector for requirement validation results.

```python
from sim_harness import ValidationResultCollector

collector = ValidationResultCollector.instance()
```

**Methods:**

#### instance()

Get the singleton instance.

```python
collector = ValidationResultCollector.instance()
```

**Returns:** `ValidationResultCollector`

#### add_result(result)

Add a validation result.

```python
from sim_harness.validation import ValidationResult

result = ValidationResult(
    req_id="REQ-001",
    description="Requirement description",
    passed=True,
    details="Test details",
    category="Category"
)
collector.add_result(result)
```

#### get_results()

Get all validation results.

```python
results = collector.get_results()
for r in results:
    print(f"{r.req_id}: {'PASS' if r.passed else 'FAIL'}")
```

**Returns:** `List[ValidationResult]`

#### print_summary()

Print a formatted summary to stdout.

```python
collector.print_summary()
```

#### export_to_json(path)

Export results to a JSON file.

```python
collector.export_to_json("/tmp/results.json")
```

#### clear()

Clear all results.

```python
collector.clear()
```

---

### ValidationResult

A single requirement validation result.

```python
from sim_harness.validation import ValidationResult

result = ValidationResult(
    req_id="REQ-SEN-001",
    description="LIDAR publishes data",
    passed=True,
    details="Received 100 messages",
    category="Sensors",
    test_name="test_lidar"
)
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `req_id` | str | Requirement identifier |
| `description` | str | Requirement description |
| `passed` | bool | Whether validation passed |
| `details` | str | Additional details |
| `category` | str | Category for grouping |
| `test_name` | str | Name of the test |
| `timestamp` | datetime | When validation occurred |
