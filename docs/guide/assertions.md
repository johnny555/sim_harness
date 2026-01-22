# Assertions

sim_harness provides pre-built assertion functions for common robotics testing scenarios.

## Sensor Assertions

### Topic Publishing

```python
from sim_harness import assert_topic_published
from sensor_msgs.msg import LaserScan

def test_lidar(self):
    result = assert_topic_published(
        self.node,
        "/scan",
        LaserScan,
        timeout_sec=5.0,
        min_messages=1
    )

    assert result.success, result.details
    print(f"Received {result.message_count} messages")
```

### Sensor Data Quality

```python
from sim_harness import assert_sensor_data_valid

def test_lidar_quality(self):
    result = assert_sensor_data_valid(
        self.node,
        "/scan",
        LaserScan,
        timeout_sec=5.0,
        allow_nan_ratio=0.01  # Allow 1% NaN values
    )

    assert result.success, result.details
```

## Vehicle Assertions

### Movement

```python
from sim_harness import assert_vehicle_moved

def test_robot_moves(self):
    result = assert_vehicle_moved(
        self.node,
        vehicle_id="robot_01",
        min_distance=0.5,      # Must move at least 0.5m
        velocity=0.2,          # Command velocity
        timeout_sec=10.0,
        odom_topic="/robot_01/odom",      # Optional custom topic
        cmd_vel_topic="/robot_01/cmd_vel" # Optional custom topic
    )

    assert result.success, result.details
    print(f"Robot moved {result.distance_moved:.2f}m")
```

### Stationary Check

```python
from sim_harness import assert_vehicle_stationary

def test_robot_stable(self):
    result = assert_vehicle_stationary(
        self.node,
        vehicle_id="robot_01",
        velocity_threshold=0.01,  # m/s
        duration_sec=2.0          # Must stay still for 2 seconds
    )

    assert result, "Robot is moving unexpectedly"
```

### Velocity

```python
from sim_harness import assert_vehicle_velocity

def test_robot_speed(self):
    result = assert_vehicle_velocity(
        self.node,
        vehicle_id="robot_01",
        target_velocity=0.5,
        tolerance=0.1,
        timeout_sec=5.0
    )

    assert result.success, result.details
```

### Position

```python
from sim_harness import assert_vehicle_in_region

def test_robot_position(self):
    result = assert_vehicle_in_region(
        self.node,
        vehicle_id="robot_01",
        min_bounds=(-1.0, -1.0, 0.0),  # (x, y, z)
        max_bounds=(1.0, 1.0, 0.5),
        timeout_sec=5.0
    )

    assert result, "Robot outside expected region"
```

### Orientation

```python
from sim_harness import assert_vehicle_orientation
import math

def test_robot_facing_north(self):
    result = assert_vehicle_orientation(
        self.node,
        vehicle_id="robot_01",
        expected_yaw=0.0,  # radians
        tolerance_rad=math.radians(10),  # +/- 10 degrees
        timeout_sec=5.0
    )

    assert result, "Robot not facing expected direction"
```

## Navigation Assertions

### Lifecycle Nodes

```python
from sim_harness import assert_lifecycle_node_active

def test_nav2_active(self):
    nodes = ["controller_server", "planner_server", "bt_navigator"]

    for node_name in nodes:
        result = assert_lifecycle_node_active(
            self.node,
            node_name,
            timeout_sec=10.0
        )
        assert result.success, f"{node_name}: {result.details}"
```

### Goal Reached

```python
from sim_harness import assert_goal_reached

def test_navigation(self):
    # Send navigation goal first...

    result = assert_goal_reached(
        self.node,
        goal_x=2.0,
        goal_y=1.0,
        tolerance=0.3,
        timeout_sec=60.0
    )

    assert result.success, result.details
```

## Service Assertions

### Service Available

```python
from sim_harness import assert_service_available

def test_services(self):
    result = assert_service_available(
        self.node,
        "/get_map",
        timeout_sec=5.0
    )

    assert result.success, f"Service not available: {result.details}"
```

### Service Response

```python
from sim_harness import assert_service_call_succeeds
from std_srvs.srv import Trigger

def test_service_works(self):
    result = assert_service_call_succeeds(
        self.node,
        "/trigger_action",
        Trigger,
        request=Trigger.Request(),
        timeout_sec=5.0
    )

    assert result.success, result.details
```

## Timing Assertions

### Message Rate

```python
from sim_harness import assert_message_rate

def test_lidar_rate(self):
    result = assert_message_rate(
        self.node,
        "/scan",
        LaserScan,
        expected_rate=10.0,  # Hz
        tolerance=2.0,       # +/- 2 Hz
        duration_sec=5.0
    )

    assert result.success, result.details
```

### Latency

```python
from sim_harness import assert_message_latency

def test_low_latency(self):
    result = assert_message_latency(
        self.node,
        "/scan",
        LaserScan,
        max_latency_sec=0.1,  # 100ms max
        duration_sec=5.0
    )

    assert result.success, result.details
```

## Custom Assertions

Create reusable custom assertions:

```python
from dataclasses import dataclass

@dataclass
class BatteryResult:
    success: bool
    voltage: float
    details: str

def assert_battery_ok(node, topic: str, min_voltage: float) -> BatteryResult:
    """Assert battery voltage is above minimum."""
    from sensor_msgs.msg import BatteryState

    collector = MessageCollector(node, topic, BatteryState)
    spin_for_duration(node, 2.0)

    messages = collector.get_messages()
    if not messages:
        return BatteryResult(False, 0.0, "No battery messages")

    voltage = messages[-1].voltage
    success = voltage >= min_voltage

    return BatteryResult(
        success=success,
        voltage=voltage,
        details=f"Voltage: {voltage:.2f}V (min: {min_voltage}V)"
    )


# Usage
def test_battery(self):
    result = assert_battery_ok(self.node, "/battery_state", min_voltage=11.0)
    assert result.success, result.details
```

## Assertion Results

Most assertions return result objects with:

| Field | Description |
|-------|-------------|
| `success` | Boolean - whether assertion passed |
| `details` | Human-readable description |
| `*` | Additional context-specific fields |

Example:

```python
result = assert_vehicle_moved(node, "robot", min_distance=1.0)

print(result.success)        # True/False
print(result.details)        # "Moved 1.5m (required: 1.0m)"
print(result.distance_moved) # 1.5
print(result.start_position) # (0.0, 0.0, 0.0)
print(result.end_position)   # (1.5, 0.0, 0.0)
```
