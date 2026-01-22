# Primitives API

Pre-built assertion primitives for common robotics testing scenarios.

## Sensor Assertions

### assert_topic_published

Assert that a topic is publishing messages.

```python
from sim_harness import assert_topic_published

result = assert_topic_published(
    node,
    topic="/scan",
    msg_type=LaserScan,
    timeout_sec=5.0,
    min_messages=1
)

assert result.success, result.details
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `topic` | str | required | Topic name |
| `msg_type` | type | required | Message type |
| `timeout_sec` | float | `5.0` | Timeout in seconds |
| `min_messages` | int | `1` | Minimum messages required |

**Returns:** `TopicResult` with fields:
- `success: bool`
- `message_count: int`
- `details: str`

---

## Vehicle Assertions

### assert_vehicle_moved

Assert that a vehicle moved a minimum distance.

```python
from sim_harness import assert_vehicle_moved

result = assert_vehicle_moved(
    node,
    vehicle_id="robot_01",
    min_distance=1.0,
    velocity=0.5,
    timeout_sec=10.0,
    odom_topic=None,     # Uses /{vehicle_id}/odom
    cmd_vel_topic=None,  # Uses /{vehicle_id}/cmd_vel
    use_twist_stamped=True
)

assert result.success, result.details
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `vehicle_id` | str | required | Vehicle namespace |
| `min_distance` | float | required | Minimum distance (meters) |
| `velocity` | float | `1.0` | Command velocity (m/s) |
| `timeout_sec` | float | `10.0` | Timeout |
| `odom_topic` | str | None | Custom odometry topic |
| `cmd_vel_topic` | str | None | Custom cmd_vel topic |
| `use_twist_stamped` | bool | `True` | Use TwistStamped vs Twist |

**Returns:** `MovementResult` with fields:
- `success: bool`
- `distance_moved: float`
- `start_position: Tuple[float, float, float]`
- `end_position: Tuple[float, float, float]`
- `details: str`

---

### assert_vehicle_stationary

Assert that a vehicle is stationary.

```python
from sim_harness import assert_vehicle_stationary

result = assert_vehicle_stationary(
    node,
    vehicle_id="robot_01",
    velocity_threshold=0.01,
    duration_sec=2.0,
    odom_topic=None
)

assert result, "Vehicle is moving"
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `vehicle_id` | str | required | Vehicle namespace |
| `velocity_threshold` | float | `0.01` | Max velocity (m/s) |
| `duration_sec` | float | `2.0` | Must stay still this long |
| `odom_topic` | str | None | Custom odometry topic |

**Returns:** `bool`

---

### assert_vehicle_velocity

Assert that a vehicle reaches a target velocity.

```python
from sim_harness import assert_vehicle_velocity

result = assert_vehicle_velocity(
    node,
    vehicle_id="robot_01",
    target_velocity=0.5,
    tolerance=0.1,
    timeout_sec=5.0,
    odom_topic=None
)

assert result.success, result.details
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `vehicle_id` | str | required | Vehicle namespace |
| `target_velocity` | float | required | Target velocity (m/s) |
| `tolerance` | float | `0.1` | Acceptable deviation |
| `timeout_sec` | float | `5.0` | Timeout |
| `odom_topic` | str | None | Custom odometry topic |

**Returns:** `VelocityResult` with fields:
- `success: bool`
- `measured_velocity: float`
- `details: str`

---

### assert_vehicle_in_region

Assert that a vehicle is within a bounding region.

```python
from sim_harness import assert_vehicle_in_region

result = assert_vehicle_in_region(
    node,
    vehicle_id="robot_01",
    min_bounds=(-5.0, -5.0, 0.0),
    max_bounds=(5.0, 5.0, 1.0),
    timeout_sec=5.0,
    odom_topic=None
)

assert result, "Vehicle outside bounds"
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `vehicle_id` | str | required | Vehicle namespace |
| `min_bounds` | Tuple[float, float, float] | required | Min (x, y, z) |
| `max_bounds` | Tuple[float, float, float] | required | Max (x, y, z) |
| `timeout_sec` | float | `5.0` | Timeout |
| `odom_topic` | str | None | Custom odometry topic |

**Returns:** `bool`

---

### assert_vehicle_orientation

Assert that a vehicle's yaw is within tolerance.

```python
from sim_harness import assert_vehicle_orientation
import math

result = assert_vehicle_orientation(
    node,
    vehicle_id="robot_01",
    expected_yaw=math.pi / 2,  # 90 degrees
    tolerance_rad=0.1,
    timeout_sec=5.0,
    odom_topic=None
)

assert result, "Vehicle orientation incorrect"
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `vehicle_id` | str | required | Vehicle namespace |
| `expected_yaw` | float | required | Expected yaw (radians) |
| `tolerance_rad` | float | `0.1` | Acceptable deviation |
| `timeout_sec` | float | `5.0` | Timeout |
| `odom_topic` | str | None | Custom odometry topic |

**Returns:** `bool`

---

## Lifecycle Assertions

### assert_lifecycle_node_active

Assert that a lifecycle node is in the active state.

```python
from sim_harness import assert_lifecycle_node_active

result = assert_lifecycle_node_active(
    node,
    lifecycle_node_name="controller_server",
    timeout_sec=10.0
)

assert result.success, result.details
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `lifecycle_node_name` | str | required | Name of lifecycle node |
| `timeout_sec` | float | `10.0` | Timeout |

**Returns:** `LifecycleResult` with fields:
- `success: bool`
- `state: str`
- `details: str`

---

## Service Assertions

### assert_service_available

Assert that a service is available.

```python
from sim_harness import assert_service_available

result = assert_service_available(
    node,
    service_name="/get_map",
    timeout_sec=5.0
)

assert result.success, result.details
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `node` | Node | required | ROS 2 node |
| `service_name` | str | required | Service name |
| `timeout_sec` | float | `5.0` | Timeout |

**Returns:** `ServiceResult` with fields:
- `success: bool`
- `details: str`
