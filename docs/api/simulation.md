# Simulation API

Classes for managing Gazebo simulation lifecycle.

## SimulationManager

Singleton manager for simulation lifecycle.

```python
from sim_harness.simulator import SimulationManager

manager = SimulationManager.get_instance()
```

### get_instance()

Get the singleton instance.

```python
manager = SimulationManager.get_instance()
```

**Returns:** `SimulationManager`

---

### request(request, ...)

Request a simulation with the given configuration.

```python
from sim_harness.simulator import SimulationRequest

request = SimulationRequest(
    package='turtlebot3_gazebo',
    launch_file='turtlebot3_world.launch.py',
    launch_args={'use_sim_time': 'true'},
)

success = manager.request(
    request,
    startup_timeout=60.0,
    gazebo_delay=5.0,
    require_sim=True
)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `request` | SimulationRequest | required | Simulation config |
| `startup_timeout` | float | `60.0` | Max startup wait |
| `gazebo_delay` | float | `5.0` | Post-Gazebo delay |
| `require_sim` | bool | `True` | Raise if can't start |

**Returns:** `bool` - True if simulation is ready

**Raises:** `RuntimeError` if require_sim=True and simulation can't start

---

### release()

Release the simulation (allow reuse).

```python
manager.release()
```

Does not stop the simulation, just decrements the user count.

---

### stop(force=False)

Stop the simulation.

```python
manager.stop()        # Only stops if no active users
manager.stop(force=True)  # Forces stop regardless
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `force` | bool | `False` | Stop even if users active |

---

### restart(request=None, ...)

Force restart the simulation.

```python
success = manager.restart(
    request=None,  # Uses current config
    startup_timeout=60.0,
    gazebo_delay=5.0
)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `request` | SimulationRequest | None | New config (or reuse current) |
| `startup_timeout` | float | `60.0` | Max startup wait |
| `gazebo_delay` | float | `5.0` | Post-Gazebo delay |

**Returns:** `bool`

---

### is_running()

Check if simulation is currently running.

```python
if manager.is_running():
    print("Simulation is active")
```

**Returns:** `bool`

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `current_config` | SimulationRequest | Current simulation config |
| `active_users` | int | Number of active users |

---

## SimulationRequest

Configuration for a simulation request.

```python
from sim_harness.simulator import SimulationRequest

request = SimulationRequest(
    package='turtlebot3_gazebo',
    launch_file='turtlebot3_world.launch.py',
    launch_args={'use_sim_time': 'true', 'world': 'empty'},
    world='empty',
    robot_model='waffle'
)
```

**Fields:**

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `package` | str | required | ROS 2 package name |
| `launch_file` | str | required | Launch file name |
| `launch_args` | dict | `{}` | Launch arguments |
| `world` | str | `""` | World name (for restart detection) |
| `robot_model` | str | `""` | Robot model (for restart detection) |

**Methods:**

#### config_hash()

Generate a hash for configuration comparison.

```python
hash1 = request1.config_hash()
hash2 = request2.config_hash()
if hash1 == hash2:
    print("Configs are compatible")
```

**Returns:** `str` - 12-character hash

#### to_launch_config(...)

Convert to internal LaunchConfig.

```python
config = request.to_launch_config(
    env_vars={'TURTLEBOT3_MODEL': 'waffle'},
    startup_timeout=60.0,
    gazebo_delay=5.0
)
```

**Returns:** `LaunchConfig`

---

## GazeboBackend

Low-level Gazebo process management.

```python
from sim_harness.simulator import GazeboBackend

gazebo = GazeboBackend()
```

### is_running()

Check if Gazebo is currently running.

```python
if gazebo.is_running():
    print("Gazebo is active")
```

**Returns:** `bool`

---

### wait_for_gazebo(timeout)

Wait for Gazebo to start.

```python
success = gazebo.wait_for_gazebo(timeout=30.0)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `timeout` | float | required | Max wait time in seconds |

**Returns:** `bool`

---

### kill_gazebo()

Kill all Gazebo processes.

```python
gazebo.kill_gazebo()
```

---

## Convenience Function

### get_simulation_manager()

Get the singleton SimulationManager instance.

```python
from sim_harness.simulator import get_simulation_manager

manager = get_simulation_manager()
```

**Returns:** `SimulationManager`
