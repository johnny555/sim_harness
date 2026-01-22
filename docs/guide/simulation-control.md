# Simulation Control

sim_harness can automatically manage the Gazebo simulation lifecycle for your tests.

## Automatic Simulation Management

Use `SimulationTestFixture` to auto-start simulations:

```python
from sim_harness import SimulationTestFixture

class TestWithSim(SimulationTestFixture):
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_robot_spawned(self):
        # Simulation is already running when this executes
        collector = self.create_message_collector("/scan", LaserScan)
        self.spin_for_duration(3.0)
        assert collector.message_count() > 0
```

## Configuration Options

### Launch Configuration

```python
class TestMyRobot(SimulationTestFixture):
    # Required: Package containing the launch file
    LAUNCH_PACKAGE = 'my_robot_gazebo'

    # Required: Launch file name
    LAUNCH_FILE = 'simulation.launch.py'

    # Optional: Arguments passed to the launch file
    LAUNCH_ARGS = {
        'use_sim_time': 'true',
        'world': 'my_world.sdf',
        'robot_name': 'my_robot',
    }

    # Optional: Environment variables
    ENV_VARS = {
        'TURTLEBOT3_MODEL': 'waffle',
        'IGN_GAZEBO_RESOURCE_PATH': '/path/to/models',
    }
```

### Timing Configuration

```python
class TestWithCustomTiming(SimulationTestFixture):
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'

    # Max time to wait for simulation to start (default: 60.0)
    STARTUP_TIMEOUT = 120.0

    # Extra delay after Gazebo is detected (default: 5.0)
    # Allows time for robot to spawn and sensors to initialize
    GAZEBO_STARTUP_DELAY = 10.0
```

### Behavior Options

```python
class TestBehavior(SimulationTestFixture):
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'

    # If True (default): test fails if simulation can't start
    # If False: test is skipped if simulation can't start
    REQUIRE_SIM = True

    # If True: don't start simulation, assume it's already running
    # Useful for development iteration
    USE_EXISTING_SIM = False
```

## Simulation Reuse

The simulation manager intelligently reuses simulations:

```python
class TestFirst(SimulationTestFixture):
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_a(self):
        pass

    def test_b(self):
        pass  # Same sim instance


class TestSecond(SimulationTestFixture):
    # Same config = same simulation is reused
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_c(self):
        pass  # Still same sim instance


class TestDifferent(SimulationTestFixture):
    # Different config = simulation is restarted
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_house.launch.py'  # Different world

    def test_d(self):
        pass  # New sim instance
```

### What Triggers a Restart?

The simulation restarts when these change:

- `LAUNCH_PACKAGE`
- `LAUNCH_FILE`
- `world` or `world_name` in `LAUNCH_ARGS`
- `robot_model` or `model` in `LAUNCH_ARGS`
- `vehicle` in `LAUNCH_ARGS`
- `headless` or `gui` in `LAUNCH_ARGS`

## Manual Simulation Control

For advanced use cases, access the simulation manager directly:

```python
from sim_harness.simulator import SimulationManager, SimulationRequest

def test_manual_control():
    manager = SimulationManager.get_instance()

    # Request a simulation
    request = SimulationRequest(
        package='turtlebot3_gazebo',
        launch_file='turtlebot3_world.launch.py',
        launch_args={'use_sim_time': 'true'},
        world='turtlebot3_world',
    )

    success = manager.request(request, startup_timeout=60.0)
    assert success, "Failed to start simulation"

    try:
        # Run tests...
        pass
    finally:
        manager.release()  # Allow reuse by other tests
        # manager.stop()   # Or force stop
```

## Using Pre-Started Simulation

For faster development iteration, start the simulation once manually:

```bash
# Terminal 1: Start simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```python
# Your test file
class TestQuickIteration(SimulationTestFixture):
    USE_EXISTING_SIM = True  # Don't manage simulation lifecycle
    REQUIRE_SIM = True       # Skip if sim not running

    def test_something(self):
        # Uses the already-running simulation
        pass
```

Run tests repeatedly without restarting simulation:

```bash
# Terminal 2: Run tests (fast!)
pytest test_my_robot.py -v
pytest test_my_robot.py -v  # Still fast - reuses sim
```

## Headless Mode

Run simulations without GUI for CI:

```python
class TestHeadless(SimulationTestFixture):
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'
    LAUNCH_ARGS = {
        'headless': 'true',
        'gui': 'false',
    }
```

Or via environment variable:

```bash
GAZEBO_HEADLESS=1 pytest tests/ -v
```

## Test Isolation

Each test run uses isolated ROS 2 communication:

```python
class TestIsolated(SimulationTestFixture):
    # ... config ...

    def test_a(self):
        # This test uses ROS_DOMAIN_ID=X
        pass

    def test_b(self):
        # This test also uses ROS_DOMAIN_ID=X (same test session)
        pass
```

Different test sessions get different domain IDs, preventing interference.

## Troubleshooting

### Simulation Won't Start

1. Check the launch file works manually:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Increase timeout:
   ```python
   STARTUP_TIMEOUT = 120.0
   ```

3. Check Gazebo resource paths:
   ```python
   ENV_VARS = {'IGN_GAZEBO_RESOURCE_PATH': '/path/to/models'}
   ```

### Tests Are Flaky

1. Increase startup delay:
   ```python
   GAZEBO_STARTUP_DELAY = 15.0
   ```

2. Add explicit waits in tests:
   ```python
   self.spin_for_duration(5.0)  # Wait for sensors to initialize
   ```

### Simulation Not Cleaning Up

Force cleanup at end of tests:

```python
import pytest
from sim_harness.simulator import SimulationManager

@pytest.fixture(scope="session", autouse=True)
def cleanup_sim():
    yield
    SimulationManager.get_instance().stop(force=True)
```
