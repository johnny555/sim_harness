# Quick Start

This guide will get you running tests in under 5 minutes.

## Run the Example Tests

sim_harness comes with example tests for TurtleBot3. Let's run them.

### 1. Start the Simulation

In one terminal, start the TurtleBot3 simulation:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run the Tests

In another terminal, run the example tests:

```bash
cd ~/ros2_ws/src/sim_harness
pytest examples/test_turtlebot3_integration.py -v -k "Basic"
```

You should see output like:

```
examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_lidar_publishes PASSED
examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_odometry_publishes PASSED
examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_imu_publishes PASSED
```

## Run Tests with Auto-Simulation

sim_harness can automatically start and stop the simulation:

```bash
pytest examples/test_turtlebot3_with_sim_control.py -v -s
```

!!! note
    The `-s` flag shows simulation startup output. The first run may take longer as Gazebo initializes.

## Using the CLI

sim_harness provides CLI tools for test discovery and execution:

```bash
# List all tests in your workspace
ros2 run sim_harness list_tests

# Run a specific test
ros2 run sim_harness run_test test_turtlebot3_integration

# Run all tests matching a pattern
ros2 run sim_harness run_tests --pattern "test_basic*"
```

## What's Next?

- [Your First Test](first-test.md) - Write your own test
- [Writing Tests](../guide/writing-tests.md) - Learn the test patterns
- [Simulation Control](../guide/simulation-control.md) - Auto-start simulations
