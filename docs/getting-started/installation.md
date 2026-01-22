# Installation

## Prerequisites

- **ROS 2** Humble or later
- **Python** 3.10+
- **Gazebo** Harmonic (for simulation tests)

## Install from PyPI

```bash
pip install sim-harness
```

## Install from Source

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/johnny555/sim_harness.git
```

Build with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select sim_harness
source install/setup.bash
```

## Verify Installation

Check that sim_harness is installed:

```bash
python3 -c "import sim_harness; print(sim_harness.__version__)"
```

List available test commands:

```bash
ros2 run sim_harness list_tests --help
```

## Optional Dependencies

For running the TurtleBot3 examples:

```bash
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo
```

For development (docs, linting):

```bash
pip install mkdocs-material mkdocstrings[python] pytest-cov
```

## Next Steps

- [Quick Start](quickstart.md) - Run your first test
- [Your First Test](first-test.md) - Write a custom test
