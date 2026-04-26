# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""sim_harness — Python test utilities for ROS 2 simulation testing.

Quick start::

    from sim_harness import SimTestFixture
    from sim_harness.checks import check_lidar_valid

    class TestMyRobot(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_sim'
        LAUNCH_FILE = 'sim.launch.py'

        def test_lidar(self):
            result = check_lidar_valid(self.node, '/scan')
            assert result.ok, result.details

Check functions::

    from sim_harness.checks import check_sensor_publishing, check_gps_valid
    from sim_harness.nav2 import check_nav2_active, check_reaches_goal
    from sim_harness.perception import check_object_detected

Pytest plugin (auto-loaded via ``pytest11`` entry point)::

    @pytest.mark.requirement("REQ-001", "Sensor publishes data", category="Sensors")
    def test_sensor():
        ...

    pytest --jama-xlsx out.xlsx        # export Jama-importable spreadsheet
"""

# -- Layer 0: Core (what every test needs) ─────────────────────────────────
from sim_harness.fixture import (  # noqa: F401
    SimTestFixture,
    ros_node,
    ros_executor,
)
from sim_harness.collector import MessageCollector  # noqa: F401
from sim_harness.spin import (  # noqa: F401
    ExecutorContext,
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)

# -- Simulator (lazy — only imported when used) ────────────────────────────
from sim_harness.simulator.simulation_manager import (  # noqa: F401
    SimulationManager,
    SimulationRequest,
    get_simulation_manager,
)
from sim_harness.simulator.simulator_interface import (  # noqa: F401
    SimulatorType,
    SimulatorConfig,
    SimulatorInterface,
)
from sim_harness.simulator.gazebo_backend import (  # noqa: F401
    GazeboBackend,
    NullBackend,
)
from sim_harness.simulator.simulation_launcher import (  # noqa: F401
    SimulationLauncher,
    LaunchConfig,
    kill_all_gazebo,
)

# -- Launch utilities ──────────────────────────────────────────────────────
from sim_harness.launch_utils import (  # noqa: F401
    chain_on_exit,
    get_gazebo_environment_actions,
    WaitForCondition,
    lifecycle_active,
    service_available,
    topic_publishing,
)

# -- Backward compatibility re-exports for sim_harness.checks ─────────────
from sim_harness.fixture import SimulationTestFixture  # noqa: F401

from sim_harness.checks import (  # noqa: F401
    ServiceResult, SensorDataResult, TimingResult, MovementResult, VelocityResult,
    check_service_available,
    check_action_server_available,
    check_node_running,
    check_nodes_running,
    check_parameter_exists,
    check_sensor_publishing,
    check_lidar_valid,
    check_gps_valid,
    check_imu_valid,
    check_camera_valid,
    check_joint_states_valid,
    check_publish_rate,
    check_latency,
    check_transform_available,
    check_action_server_responsive,
    check_vehicle_moved,
    check_vehicle_moved_with_ground_truth,
    check_vehicle_stationary,
    check_vehicle_velocity,
    check_vehicle_in_region,
    check_vehicle_orientation,
)

__all__ = [
    # Layer 0: Core fixture
    'SimTestFixture',
    'MessageCollector',
    'ExecutorContext',
    'spin_for_duration',
    'spin_until_condition',
    'spin_until_messages_received',
    'ros_node',
    'ros_executor',
    # Simulator
    'SimulationManager',
    'SimulationRequest',
    'get_simulation_manager',
    'SimulatorType',
    'SimulatorConfig',
    'SimulatorInterface',
    'GazeboBackend',
    'NullBackend',
    'SimulationLauncher',
    'LaunchConfig',
    'kill_all_gazebo',
    # Launch utilities
    'chain_on_exit',
    'get_gazebo_environment_actions',
    'WaitForCondition',
    'lifecycle_active',
    'service_available',
    'topic_publishing',
]
