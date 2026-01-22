# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0
"""Launch testing integration test for TurtleBot3 with Nav2 and Gazebo."""

import os
import random
import subprocess
import unittest

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers

import pytest


# Test configuration
GAZEBO_STARTUP_DELAY = 20.0   # Seconds to wait for Gazebo to start
NAV2_STARTUP_DELAY = 15.0     # Additional delay for Nav2 after Gazebo
TEST_STARTUP_DELAY = 50.0     # Total delay before running tests
INITIAL_POSE_DELAY = 35.0     # When to publish initial pose for AMCL


def kill_gazebo_processes():
    """Kill any remaining Gazebo processes."""
    try:
        # Kill gz sim processes
        subprocess.run(
            ['pkill', '-9', '-f', 'gz sim'],
            capture_output=True,
            timeout=5
        )
        subprocess.run(
            ['pkill', '-9', '-f', 'ruby.*gz'],
            capture_output=True,
            timeout=5
        )
    except Exception:
        pass


def get_unique_domain_id():
    """Generate a unique ROS_DOMAIN_ID for test isolation."""
    return str(random.randint(1, 232))


def check_simulation_available():
    """Check if required simulation packages are available."""
    try:
        get_package_share_directory('turtlebot3_gazebo')
        get_package_share_directory('turtlebot3_navigation2')
        return True, 'turtlebot3'
    except PackageNotFoundError:
        pass

    try:
        # Check for Gazebo and Nav2 minimal sim
        get_package_share_directory('ros_gz_sim')
        get_package_share_directory('nav2_bringup')
        return True, 'minimal'
    except PackageNotFoundError:
        pass

    return False, None


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate the test launch description."""
    # Check if simulation is available
    sim_available, sim_type = check_simulation_available()

    # Generate unique domain ID for test isolation
    domain_id = get_unique_domain_id()
    gz_partition = f'gz_test_{domain_id}'

    # Get package directories
    try:
        sim_test_utils_dir = get_package_share_directory('sim_harness')
    except PackageNotFoundError:
        # Package not installed yet - skip
        return LaunchDescription([
            LogInfo(msg='sim_test_utils package not found, skipping test'),
            launch_testing.actions.ReadyToTest(),
        ]), {}

    # Environment setup for isolation
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id)
    set_gz_partition = SetEnvironmentVariable('GZ_PARTITION', gz_partition)
    set_turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # Test binary path - in lib/<package>/, not share/<package>/lib/
    # The share dir is sim_test_utils_dir, we need the lib dir
    install_prefix = os.path.dirname(os.path.dirname(sim_test_utils_dir))
    test_binary = os.path.join(
        install_prefix,
        'lib',
        'sim_harness',
        'turtlebot3_integration_test'
    )

    # Check if test binary exists
    if not os.path.exists(test_binary):
        return LaunchDescription([
            LogInfo(msg=f'Test binary not found: {test_binary}'),
            LogInfo(msg='Build the package first with: colcon build'),
            launch_testing.actions.ReadyToTest(),
        ]), {}

    if not sim_available:
        # No simulation available - run a minimal test that just checks
        # the test binary can start (it will fail fast but that's OK)
        test_process = ExecuteProcess(
            cmd=[
                test_binary,
                '--gtest_filter=*Dummy*',  # No tests match, exits quickly
                '--gtest_output=xml:/tmp/turtlebot3_gtest_results.xml',
            ],
            name='turtlebot3_integration_test',
            output='screen',
        )

        return LaunchDescription([
            set_domain_id,
            LogInfo(msg='WARNING: Simulation packages not found.'),
            LogInfo(msg='Install: sudo apt install ros-jazzy-turtlebot3-gazebo '
                        'ros-jazzy-turtlebot3-navigation2'),
            LogInfo(msg='Running minimal test (binary check only)...'),
            test_process,
            launch_testing.actions.ReadyToTest(),
        ]), {'test_process': test_process}

    def launch_setup(context, *args, **kwargs):
        """Set up simulation launch."""
        actions = []

        if sim_type == 'turtlebot3':
            try:
                tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
                tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
                nav2_bringup_dir = get_package_share_directory('nav2_bringup')

                # Nav2 params file
                nav2_params_file = os.path.join(
                    tb3_nav2_dir, 'param', 'waffle.yaml'
                )
                map_file = os.path.join(tb3_nav2_dir, 'map', 'map.yaml')

                # Launch TurtleBot3 Gazebo
                tb3_gazebo_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'
                        )
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                )
                actions.append(tb3_gazebo_launch)

                # Launch Nav2 (delayed to let Gazebo start first)
                nav2_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map': map_file,
                        'params_file': nav2_params_file,
                        'autostart': 'true',
                    }.items(),
                )
                actions.append(TimerAction(
                    period=NAV2_STARTUP_DELAY,
                    actions=[nav2_launch]
                ))

                # Publish initial pose for AMCL (TurtleBot3 starts at -2.0, -0.5)
                # This is needed for localization to work
                initial_pose_cmd = ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/initialpose',
                        'geometry_msgs/msg/PoseWithCovarianceStamped',
                        '{header: {frame_id: "map"}, '
                        'pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, '
                        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
                        'covariance: [0.25, 0, 0, 0, 0, 0, '
                        '0, 0.25, 0, 0, 0, 0, '
                        '0, 0, 0, 0, 0, 0, '
                        '0, 0, 0, 0, 0, 0, '
                        '0, 0, 0, 0, 0, 0, '
                        '0, 0, 0, 0, 0, 0.07]}}'
                    ],
                    name='initial_pose_publisher',
                    output='screen',
                )
                actions.append(TimerAction(
                    period=INITIAL_POSE_DELAY,
                    actions=[
                        LogInfo(msg='Publishing initial pose for AMCL...'),
                        initial_pose_cmd
                    ]
                ))

                actions.append(LogInfo(msg='Launched turtlebot3_gazebo + Nav2'))
            except Exception as e:
                actions.append(LogInfo(msg=f'Error launching simulation: {e}'))

        return actions

    # The test process (delayed to allow system startup)
    test_process = ExecuteProcess(
        cmd=[
            test_binary,
            '--gtest_output=xml:/tmp/turtlebot3_gtest_results.xml',
        ],
        name='turtlebot3_integration_test',
        output='screen',
    )

    # When test process exits, shut down everything
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=test_process,
            on_exit=[
                LogInfo(msg='Test process exited, shutting down...'),
                Shutdown(reason='Test completed'),
            ],
        )
    )

    # Delay test start to allow Gazebo + Nav2 + AMCL to fully initialize
    delayed_test = TimerAction(
        period=TEST_STARTUP_DELAY,
        actions=[
            LogInfo(msg='Starting integration tests...'),
            test_process
        ],
    )

    return LaunchDescription([
        # Environment setup
        set_domain_id,
        set_gz_partition,
        set_turtlebot_model,

        # Launch setup (Gazebo, Nav2)
        OpaqueFunction(function=launch_setup),

        # Test process (delayed)
        delayed_test,

        # Shutdown handler
        shutdown_on_exit,

        # Mark ready for launch_testing
        launch_testing.actions.ReadyToTest(),
    ]), {'test_process': test_process}


class TestTurtleBot3Integration(unittest.TestCase):
    """Test class that runs while launch is active."""

    def test_wait_for_test_process(self, test_process, proc_output, proc_info):
        """Wait for the C++ test binary to complete and verify it passes."""
        # Wait for the test process to finish (up to 120 seconds)
        proc_info.assertWaitForShutdown(process=test_process, timeout=120)


@launch_testing.post_shutdown_test()
class TestTurtleBot3Shutdown(unittest.TestCase):
    """Post-shutdown tests."""

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests - ensure Gazebo is killed."""
        kill_gazebo_processes()

    def test_exit_code(self, test_process, proc_info):
        """Verify the integration test exited successfully."""
        # Check that the C++ GTest binary returned 0 (all tests passed)
        launch_testing.asserts.assertExitCodes(proc_info, [0], process=test_process)
        # Also kill any remaining Gazebo processes
        kill_gazebo_processes()
