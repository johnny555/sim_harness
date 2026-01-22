# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""Launch file for TurtleBot3 integration test with Nav2 and Gazebo.

This launch file:
1. Sets up test isolation (ROS_DOMAIN_ID, GZ_PARTITION)
2. Launches Gazebo with TurtleBot3 world
3. Spawns TurtleBot3 robot
4. Launches Nav2 navigation stack
5. Runs the integration test binary
6. Cleans up on completion

Prerequisites:
    sudo apt install ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2
    export TURTLEBOT3_MODEL=waffle  # or burger

Usage:
    ros2 launch sim_test_utils turtlebot3_test.launch.py

    # With custom model:
    ros2 launch sim_test_utils turtlebot3_test.launch.py model:=burger

    # Headless mode:
    ros2 launch sim_test_utils turtlebot3_test.launch.py headless:=true
"""

import os
import random

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch_testing
from launch_testing.actions import ReadyToTest


def get_unique_domain_id():
    """Generate a unique ROS_DOMAIN_ID for test isolation."""
    return str(random.randint(1, 232))


def generate_launch_description():
    # Get package directories
    sim_test_utils_dir = get_package_share_directory('sim_harness')

    # Declare launch arguments
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='waffle',
        description='TurtleBot3 model (burger, waffle, waffle_pi)'
    )

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='Gazebo world name (without .world extension)'
    )

    declare_test_timeout_arg = DeclareLaunchArgument(
        'test_timeout',
        default_value='300',
        description='Test timeout in seconds'
    )

    declare_nav2_params_arg = DeclareLaunchArgument(
        'nav2_params',
        default_value='',
        description='Path to custom Nav2 params file (optional)'
    )

    # Generate unique domain ID for test isolation
    domain_id = get_unique_domain_id()
    gz_partition = f"gz_test_{domain_id}"

    # Environment setup for isolation
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id)
    set_gz_partition = SetEnvironmentVariable('GZ_PARTITION', gz_partition)
    set_turtlebot_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        LaunchConfiguration('model')
    )

    # Log test configuration
    log_config = LogInfo(
        msg=[
            'Starting TurtleBot3 Integration Test\n',
            '  ROS_DOMAIN_ID: ', domain_id, '\n',
            '  GZ_PARTITION: ', gz_partition, '\n',
            '  Model: ', LaunchConfiguration('model'), '\n',
            '  World: ', LaunchConfiguration('world'), '\n',
        ]
    )

    def launch_setup(context, *args, **kwargs):
        model = LaunchConfiguration('model').perform(context)
        world = LaunchConfiguration('world').perform(context)
        headless = LaunchConfiguration('headless').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        nav2_params = LaunchConfiguration('nav2_params').perform(context)

        # Get TurtleBot3 package directories
        try:
            tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
            tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
            nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        except Exception as e:
            return [
                LogInfo(msg=f'ERROR: Required packages not found: {e}'),
                LogInfo(msg='Install with: sudo apt install ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2'),
            ]

        # World file path
        world_file = os.path.join(
            tb3_gazebo_dir,
            'worlds',
            f'{world}.world'
        )

        # Nav2 params file
        if nav2_params:
            nav2_params_file = nav2_params
        else:
            nav2_params_file = os.path.join(
                tb3_nav2_dir,
                'param',
                f'{model}.yaml'
            )

        # Map file for navigation
        map_file = os.path.join(
            tb3_nav2_dir,
            'map',
            'map.yaml'
        )

        # URDF/robot description
        urdf_file = os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            f'turtlebot3_{model}.urdf'
        )

        # Gazebo launch arguments
        gz_args = f'-r {world_file}'
        if headless.lower() == 'true':
            gz_args += ' -s'  # Server only (headless)

        # 1. Launch Gazebo
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={
                'gz_args': gz_args,
            }.items(),
        )

        # Alternative: Use TurtleBot3's own Gazebo launch
        tb3_gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(tb3_gazebo_dir, 'launch', f'{world}.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        )

        # 2. Launch Nav2 (with map for localization)
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': nav2_params_file,
                'autostart': 'true',
            }.items(),
        )

        # 3. Test binary (delayed start to allow system to initialize)
        test_binary = os.path.join(
            sim_test_utils_dir,
            'lib',
            'sim_harness',
            'turtlebot3_integration_test'
        )

        test_process = ExecuteProcess(
            cmd=[
                test_binary,
                '--gtest_output=xml:/tmp/turtlebot3_test_results.xml',
            ],
            name='integration_test',
            output='screen',
            env={
                'ROS_DOMAIN_ID': domain_id,
            },
        )

        # Delay test start to allow Gazebo and Nav2 to initialize
        delayed_test = TimerAction(
            period=30.0,  # Wait 30 seconds for system startup
            actions=[test_process],
        )

        return [
            tb3_gazebo_launch,
            # Delay Nav2 to let Gazebo start first
            TimerAction(
                period=10.0,
                actions=[nav2_launch],
            ),
            delayed_test,
        ]

    # Use OpaqueFunction to evaluate launch configurations
    launch_setup_action = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        # Launch arguments
        declare_model_arg,
        declare_headless_arg,
        declare_use_sim_time_arg,
        declare_world_arg,
        declare_test_timeout_arg,
        declare_nav2_params_arg,

        # Environment setup
        set_domain_id,
        set_gz_partition,
        set_turtlebot_model,

        # Logging
        log_config,

        # Launch setup (includes Gazebo, Nav2, and test)
        launch_setup_action,

        # Ready for launch_testing
        ReadyToTest(),
    ])
