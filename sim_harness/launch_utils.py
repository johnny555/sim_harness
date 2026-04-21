# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Reusable launch utilities for ROS 2 + Gazebo simulation launch files.

Provides:
- ``chain_on_exit(*actions)`` -- sequential process chaining
- ``get_gazebo_environment_actions(package_name)`` -- Gazebo env var setup
- ``WaitForCondition`` + factories -- predicate-based launch gating
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit

from sim_harness.launch_wait import (  # noqa: F401
    WaitForCondition,
    lifecycle_active,
    service_available,
    topic_publishing,
)


def chain_on_exit(*actions):
    """Chain launch actions to execute sequentially when each process exits.

    Each action in the chain starts when the previous one exits.

    Args:
        *actions: Variable number of launch actions to chain (minimum 2).

    Returns:
        tuple: (handlers, first_action)
            - handlers: List of RegisterEventHandler to add to LaunchDescription
            - first_action: The first action that should be launched immediately

    Example::

        handlers, first = chain_on_exit(
            spawn_entity,
            joint_state_broadcaster_spawner,
            controller_spawner,
        )

        return LaunchDescription([
            first,      # Launches immediately
            *handlers,  # Event handlers trigger subsequent actions
        ])
    """
    if len(actions) < 2:
        raise ValueError("chain_on_exit requires at least 2 actions")

    handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=current,
                on_exit=[next_action],
            )
        )
        for current, next_action in zip(actions, actions[1:])
    ]
    return handlers, actions[0]


def get_gazebo_environment_actions(package_name, models_subdir='models'):
    """Get common Gazebo Harmonic environment variable setup actions.

    Sets up:
    - ``GZ_SIM_RESOURCE_PATH`` -- mesh/model search paths
    - ``GZ_VERSION`` -- ``harmonic``
    - ``GZ_SIM_SYSTEM_PLUGIN_PATH`` -- plugin search path (e.g. gz_ros2_control)
    - Vulkan workarounds for OGRE2/NVIDIA

    Args:
        package_name: ROS 2 package whose share/lib dirs contain models and plugins.
        models_subdir: Subdirectory within the package share dir that contains
            Gazebo models (default ``'models'``).

    Returns:
        tuple of SetEnvironmentVariable actions:
            (gz_resource_path, gz_version, gz_plugin_path,
             vk_layer_enables, vk_icd_filenames, vk_loader_debug)
    """
    pkg_share = get_package_share_directory(package_name)
    models_path = os.path.join(pkg_share, models_subdir)

    # GZ_SIM_RESOURCE_PATH: prepend package models + share dirs
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_resource_path:
        resource_path = f'{models_path}:{pkg_share}:{existing_resource_path}'
    else:
        resource_path = f'{models_path}:{pkg_share}'

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    gz_version = SetEnvironmentVariable(
        name='GZ_VERSION',
        value='harmonic',
    )

    # GZ_SIM_SYSTEM_PLUGIN_PATH: append package install lib dir
    install_prefix = get_package_prefix(package_name)
    install_lib = os.path.join(install_prefix, 'lib')

    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if existing_plugin_path:
        plugin_path = f'{existing_plugin_path}:{install_lib}'
    else:
        plugin_path = install_lib

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path,
    )

    # Vulkan workarounds for OGRE2
    vk_layer_enables = SetEnvironmentVariable(
        name='VK_INSTANCE_LAYERS',
        value='',
    )
    vk_icd_filenames = SetEnvironmentVariable(
        name='VK_ICD_FILENAMES',
        value='/usr/share/vulkan/icd.d/nvidia_icd.json',
    )
    vk_loader_debug = SetEnvironmentVariable(
        name='VK_LOADER_DEBUG',
        value='none',
    )

    return (gz_resource_path, gz_version, gz_plugin_path,
            vk_layer_enables, vk_icd_filenames, vk_loader_debug)
