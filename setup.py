# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Setup file for sim_harness package.

This file is used by ament_python_install_package to install the Python
package with its entry points for ros2cli.
"""

from setuptools import setup, find_packages

setup(
    name='sim_harness',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    extras_require={
        'hypothesis': ['hypothesis>=6.0'],
    },
    zip_safe=True,
    author='John',
    author_email='john@example.com',
    maintainer='John',
    maintainer_email='john@example.com',
    description='Unified test harness for ROS 2 simulation testing',
    license='Apache-2.0',
    entry_points={
        # Register 'test' as a new ros2 command
        'ros2cli.command': [
            'test = sim_harness.ros2test.command.test:TestCommand',
        ],
        # Register the extension point for ros2 test verbs
        'ros2cli.extension_point': [
            'ros2test.verb = sim_harness.ros2test.verb:VerbExtension',
        ],
        # Register the verb implementations
        'ros2test.verb': [
            'list = sim_harness.ros2test.verb.list:ListVerb',
            'run = sim_harness.ros2test.verb.run:RunVerb',
            'failed = sim_harness.ros2test.verb.failed:FailedVerb',
        ],
    },
)
