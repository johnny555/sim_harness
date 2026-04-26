# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""Setup file for sim_harness.

Most metadata lives in ``setup.cfg``; this file exists so
``ament_python_install_package`` (called from ``CMakeLists.txt``) has
something to import.
"""

from setuptools import setup, find_packages

setup(
    name='sim_harness',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    # openpyxl is the runtime backend for the --jama-xlsx pytest plugin.
    # In a ROS 2 workspace, prefer `rosdep install` (resolves to
    # python3-openpyxl via apt); listing it here covers the non-ROS pip-install
    # path and matches the package.xml exec_depend.
    install_requires=['setuptools', 'openpyxl>=3.0'],
    extras_require={
        'hypothesis': ['hypothesis>=6.0'],
    },
    zip_safe=True,
    author='John',
    author_email='john@example.com',
    maintainer='John',
    maintainer_email='john@example.com',
    description='Pytest plugin and helpers for ROS 2 simulation testing',
    license='Apache-2.0',
    entry_points={
        # Pytest plugins (auto-loaded whenever sim_harness is on the import path).
        'pytest11': [
            'sim_harness_jama = sim_harness.pytest_jama_plugin',
            'sim_harness_ros  = sim_harness.pytest_ros_fixtures',
        ],
    },
)
