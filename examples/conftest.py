# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Pytest configuration for integration tests.

Sets up environment variables needed for simulation tests.
"""

import os
import pytest


def pytest_configure(config):
    """Configure pytest for integration tests."""
    # Set DISPLAY if not set (needed for Gazebo GUI)
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'

    # Set TurtleBot3 model if not set
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'


def pytest_collection_modifyitems(config, items):
    """
    Modify test collection to handle integration tests.

    If not explicitly running integration tests, skip them.
    """
    # Check if we're running integration tests
    markexpr = config.getoption('-m', default='')

    # If no marker expression specified and running all tests,
    # we don't skip integration tests (let them run)
    if not markexpr:
        return

    # If explicitly excluding integration tests, they'll be filtered by pytest
    # If explicitly including integration tests, they'll run


@pytest.fixture(scope="session", autouse=True)
def setup_integration_environment():
    """
    Session-scoped fixture to set up integration test environment.

    Ensures proper environment variables are set for simulation tests.
    """
    # Store original values
    original_env = {}
    env_vars = {
        'DISPLAY': os.environ.get('DISPLAY', ':0'),
        'TURTLEBOT3_MODEL': os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
    }

    for key, value in env_vars.items():
        original_env[key] = os.environ.get(key)
        os.environ[key] = value

    yield

    # Restore original values
    for key, original in original_env.items():
        if original is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = original
