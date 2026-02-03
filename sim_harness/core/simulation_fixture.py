# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Backwards compatibility shim.

``SimulationTestFixture`` has been merged into :class:`SimTestFixture`.
Set ``LAUNCH_PACKAGE`` / ``LAUNCH_FILE`` on your test class to enable
simulation management.

This module re-exports the aliases so existing code keeps working.
"""

from sim_harness.core.test_fixture import (  # noqa: F401
    SimTestFixture as SimulationTestFixture,
)
from sim_harness.simulator.simulation_manager import (  # noqa: F401
    get_simulation_manager as _get_mgr,
)

import pytest


# Keep these module-level fixtures for backwards compat
@pytest.fixture(scope="module")
def simulation_manager():
    """Module-scoped fixture providing the SimulationManager."""
    manager = _get_mgr()
    yield manager


@pytest.fixture(scope="session")
def session_simulation():
    """Session-scoped fixture for the SimulationManager."""
    manager = _get_mgr()
    yield manager
    manager.stop(force=True)
