# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""
Abstract interface for simulator backends.

Allows tests to be written independent of the specific simulator.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional


class SimulatorType(Enum):
    """Supported simulator types."""
    GAZEBO = auto()      # Gazebo Harmonic
    MUJOCO = auto()      # MuJoCo (future)
    OMNIVERSE = auto()   # NVIDIA Omniverse (future)
    NONE = auto()        # No simulator (for unit tests)


def simulator_type_to_string(sim_type: SimulatorType) -> str:
    """Convert SimulatorType to string."""
    return {
        SimulatorType.GAZEBO: "Gazebo",
        SimulatorType.MUJOCO: "MuJoCo",
        SimulatorType.OMNIVERSE: "Omniverse",
        SimulatorType.NONE: "None",
    }.get(sim_type, "Unknown")


@dataclass
class SimulatorConfig:
    """Configuration for launching a simulator."""

    world_file: str = ""
    """Path to world file (SDF, MJCF, USD, etc.)."""

    launch_args: Dict[str, str] = field(default_factory=dict)
    """Launch arguments (key-value pairs)."""

    startup_timeout_sec: float = 30.0
    """Maximum time to wait for simulator startup."""

    headless: bool = True
    """Whether to run headless (no GUI)."""


class SimulatorInterface(ABC):
    """
    Abstract interface for simulator backends.

    Allows tests to be written independent of the specific simulator.
    Implementations handle simulator-specific details.

    Usage:
        sim = SimulatorInterface.create(SimulatorType.GAZEBO)
        sim.wait_until_ready(timeout_sec=30)
        # Run tests...
    """

    @abstractmethod
    def type(self) -> SimulatorType:
        """Get the simulator type."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Check if the simulator is running."""
        pass

    @abstractmethod
    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait until the simulator is ready for testing.

        Args:
            timeout_sec: Maximum time to wait

        Returns:
            True if simulator is ready, False on timeout
        """
        pass

    @abstractmethod
    def get_partition(self) -> str:
        """
        Get simulator-specific partition/namespace.

        For Gazebo this is GZ_PARTITION, for others it may be different.
        """
        pass

    @staticmethod
    def create(sim_type: SimulatorType) -> 'SimulatorInterface':
        """
        Factory method to create simulator backend.

        Args:
            sim_type: Type of simulator

        Returns:
            Simulator interface instance
        """
        from sim_harness.simulator.gazebo_backend import GazeboBackend, NullBackend

        if sim_type == SimulatorType.GAZEBO:
            return GazeboBackend()
        elif sim_type == SimulatorType.NONE:
            return NullBackend()
        else:
            # Not yet implemented - return null backend
            return NullBackend()
