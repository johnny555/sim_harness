# Copyright 2026 The sim_harness Authors
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
        sim.start(config)
        sim.wait_until_ready(timeout_sec=30)
        # Run tests...
        sim.stop()
    """

    def __init__(self, config: Optional[SimulatorConfig] = None):
        """
        Initialize the simulator interface.

        Args:
            config: Optional simulator configuration
        """
        self._config = config or SimulatorConfig()

    @abstractmethod
    def type(self) -> SimulatorType:
        """Get the simulator type."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Check if the simulator processes are running."""
        pass

    @abstractmethod
    def is_responsive(self, timeout_sec: float = 2.0) -> bool:
        """
        Check if the simulator is responsive (not just running).

        This is a stronger check than is_running() - verifies the simulator
        is actually functional, not just that processes exist.

        Args:
            timeout_sec: Maximum time to wait for response check

        Returns:
            True if simulator is responsive, False otherwise
        """
        pass

    @abstractmethod
    def wait_until_ready(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait until the simulator is ready for testing.

        Args:
            timeout_sec: Maximum time to wait. If None, uses config value.

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

    @abstractmethod
    def start(self, config: Optional[SimulatorConfig] = None) -> bool:
        """
        Start the simulator.

        Args:
            config: Optional configuration to override current config

        Returns:
            True if simulator started successfully, False otherwise
        """
        pass

    @abstractmethod
    def stop(self) -> bool:
        """
        Stop the simulator gracefully.

        Returns:
            True if simulator stopped successfully, False otherwise
        """
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """
        Forcefully shutdown the simulator and clean up resources.

        This should be called to ensure all resources are released.
        """
        pass

    @staticmethod
    def create(sim_type: SimulatorType, config: Optional[SimulatorConfig] = None) -> 'SimulatorInterface':
        """
        Factory method to create simulator backend.

        Args:
            sim_type: Type of simulator
            config: Optional simulator configuration

        Returns:
            Simulator interface instance
        """
        from sim_harness.simulator.gazebo_backend import GazeboBackend, NullBackend

        if sim_type == SimulatorType.GAZEBO:
            return GazeboBackend(config)
        elif sim_type == SimulatorType.NONE:
            return NullBackend(config)
        else:
            # Not yet implemented - return null backend
            return NullBackend(config)
