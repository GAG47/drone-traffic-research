"""Local state interface based on RobotSim CreatePhase/SelectNearbyVisibleAgents.

This first version uses perfect instantaneous observations and no packet loss.
Self exclusion uses identity, so a DIFFERENT agent at zero distance is retained.
"""

from dataclasses import dataclass
import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from .state import DroneState, readonly_array


@dataclass(frozen=True, slots=True)
class NeighborStates:
    ids: NDArray
    positions: NDArray
    velocities: NDArray
    distances: NDArray


@dataclass(frozen=True, slots=True)
class AgentObservation:
    agent_id: int
    position: NDArray
    velocity: NDArray
    neighbors: NeighborStates


def neighbors_within(state: DroneState, radius: float) -> tuple[NeighborStates, ...]:
    """Inclusive radius; stable row ordering; all arrays detached and read-only."""
    if not math.isfinite(radius) or radius < 0:
        raise ValueError("communication radius must be finite and nonnegative")
    result = []
    for i in range(state.count):
        distances = np.linalg.norm(state.positions - state.positions[i], axis=1)
        mask = distances <= radius
        mask[i] = False
        result.append(
            NeighborStates(
                readonly_array(state.ids[mask], np.int64),
                readonly_array(state.positions[mask]),
                readonly_array(state.velocities[mask]),
                readonly_array(distances[mask]),
            )
        )
    return tuple(result)


class ObservationProvider(Protocol):
    def observe(
        self, state: DroneState, time: float, radius: float
    ) -> tuple[AgentObservation, ...]:
        """Return one observation per state row, preserving IDs and row order.

        Future providers can own sensor state/history and return delayed
        neighbor records without changing the engine or controller signature.
        Called once per integration step, not once per individual agent.
        """
        ...


class PerfectObservation:
    def observe(
        self, state: DroneState, time: float, radius: float
    ) -> tuple[AgentObservation, ...]:
        return tuple(
            AgentObservation(
                int(state.ids[i]),
                readonly_array(state.positions[i]),
                readonly_array(state.velocities[i]),
                neighbors,
            )
            for i, neighbors in enumerate(neighbors_within(state, radius))
        )
