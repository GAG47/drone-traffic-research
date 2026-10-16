"""Test-only prescribed velocities. This is NOT the paper's Self-drive."""

from collections.abc import Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from simulator.neighbors import AgentObservation
from simulator.state import readonly_array


class FixedVelocityController:
    def __init__(self, velocity_by_id: Mapping[int, ArrayLike]):
        self._velocities = {}
        for agent_id, velocity in velocity_by_id.items():
            value = readonly_array(velocity)
            if value.shape != (2,) or not np.isfinite(value).all():
                raise ValueError("fixed velocities must be finite 2D vectors")
            self._velocities[int(agent_id)] = value

    def __call__(self, observation: AgentObservation, time: float) -> NDArray:
        return self._velocities[observation.agent_id]
