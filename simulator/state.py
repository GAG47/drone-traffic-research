"""SI state arrays, corresponding to RobotSim utilities/dynamics_utils.h:phase_t."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


def readonly_array(value: ArrayLike, dtype=np.float64) -> NDArray:
    """Own a copy: callers and successive snapshots cannot alias writable data."""
    array = np.array(value, dtype=dtype, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True)
class DroneState:
    """One synchronous snapshot. Rows are agents; columns are spatial components.

    The current public state is 2D. Vector operations elsewhere use the dimension
    property so a later 3D extension can add a separate vertical relaxation time.
    desired_velocities holds the last applied command (initially user supplied).
    """

    ids: NDArray
    positions: NDArray
    velocities: NDArray
    desired_velocities: NDArray

    def __post_init__(self) -> None:
        ids = np.asarray(self.ids)
        if ids.ndim != 1 or ids.size == 0 or ids.dtype.kind not in "iu":
            raise ValueError("ids must be a nonempty one-dimensional integer array")
        if len(np.unique(ids)) != len(ids):
            raise ValueError("agent IDs must be unique")
        object.__setattr__(self, "ids", readonly_array(ids, np.int64))
        for name in ("positions", "velocities", "desired_velocities"):
            array = readonly_array(getattr(self, name))
            if array.shape != (len(ids), 2) or not np.isfinite(array).all():
                raise ValueError(f"{name} must be finite with shape (N, 2)")
            object.__setattr__(self, name, array)

    @property
    def count(self) -> int:
        return len(self.ids)

    @property
    def dimension(self) -> int:
        return self.positions.shape[1]

    @property
    def speeds(self) -> NDArray:
        return np.linalg.norm(self.velocities, axis=1)

    @classmethod
    def at_rest(cls, ids: ArrayLike, positions: ArrayLike) -> "DroneState":
        positions = np.asarray(positions, dtype=float)
        return cls(ids, positions, np.zeros_like(positions), np.zeros_like(positions))
