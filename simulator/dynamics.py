"""Noise-free horizontal RobotSim dynamics, in m, s, m/s and m/s².

Reference: robotmodel.c:RealCoptForceLaw (183–240) and Step (279–375),
commit ea54b97e7e2659449e721ebd4a2285beb9b0e9ce.
Derived from MIT-licensed RobotSim; see THIRD_PARTY_NOTICES.md.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .state import DroneState


@dataclass(frozen=True, slots=True)
class DynamicsParameters:
    relaxation_time: float = 1.0  # tau_PID_XY, seconds
    max_acceleration: float = 6.0  # a_max=600 cm/s² in original defaults

    def __post_init__(self) -> None:
        for name in ("relaxation_time", "max_acceleration"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


def advance(
    state: DroneState,
    desired_velocities: ArrayLike,
    dt: float,
    parameters: DynamicsParameters,
    measured_velocities: ArrayLike | None = None,
) -> tuple[DroneState, NDArray]:
    """Pure synchronous step, returning next state and deterministic acceleration.

    x[k+1] = x[k] + dt*v[k]                 (OLD velocity)
    dv_raw = dt/tau * (command - measured_v)
    v[k+1] = v[k] + norm_clip(dv_raw, a_max*dt)

    measured_velocities is the sensor extension point; by default it is the
    current true velocity. No physical speed cap is added.
    """
    if not math.isfinite(dt) or dt <= 0 or dt > parameters.relaxation_time:
        raise ValueError("require 0 < dt <= relaxation_time for monotone Euler tracking")
    desired = np.asarray(desired_velocities, dtype=float)
    measured = state.velocities if measured_velocities is None else np.asarray(
        measured_velocities, dtype=float
    )
    for name, array in (("desired_velocities", desired), ("measured_velocities", measured)):
        if array.shape != state.velocities.shape or not np.isfinite(array).all():
            raise ValueError(f"{name} must be finite and match state shape")

    raw_velocity = state.velocities + (dt / parameters.relaxation_time) * (
        desired - measured
    )
    delta = raw_velocity - state.velocities
    magnitudes = np.linalg.norm(delta, axis=1)
    saturated = magnitudes / dt > parameters.max_acceleration
    if np.any(saturated):
        # Norm cap, not independent x/y clipping (RobotSim Step).
        delta[saturated] *= (
            parameters.max_acceleration * dt / magnitudes[saturated]
        )[:, None]
    next_state = DroneState(
        state.ids,
        state.positions + state.velocities * dt,
        state.velocities + delta,
        desired,
    )
    return next_state, delta / dt
