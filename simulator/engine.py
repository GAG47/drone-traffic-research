"""Fixed-step engine with a common immutable snapshot and held control commands."""

from dataclasses import dataclass, field
import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from .dynamics import DynamicsParameters, advance
from .logging import TrajectoryRecorder
from .neighbors import AgentObservation, ObservationProvider, PerfectObservation
from .state import DroneState


def integral_steps(duration: float, dt: float, name: str = "duration") -> int:
    if not math.isfinite(duration) or duration < 0:
        raise ValueError(f"{name} must be finite and nonnegative")
    ratio = duration / dt
    steps = round(ratio)
    if not math.isclose(ratio, steps, rel_tol=1e-12, abs_tol=1e-10):
        raise ValueError(f"{name} must be an integer multiple of dt")
    return steps


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    dt: float = 0.01
    control_period: float = 0.2
    communication_radius: float = 80.0
    dynamics: DynamicsParameters = field(default_factory=DynamicsParameters)

    def __post_init__(self) -> None:
        if not math.isfinite(self.dt) or not 0 < self.dt <= self.dynamics.relaxation_time:
            raise ValueError("require 0 < dt <= relaxation_time")
        if integral_steps(self.control_period, self.dt, "control_period") < 1:
            raise ValueError("control_period must be at least dt")
        if not math.isfinite(self.communication_radius) or self.communication_radius < 0:
            raise ValueError("communication_radius must be finite and nonnegative")


class Controller(Protocol):
    def __call__(self, observation: AgentObservation, time: float) -> NDArray:
        """One desired velocity in SI. Must not mutate observations.

        Each call sees the same time-step snapshot; any controller memory should
        be keyed by agent ID so row reordering does not change the decisions.
        """
        ...


class Simulation:
    def __init__(
        self,
        initial_state: DroneState,
        controller: Controller,
        config: SimulationConfig | None = None,
        observations: ObservationProvider | None = None,
    ):
        self.config = config if config is not None else SimulationConfig()
        self.controller = controller
        self.observations = observations if observations is not None else PerfectObservation()
        self._state = initial_state
        self._step_index = 0
        self._control_steps = integral_steps(
            self.config.control_period, self.config.dt, "control_period"
        )
        self.recorder = TrajectoryRecorder(initial_state, self.config.communication_radius)

    @property
    def state(self) -> DroneState:
        return self._state

    @property
    def time(self) -> float:
        # Avoid accumulation of floating point clock drift.
        return self._step_index * self.config.dt

    def step(self) -> DroneState:
        snapshot = self._state
        observations = self.observations.observe(
            snapshot, self.time, self.config.communication_radius
        )
        if len(observations) != snapshot.count or any(
            obs.agent_id != int(snapshot.ids[i]) for i, obs in enumerate(observations)
        ):
            raise ValueError("observations must preserve state row order and IDs")
        measured_velocities = np.asarray([obs.velocity for obs in observations])
        commands = snapshot.desired_velocities
        if self._step_index % self._control_steps == 0:
            # Collect every output BEFORE updating any state.
            commands = np.asarray([self.controller(obs, self.time) for obs in observations])
        next_state, acceleration = advance(
            snapshot, commands, self.config.dt, self.config.dynamics, measured_velocities
        )
        self._state = next_state
        self._step_index += 1
        self.recorder.append(self.time, next_state, acceleration, commands)
        return next_state

    def run(self, duration: float) -> DroneState:
        """Advance for duration seconds; repeated calls continue the same clock."""
        for _ in range(integral_steps(duration, self.config.dt)):
            self.step()
        return self.state
