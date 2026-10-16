from dataclasses import dataclass

import numpy as np

from controllers.basic import FixedVelocityController
from simulator import DroneState, DynamicsParameters, SimulationConfig


SCENARIO_NAMES = ("single", "head_on", "five_agents")


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    initial: DroneState
    commands: np.ndarray
    config: SimulationConfig
    duration: float

    def controller(self) -> FixedVelocityController:
        return FixedVelocityController(dict(zip(self.initial.ids, self.commands)))


def make_scenario(name: str, dt: float = 0.01) -> Scenario:
    """All positions/speeds and small radii are test settings, not paper benchmarks."""
    if name == "single":
        ids, positions, commands = [10], [[0, 0]], [[8, 0]]
        radius, duration = 80.0, 10.0
        description = "One drone at rest; 8 m/s command activates the 6 m/s² cap."
    elif name == "head_on":
        ids = [10, 20]
        positions, commands = [[-10, 0], [10, 0]], [[2, 0], [-2, 0]]
        radius, duration = 8.0, 12.0
        description = "Two independent drones approach and cross; no avoidance controller."
    elif name == "five_agents":
        ids = [10, 20, 30, 40, 50]
        positions = [[-6, -3], [-2, 0], [2, 0], [6, 3], [0, 6]]
        commands = [[2, 1], [0.5, -1.5], [-1, 0.5], [-2, -1], [0, -2]]
        radius, duration = 6.0, 10.0
        description = "Five distinct fixed commands; changing local neighborhoods."
    else:
        raise ValueError(f"unknown scenario {name!r}; choose from {SCENARIO_NAMES}")
    return Scenario(
        name, description, DroneState.at_rest(ids, positions), np.array(commands, dtype=float),
        SimulationConfig(
            dt=dt, control_period=0.2, communication_radius=radius,
            dynamics=DynamicsParameters(relaxation_time=1.0, max_acceleration=6.0),
        ),
        duration,
    )
