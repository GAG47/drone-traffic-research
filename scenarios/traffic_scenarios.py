"""Small deterministic tests designed here; NOT paper Fig. 2 experiment inputs."""

from dataclasses import dataclass, field
import numpy as np

from simulator.engine import SimulationConfig
from simulator.state import DroneState


SCENARIO_NAMES = ("single", "head_on", "crossing", "crossing_offset", "overtaking", "five_agents",
                  "crossing_cruise", "five_agents_cruise", "close_parallel")


@dataclass(frozen=True)
class TrafficScenario:
    name: str
    description: str
    initial: DroneState
    goals: dict
    duration: float
    cruise_speeds: dict = field(default_factory=dict)
    config: SimulationConfig = field(default_factory=SimulationConfig)


def make_traffic_scenario(name, dt=0.01):
    if name in ("crossing_cruise", "five_agents_cruise"):
        base = make_traffic_scenario(name.removesuffix('_cruise'), dt)
        delta = np.array(list(base.goals.values()))-base.initial.positions
        velocities = 8*delta/np.linalg.norm(delta, axis=1)[:, None]
        initial = DroneState(base.initial.ids, base.initial.positions, velocities, velocities)
        return TrafficScenario(name, base.description+'; cruise-speed initial condition sensitivity',
                               initial, base.goals, base.duration, base.cruise_speeds, base.config)
    velocities = None
    speeds = {}
    if name == "single":
        positions, goals, duration = [[0, 0]], [[80, 0]], 30.0
        description = "A: single agent goal navigation and braking"
    elif name == "head_on":
        positions, goals, duration = [[-60, 0], [60, 0]], [[60, 0], [-60, 0]], 50.0
        description = "B: symmetric head-on, 120 m initial separation, initially at rest"
    elif name == "crossing":
        positions, goals, duration = [[-60, 0], [0, -60]], [[60, 0], [0, 60]], 50.0
        description = "C1: perpendicular crossing with simultaneous nominal arrival"
    elif name == "crossing_offset":
        positions, goals, duration = [[-30, 0], [0, -100]], [[90, 0], [0, 60]], 50.0
        description = "C2: same crossing point, nominal arrivals separated by 8.75 s"
    elif name == "overtaking":
        positions, goals, duration = [[-70, 0], [-25, 0]], [[110, 0], [90, 0]], 60.0
        velocities = [[8, 0], [4, 0]]
        speeds = {10: 8.0, 20: 4.0}  # Deliberate heterogeneous test, not S2 homogeneous fleet.
        description = "D: 8 m/s follower and 4 m/s leader, 45 m initial gap"
    elif name == "close_parallel":
        positions, goals, duration = [[0, 0], [8, 3]], [[80, 0], [100, 3]], 50.0
        velocities = [[4, 0], [4, 0]]
        speeds = {10: 4.0, 20: 4.0}
        description = "D2: same initial velocity, 8.54 m safe gap; activates close-range repulsion"
    elif name == "five_agents":
        angles = 2 * np.pi * np.arange(5) / 5
        positions = np.column_stack((70 * np.cos(angles), 70 * np.sin(angles)))
        goals, duration = -positions, 70.0
        description = "E: five agents from a radius-70 m pentagon toward opposite goals"
    else:
        raise ValueError(f"unknown traffic scenario: {name}")
    positions, goals = np.asarray(positions, float), np.asarray(goals, float)
    ids = 10 * np.arange(1, len(positions) + 1)
    velocities = np.zeros_like(positions) if velocities is None else np.asarray(velocities, float)
    initial = DroneState(ids, positions, velocities, np.zeros_like(positions))
    return TrafficScenario(name, description, initial, dict(zip(map(int, ids), goals)), duration,
                           speeds, SimulationConfig(dt=dt))
