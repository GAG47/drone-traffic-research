"""Full state records plus interval-aligned commands/accelerations, all in SI."""

import csv
from pathlib import Path

import numpy as np

from .neighbors import neighbors_within
from .state import DroneState


class TrajectoryRecorder:
    def __init__(self, initial: DroneState, communication_radius: float):
        self._radius = communication_radius
        self._ids = initial.ids.copy()
        self._dimension = initial.dimension
        self._times = [0.0]
        self._states = [initial]
        self._accelerations = []
        self._commands = []
        self._neighbor_counts = [self._counts(initial)]

    def _counts(self, state: DroneState) -> list[int]:
        # Ground-truth counts at the logged time, not stale control-time counts.
        return [len(group.ids) for group in neighbors_within(state, self._radius)]

    def append(self, time, state, acceleration, applied_command) -> None:
        self._times.append(time)
        self._states.append(state)
        self._accelerations.append(np.array(acceleration, copy=True))
        self._commands.append(np.array(applied_command, copy=True))
        self._neighbor_counts.append(self._counts(state))

    def arrays(self) -> dict[str, np.ndarray]:
        shape = (-1, len(self._ids), self._dimension)
        return {
            "times": np.asarray(self._times),
            "ids": self._ids.copy(),
            "positions": np.stack([s.positions for s in self._states]),
            "velocities": np.stack([s.velocities for s in self._states]),
            "desired_velocities": np.stack([s.desired_velocities for s in self._states]),
            "accelerations": np.asarray(self._accelerations).reshape(shape),
            "applied_desired_velocities": np.asarray(self._commands).reshape(shape),
            "neighbor_counts": np.asarray(self._neighbor_counts, dtype=np.int64),
        }

    def save(self, directory: str | Path) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        data = self.arrays()
        np.savez_compressed(directory / "trajectory.npz", **data)
        with (directory / "trajectory.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([
                "time_s", "agent_id", "x_m", "y_m", "vx_m_s", "vy_m_s",
                "held_desired_vx_m_s", "held_desired_vy_m_s", "speed_m_s",
                "neighbor_count",
            ])
            for k, time in enumerate(data["times"]):
                for i, agent_id in enumerate(data["ids"]):
                    writer.writerow([
                        time, agent_id, *data["positions"][k, i],
                        *data["velocities"][k, i], *data["desired_velocities"][k, i],
                        np.linalg.norm(data["velocities"][k, i]), data["neighbor_counts"][k, i],
                    ])
        with (directory / "speed.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["time_s", "agent_id", "speed_m_s", "held_desired_speed_m_s"])
            for k, time in enumerate(data["times"]):
                for i, agent_id in enumerate(data["ids"]):
                    writer.writerow([
                        time, agent_id, np.linalg.norm(data["velocities"][k, i]),
                        np.linalg.norm(data["desired_velocities"][k, i]),
                    ])
