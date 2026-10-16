"""Per-ID goals, Section 3 velocity sum; plugs into the unchanged V1 engine."""

from dataclasses import replace
from time import perf_counter
import numpy as np

from algorithms.parameters import TrafficParameters
from algorithms.self_drive import goal_navigation, self_drive
from algorithms.interactions import repulsion, friction
from simulator.state import readonly_array


class TrafficController:
    MODES = ("navigation", "self_drive", "full")

    def __init__(self, goals, parameters=None, mode="full", cruise_speeds=None, record=True):
        if mode not in self.MODES:
            raise ValueError(f"mode must be one of {self.MODES}")
        self.parameters = parameters or TrafficParameters()
        self.mode = mode
        self.goals = {int(i): readonly_array(goal) for i, goal in goals.items()}
        if not self.goals or any(goal.shape != (2,) or not np.isfinite(goal).all() for goal in self.goals.values()):
            raise ValueError("finite 2D goals required")
        speeds = cruise_speeds or {}
        if not set(speeds).issubset(self.goals):
            raise ValueError("cruise speed IDs must have goals")
        self._parameters = {i: replace(self.parameters, cruise_speed=speeds.get(i, self.parameters.cruise_speed))
                            for i in self.goals}
        self.record = record
        self.diagnostics = []

    def __call__(self, observation, time):
        start = perf_counter()
        goal = self.goals[observation.agent_id]
        parameters = self._parameters[observation.agent_id]
        if self.mode == "navigation":
            own = goal_navigation(observation, goal, parameters)
            trace = {"iterations": 0, "steps": [], "fallback": False, "exit_reason": "navigation"}
        else:
            own, trace = self_drive(observation, goal, parameters)
        rep = repulsion(observation, goal, parameters) if self.mode == "full" else np.zeros(2)
        frict = friction(observation, goal, parameters) if self.mode == "full" else np.zeros(2)
        desired = own + rep + frict  # Eq. (1); no SPP renormalization or extra speed cap.
        elapsed = perf_counter() - start
        if self.record:
            self.diagnostics.append({"time_s": float(time), "agent_id": observation.agent_id,
                                     "self_drive": own.tolist(), "repulsion": rep.tolist(),
                                     "friction": frict.tolist(), "desired": desired.tolist(),
                                     "compute_seconds": elapsed, **trace})
        return desired
