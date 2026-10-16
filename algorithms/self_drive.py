"""Dynamic predictive Self-drive, Section 3.4, pp. 36--38 / Fig. 1."""

from dataclasses import dataclass
import math
import numpy as np

from .geometry import braking_speed, constrained_candidate, pair_direction


@dataclass(frozen=True, slots=True)
class Danger:
    neighbor_id: int
    row: int
    conditions: tuple[bool, bool, bool, bool]
    urgency: float
    distance: float
    normal: np.ndarray

    @property
    def threatening(self):
        return all(self.conditions)


def danger_conditions(candidate, neighbor_velocity, displacement, horizon, parameters,
                      own_id=0, neighbor_id=1, row=0):
    """D1--D4. Tangency is allowed; zero relative velocity is not threatening.

    D4 uses the paper's distance-gap / relative-speed surrogate, not quadratic
    time of collision. At overlap, its negative value is retained.
    """
    distance = float(np.linalg.norm(displacement))
    normal = pair_direction(displacement, own_id, neighbor_id)
    relative = candidate - neighbor_velocity
    speed = float(np.linalg.norm(relative))
    angle = math.asin(min(parameters.avoidance_radius / max(distance, 1e-300), 1.0))
    projection = float(relative @ normal)
    limit = braking_speed(distance, parameters.avoidance_radius,
                          parameters.avoidance_slope, parameters.avoidance_acceleration)
    urgency = (distance - parameters.avoidance_radius) / speed if speed > parameters.tolerance else math.inf
    conditions = (
        speed > parameters.tolerance and projection > speed * math.cos(angle) + parameters.tolerance,
        projection > limit + parameters.tolerance,
        float(candidate @ normal) > parameters.tolerance,
        urgency < horizon,
    )
    return Danger(int(neighbor_id), row, tuple(bool(x) for x in conditions), urgency, distance, normal)


def inspect_neighbors(candidate, observation, horizon, parameters):
    neighbors = observation.neighbors
    return [danger_conditions(candidate, neighbors.velocities[j],
                              neighbors.positions[j] - observation.position,
                              horizon, parameters, observation.agent_id, int(neighbors.ids[j]), j)
            for j in range(len(neighbors.ids))]


def goal_navigation(observation, goal, parameters):
    """No avoidance baseline; same goal braking and arrival criterion as Self-drive."""
    delta = np.asarray(goal) - observation.position
    distance = float(np.linalg.norm(delta))
    if distance <= parameters.arrival_radius and np.linalg.norm(observation.velocity) <= parameters.arrival_speed:
        return np.zeros(2)
    if distance <= parameters.tolerance:
        return np.zeros(2)
    speed = min(parameters.cruise_speed, braking_speed(distance, 0, parameters.avoidance_slope,
                                                      parameters.avoidance_acceleration))
    return delta * (speed / distance)


def self_drive(observation, goal, parameters):
    """Return (velocity, JSON-compatible trace); no input mutation or global state.

    Engineering completions: fixed d_goal/vSPP horizon per call, CCW relative
    passing side on exact ties, 32-step budget, zero-command failure, recheck
    after goal slowdown. A failure means braking, NOT guaranteed safety.
    """
    delta = np.asarray(goal) - observation.position
    distance = float(np.linalg.norm(delta))
    horizon = distance / parameters.cruise_speed
    trace = {"horizon_s": horizon, "iterations": 0, "steps": [], "fallback": False,
             "restored_goal_direction": False, "goal_slowdown": False}
    if distance <= parameters.tolerance or (
        distance <= parameters.arrival_radius and np.linalg.norm(observation.velocity) <= parameters.arrival_speed
    ):
        trace["exit_reason"] = "at_goal"
        return np.zeros(2), trace
    goal_direction = delta / distance
    candidate = parameters.cruise_speed * goal_direction
    trace["initial_velocity"] = candidate.tolist()

    def resolve(current, phase):
        visited = []
        while True:
            threats = [d for d in inspect_neighbors(current, observation, horizon, parameters) if d.threatening]
            if not threats:
                return current, None
            if trace["iterations"] >= parameters.max_iterations:
                trace["unresolved_threats"] = sorted(d.neighbor_id for d in threats)
                return np.zeros(2), "iteration_limit"
            if any(np.linalg.norm(current - old) <= parameters.tolerance for old in visited):
                trace["unresolved_threats"] = sorted(d.neighbor_id for d in threats)
                return np.zeros(2), "cycle"
            visited.append(current.copy())
            selected = min(threats, key=lambda d: (d.urgency, d.neighbor_id))
            updated, side = constrained_candidate(current, observation.neighbors.velocities[selected.row],
                                                   selected.normal, selected.distance, parameters)
            trace["iterations"] += 1
            trace["steps"].append({
                "phase": phase, "threats": sorted(d.neighbor_id for d in threats),
                "selected_id": selected.neighbor_id, "urgency_s": selected.urgency,
                "distance_m": selected.distance, "D1_D4": list(selected.conditions),
                "before": current.tolist(), "after": None if updated is None else updated.tolist(),
                "passing_side": side, "solver": "infeasible" if updated is None else "optimal",
            })
            if updated is None:
                return np.zeros(2), "infeasible"
            current = updated

    candidate, failure = resolve(candidate, "avoidance")
    if failure is None:
        speed = float(np.linalg.norm(candidate))
        if speed < parameters.cruise_speed and float(delta @ candidate) < 0:
            restored = speed * goal_direction
            if not any(d.threatening for d in inspect_neighbors(restored, observation, horizon, parameters)):
                candidate = restored
                trace["restored_goal_direction"] = True
        speed = float(np.linalg.norm(candidate))
        limit = braking_speed(distance, 0, parameters.avoidance_slope, parameters.avoidance_acceleration)
        if speed > limit:
            candidate = candidate * (limit / speed)
            trace["goal_slowdown"] = True
            candidate, failure = resolve(candidate, "goal_slowdown_recheck")
    trace["fallback"] = failure is not None
    trace["exit_reason"] = failure or ("resolved" if trace["iterations"] else "no_threat")
    trace["final_velocity"] = candidate.tolist()
    return candidate, trace
