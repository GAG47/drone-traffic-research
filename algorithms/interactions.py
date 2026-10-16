"""Section 3.2/3.3 anisotropic repulsion and selective friction, Eqs. (2)--(6)."""

import math
import numpy as np

from .geometry import braking_speed, cross, direction, pair_direction, rotate


def within_angle(vector, reference, cosine, tolerance=1e-9):
    norm = float(np.linalg.norm(vector) * np.linalg.norm(reference))
    return norm > tolerance and float(vector @ reference) >= cosine * norm - tolerance


def repulsion(observation, goal, parameters):
    delta = np.asarray(goal) - observation.position
    goal_defined = np.linalg.norm(delta) > parameters.tolerance
    g = direction(delta)
    total = np.zeros(2)
    neighbors = observation.neighbors
    for j in np.argsort(neighbors.ids):
        q = neighbors.positions[j] - observation.position
        d = float(np.linalg.norm(q))
        if d >= parameters.repulsion_radius:
            continue
        n = pair_direction(q, observation.agent_id, int(neighbors.ids[j]))
        if not goal_defined or d <= parameters.tolerance:
            # Undefined target axis / coincident positions: isotropic separation.
            rep_direction = -n
        else:
            phi = math.acos(float(np.clip(n @ g, -1.0, 1.0)))
            speed = float(np.linalg.norm(neighbors.velocities[j]))
            parallel = speed > parameters.tolerance and float(neighbors.velocities[j] @ g) > 0.5 * speed
            A = parameters.anisotropy
            if parallel:
                rho = (1 - A) * phi if phi <= math.pi / 2 else math.pi + (1 - A) * (phi - math.pi)
            else:
                rho = (1 - A / 2) * (phi - math.pi) + math.pi
            # Reflect the unsigned paper angle; exact head-on repels to the
            # same own-frame side as the Self-drive symmetric tie convention.
            sign = 1 if cross(g, n) > parameters.tolerance else -1
            rep_direction = rotate(-g, sign * rho)
        total += parameters.repulsion_gain * (parameters.repulsion_radius - d) * rep_direction
    return total


def friction_selection(displacement, neighbor_velocity, goal_direction, tolerance=1e-9):
    """F1 toward self (explicit sign completion), F2 +/-120 deg, F3 +/-90 deg.

    A stationary neighbor has no heading: F1=False, F3=True (not moving away).
    Coincidence counts as in front/toward; a zero goal vector gives F2/F3=False
    for moving neighbors. These degenerate conventions are engineering choices.
    """
    coincident = np.linalg.norm(displacement) <= tolerance
    stationary = np.linalg.norm(neighbor_velocity) <= tolerance
    f1 = coincident or within_angle(neighbor_velocity, -displacement, math.cos(math.pi / 4), tolerance)
    f2 = coincident or within_angle(displacement, goal_direction, -0.5, tolerance)
    f3 = stationary or within_angle(neighbor_velocity, goal_direction, 0.0, tolerance)
    return (bool(f1), bool(f2), bool(f3))


def friction(observation, goal, parameters):
    g = np.asarray(goal) - observation.position
    total = np.zeros(2)
    neighbors = observation.neighbors
    for j in np.argsort(neighbors.ids):
        q = neighbors.positions[j] - observation.position
        f1, f2, f3 = friction_selection(q, neighbors.velocities[j], g, parameters.tolerance)
        if not ((f1 and f2) or ((f1 or f2) and f3)):
            continue
        relative = neighbors.velocities[j] - observation.velocity
        speed = float(np.linalg.norm(relative))
        allowed = max(parameters.friction_slack,
                      braking_speed(float(np.linalg.norm(q)), parameters.friction_radius,
                                    parameters.friction_slope, parameters.friction_acceleration))
        if speed > allowed and speed > parameters.tolerance:
            # Deliberate Eq. (4) amplitude hypothesis, NOT literal Heaviside.
            total += parameters.friction_gain * (speed - allowed) * relative / speed
    return total
