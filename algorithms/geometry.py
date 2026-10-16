"""2D geometric optimization, no angular sampling and no ORCA substitution."""

import math
import numpy as np


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]])


def direction(vector, fallback=(1.0, 0.0)):
    length = float(np.linalg.norm(vector))
    return np.asarray(vector, dtype=float) / length if length > 1e-12 else np.array(fallback, dtype=float)


def pair_direction(displacement, own_id, neighbor_id):
    # Coincidence has no geometric line: choose an antisymmetric ID-based axis.
    return direction(displacement, (1.0 if own_id < neighbor_id else -1.0, 0.0))


def braking_speed(distance, radius, slope, acceleration):
    """Paper Eq. (6), m/s. Scalar or array distance; continuous first derivative."""
    if slope <= 0 or acceleration <= 0:
        raise ValueError("positive slope and acceleration required")
    gap = np.maximum(np.asarray(distance, dtype=float) - radius, 0.0)
    knee = acceleration / slope**2
    result = np.where(gap <= knee, slope * gap,
                      np.sqrt(np.maximum(2 * acceleration * gap - acceleration**2 / slope**2, 0)))
    return float(result) if result.ndim == 0 else result


def maximize_on_disk(objective, radius, normals, bounds, tolerance=1e-9):
    """Maximize objective @ u with ||u||<=radius and normals @ u<=bounds.

    In 2D an optimum is the unconstrained circle support, a line/circle
    intersection, or a line/line vertex (also covers singleton feasible sets).
    Include line projections for zero objective/degenerate cases. Return None
    if empty. Normals are normalized to make tolerance a velocity tolerance.
    """
    objective = np.asarray(objective, dtype=float)
    normals = np.asarray(normals, dtype=float).reshape(-1, 2)
    bounds = np.asarray(bounds, dtype=float)
    norms = np.linalg.norm(normals, axis=1)
    if np.any(norms == 0) or radius < 0:
        raise ValueError("nonzero normals and nonnegative radius required")
    normals, bounds = normals / norms[:, None], bounds / norms
    candidates = [np.zeros(2), radius * direction(objective)]
    for normal, bound in zip(normals, bounds):
        foot = bound * normal
        candidates.append(foot)
        if abs(bound) <= radius + tolerance:
            tangent = np.array([-normal[1], normal[0]])
            length = math.sqrt(max(radius**2 - bound**2, 0.0))
            candidates.extend((foot + length * tangent, foot - length * tangent))
    for i in range(len(bounds)):
        for j in range(i):
            matrix = normals[[i, j]]
            if abs(np.linalg.det(matrix)) > 1e-12:
                candidates.append(np.linalg.solve(matrix, bounds[[i, j]]))
    feasible = [u for u in candidates if np.linalg.norm(u) <= radius + tolerance
                and np.all(normals @ u <= bounds + tolerance)]
    if not feasible:
        return None
    # Stable tie break; never round the objective or replace its maximization.
    return max(feasible, key=lambda u: (float(objective @ u), -float(u @ u),
                                        float(u[0]), float(u[1]))).copy()


def constrained_candidate(previous, neighbor_velocity, normal, distance, parameters):
    """Paper C1--C5 (p. 38). Returns candidate or None, and chosen passing side."""
    relative = previous - neighbor_velocity
    signed = cross(normal, relative)
    side = 1 if signed >= -parameters.tolerance else -1
    angle = math.asin(min(parameters.avoidance_radius / max(distance, 1e-300), 1.0))
    tangent = rotate(normal, side * angle)
    # -side*cross(tangent,u-vj)<=0 (C1); same with normal (C2).
    c1 = side * np.array([tangent[1], -tangent[0]])
    c2 = side * np.array([normal[1], -normal[0]])
    limit = braking_speed(distance, parameters.avoidance_radius,
                          parameters.avoidance_slope, parameters.avoidance_acceleration)
    normals = np.stack([c1, c2, normal])
    bounds = np.array([c1 @ neighbor_velocity, c2 @ neighbor_velocity, limit])
    return maximize_on_disk(previous, float(np.linalg.norm(previous)), normals, bounds,
                            parameters.tolerance), side
