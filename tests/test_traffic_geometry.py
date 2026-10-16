"""Independent constraints/objective checks for the paper velocity-space solver."""

from dataclasses import replace
import math
import numpy as np
import pytest

from algorithms.parameters import TrafficParameters
from algorithms.geometry import braking_speed, constrained_candidate, maximize_on_disk
from algorithms.self_drive import danger_conditions


P = TrafficParameters()


def test_braking_piecewise_continuity_and_slope():
    # R=10,p=0.5,a=3: knee at d=22, value=6, derivative=0.5.
    assert braking_speed(9, 10, .5, 3) == 0
    assert braking_speed(10, 10, .5, 3) == 0
    assert braking_speed(14, 10, .5, 3) == 2
    assert braking_speed(22, 10, .5, 3) == 6
    assert braking_speed(30, 10, .5, 3) == pytest.approx(math.sqrt(84))
    h = 1e-5
    assert (braking_speed(22+h, 10, .5, 3)-6)/h == pytest.approx(.5, abs=1e-6)
    assert (6-braking_speed(22-h, 10, .5, 3))/h == pytest.approx(.5, abs=1e-6)
    np.testing.assert_allclose(braking_speed([9, 14, 22], 10, .5, 3), [0, 2, 6])


@pytest.mark.parametrize("u,v,q,horizon,expected", [
    ([8, 0], [-8, 0], [30, 0], 10, (True, True, True, True)),
    ([8, 0], [0, -20], [30, 0], 10, (False, False, True, True)),
    ([1, 0], [0, 0], [30, 0], 100, (True, False, True, True)),
    ([-1, 0], [-20, 0], [30, 0], 10, (True, True, False, True)),
    ([8, 0], [-8, 0], [30, 0], .1, (True, True, True, False)),
    ([8, 0], [8, 0], [30, 0], 10, (False, False, True, False)),
])
def test_each_danger_condition(u, v, q, horizon, expected):
    d = danger_conditions(np.array(u), np.array(v), np.array(q), horizon, P)
    assert d.conditions == expected
    assert d.threatening == all(expected)


def test_d1_tangent_is_not_inside_and_d4_is_paper_surrogate():
    theta = math.asin(13/30)
    u = 8*np.array([math.cos(theta), math.sin(theta)])
    d = danger_conditions(u, np.zeros(2), np.array([30, 0]), 10, P)
    assert not d.conditions[0]
    assert d.urgency == pytest.approx(17/8)


def independently_feasible(points, previous, neighbor, n, distance, side):
    # Independent angle-based C1/C2, squared C3, analytic scalar C4.
    rel = points - neighbor
    angles = np.arctan2(n[0]*rel[:, 1]-n[1]*rel[:, 0], rel @ n)
    alpha = math.asin(min(13 / distance, 1))
    gap = distance - 13
    limit = 0 if gap < 0 else .5*gap if gap <= 12 else math.sqrt(6*gap-36)
    zero = np.linalg.norm(rel, axis=1) < 1e-8
    return ((np.abs(angles) >= alpha-1e-8) | zero) & (
        side*(n[0]*rel[:, 1]-n[1]*rel[:, 0]) >= -1e-8
    ) & (np.sum(points**2, axis=1) <= previous@previous+1e-7) & (points@n <= limit+1e-8)


@pytest.mark.parametrize("previous,neighbor,n,distance", [
    ([8, 0], [-8, 0], [1, 0], 30),
    ([8, 0], [0, 0], [1, 0], 22),
    ([7, -2], [-3, 1], [1, 0], 23),
    ([0, 8], [2, -5], [0, 1], 28),
    ([4, 2], [1, -1], [.8, .6], 12),
    ([0, 0], [0, 0], [1, 0], 30),
])
def test_c1_c5_independent_feasibility_and_global_objective(previous, neighbor, n, distance):
    previous, neighbor, n = map(lambda x: np.array(x, float), (previous, neighbor, n))
    candidate, side = constrained_candidate(previous, neighbor, n, distance, P)
    assert candidate is not None
    assert independently_feasible(candidate[None], previous, neighbor, n, distance, side)[0]
    # Sampling is ONLY an independent test oracle, never used in implementation.
    rng = np.random.default_rng(984)
    samples = rng.uniform(-1, 1, (100000, 2))*np.linalg.norm(previous)
    feasible = samples[independently_feasible(samples, previous, neighbor, n, distance, side)]
    if len(feasible):
        assert np.max(feasible @ previous) <= candidate @ previous + 1e-7


def test_analytic_head_on_optimum_and_infeasible_case():
    candidate, side = constrained_candidate(np.array([8., 0]), np.array([-8., 0]), np.array([1., 0]), 30, P)
    # Translated tangent intersects the radius-8 circle at x=8*cos(2*asin(13/30)).
    alpha = math.asin(13/30)
    np.testing.assert_allclose(candidate, [8*math.cos(2*alpha), 8*math.sin(2*alpha)], atol=1e-9)
    assert side == 1
    assert maximize_on_disk([1, 0], 1, [[1, 0]], [-2]) is None
    candidate, _ = constrained_candidate(np.array([1., 0]), np.array([-100., 0]), np.array([1., 0]), 14, P)
    assert candidate is None


def test_disk_solver_vertex_tangent_parallel_and_zero_radius():
    np.testing.assert_allclose(maximize_on_disk([1, 1], 5, [[1, 0], [0, 1]], [1, 2]), [1, 2])
    np.testing.assert_allclose(maximize_on_disk([1, 0], 1, [[-1, 0], [-2, 0]], [-1, -2]), [1, 0])
    np.testing.assert_allclose(maximize_on_disk([0, 0], 0, [[1, 0]], [0]), [0, 0])


@pytest.mark.parametrize("kwargs", [{"anisotropy": 1.1}, {"cruise_speed": 0}, {"friction_gain": -1},
                                   {"max_iterations": 1.5}, {"avoidance_slope": float("nan")}])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        replace(P, **kwargs)
