"""Independent closed-form checks, not a duplicate of the stepping loop."""

import math

import numpy as np
import pytest

from simulator import DroneState, DynamicsParameters, Simulation, SimulationConfig
from simulator.dynamics import advance
from controllers.basic import FixedVelocityController


def constant_command_solution(position, velocity, command, times, tau, acceleration):
    """Exact discrete solution: finite arithmetic series + geometric tail.

    Error direction is constant. m = ceil((|u-v0|-a*tau)/(a*dt)) capped
    steps bring the error into the unsaturated region. Thereafter q=1-dt/tau.
    This computes whole trajectories without calling a time-stepping recurrence.
    """
    dt = times[1] - times[0]
    delta = np.asarray(command) - velocity
    magnitude = np.linalg.norm(delta)
    if magnitude == 0:
        return position + times[:, None] * velocity, np.tile(velocity, (len(times), 1))
    direction = delta / magnitude
    m = max(0, math.ceil((magnitude - acceleration * tau) / (acceleration * dt) - 1e-12))
    x, v = [], []
    for n in range(len(times)):
        capped = min(n, m)
        tail = n - capped
        end_error = magnitude - capped * acceleration * dt
        q_tail = (1 - dt / tau) ** tail
        x.append(
            position + capped * dt * velocity
            + 0.5 * acceleration * dt**2 * capped * (capped - 1) * direction
            + tail * dt * command - tau * end_error * (1 - q_tail) * direction
        )
        v.append(command - end_error * q_tail * direction)
    return np.asarray(x), np.asarray(v)


@pytest.mark.parametrize("dt", [0.01, 0.05, 0.1, 0.2, 0.5])
def test_geometric_response_and_position(dt):
    initial = DroneState([7], [[1, -2]], [[0.25, -0.5]], [[0, 0]])
    command = np.array([3.0, 4.0])
    sim = Simulation(
        initial, FixedVelocityController({7: command}),
        SimulationConfig(dt, dt, 10, DynamicsParameters(1, 20)),
    )
    sim.run(5)
    data = sim.recorder.arrays()
    qn = (1 - dt) ** np.arange(len(data["times"]))
    expected_v = command + (initial.velocities[0] - command) * qn[:, None]
    expected_x = (
        initial.positions[0] + data["times"][:, None] * command
        + (initial.velocities[0] - command) * (1 - qn[:, None])
    )
    np.testing.assert_allclose(data["velocities"][:, 0], expected_v, atol=1e-13)
    np.testing.assert_allclose(data["positions"][:, 0], expected_x, atol=1e-12)
    errors = np.linalg.norm(data["velocities"][:, 0] - command, axis=1)
    assert np.all(np.diff(errors) <= 1e-12)


@pytest.mark.parametrize("command,initial_velocity,a_max", [
    ([8, 0], [0, 0], 6),
    ([8, 6], [-1, 2], 3),
    ([0, 0], [10, -5], 2),
])
def test_capped_then_geometric_closed_form(command, initial_velocity, a_max):
    initial = DroneState([1], [[3, -4]], [initial_velocity], [[0, 0]])
    config = SimulationConfig(0.01, 0.2, 20, DynamicsParameters(1, a_max))
    sim = Simulation(initial, FixedVelocityController({1: command}), config)
    sim.run(10)
    data = sim.recorder.arrays()
    x, v = constant_command_solution(
        initial.positions[0], initial.velocities[0], np.asarray(command),
        data["times"], 1, a_max,
    )
    np.testing.assert_allclose(data["positions"][:, 0], x, atol=2e-12)
    np.testing.assert_allclose(data["velocities"][:, 0], v, atol=2e-12)
    assert np.max(np.linalg.norm(data["accelerations"], axis=2)) <= a_max + 1e-12


def test_zero_command_at_rest_stays_put():
    initial = DroneState.at_rest([13], [[12, -9]])
    sim = Simulation(initial, FixedVelocityController({13: [0, 0]}))
    sim.run(1)
    data = sim.recorder.arrays()
    np.testing.assert_array_equal(data["positions"], np.tile([[[12, -9]]], (101, 1, 1)))
    assert np.count_nonzero(data["velocities"]) == 0
    assert np.count_nonzero(data["accelerations"]) == 0


def test_position_uses_old_actual_velocity_not_new_or_desired():
    state = DroneState.at_rest([1], [[0, 0]])
    new, _ = advance(state, [[10, 0]], 0.1, DynamicsParameters(1, 2))
    np.testing.assert_array_equal(new.positions, [[0, 0]])
    np.testing.assert_allclose(new.velocities, [[0.2, 0]])
    later, _ = advance(new, [[10, 0]], 0.1, DynamicsParameters(1, 2))
    np.testing.assert_allclose(later.positions, [[0.02, 0]])


def test_cap_preserves_direction_not_componentwise_clip():
    state = DroneState.at_rest([1], [[0, 0]])
    new, acceleration = advance(state, [[30, 40]], 0.1, DynamicsParameters(1, 2))
    np.testing.assert_allclose(new.velocities, [[0.12, 0.16]])
    np.testing.assert_allclose(acceleration, [[1.2, 1.6]])


def test_true_velocity_plus_measured_velocity_tracking_error():
    state = DroneState([1], [[0, 0]], [[1, 2]], [[0, 0]])
    new, _ = advance(state, [[4, 3]], 0.1, DynamicsParameters(1, 100), [[1.5, 1.0]])
    np.testing.assert_allclose(new.velocities, [[1.25, 2.2]])
    np.testing.assert_allclose(new.positions, [[0.1, 0.2]])


@pytest.mark.parametrize("dt", [0, -0.1, float("nan"), 1.01])
def test_reject_invalid_or_nonmonotone_step(dt):
    with pytest.raises(ValueError):
        advance(DroneState.at_rest([1], [[0, 0]]), [[1, 0]], dt, DynamicsParameters())


@pytest.mark.parametrize("params", [(0, 1), (1, 0), (-1, 1), (1, float("inf"))])
def test_invalid_dynamics_parameters(params):
    with pytest.raises(ValueError):
        DynamicsParameters(*params)
