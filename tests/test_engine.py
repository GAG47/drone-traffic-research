from dataclasses import replace

import numpy as np
import pytest

from controllers.basic import FixedVelocityController
from scenarios.basic_scenarios import SCENARIO_NAMES, make_scenario
from simulator import DroneState, DynamicsParameters, Simulation, SimulationConfig
from simulator.neighbors import PerfectObservation


def test_control_interval_holds_commands_and_clock_continues():
    calls = []

    def controller(obs, time):
        calls.append(time)
        return [1, 0] if time < 0.15 else [3, 0]

    config = SimulationConfig(0.1, 0.2, 10)
    sim = Simulation(DroneState.at_rest([1], [[0, 0]]), controller, config)
    sim.run(0.2)
    sim.run(0.3)
    data = sim.recorder.arrays()
    np.testing.assert_allclose(calls, [0, 0.2, 0.4])
    np.testing.assert_allclose(data["times"], np.arange(6) * 0.1)
    np.testing.assert_array_equal(data["applied_desired_velocities"][:, 0, 0], [1, 1, 3, 3, 3])
    assert data["accelerations"].shape == (5, 1, 2)
    assert sim.time == 0.5


def test_all_controllers_see_same_snapshot_and_row_order_is_irrelevant():
    def controller(obs, time):
        return obs.neighbors.velocities[0]

    initial = DroneState([3, 7], [[0, 0], [1, 0]], [[1, 0], [3, 0]], np.zeros((2, 2)))
    config = SimulationConfig(0.1, 0.1, 5, DynamicsParameters(1, 100))
    sim = Simulation(initial, controller, config)
    sim.step()
    np.testing.assert_allclose(sim.state.velocities, [[1.2, 0], [2.8, 0]])
    sim.run(0.9)
    reversed_state = DroneState(
        initial.ids[::-1], initial.positions[::-1], initial.velocities[::-1],
        initial.desired_velocities[::-1],
    )
    other = Simulation(reversed_state, controller, config)
    other.run(1)
    np.testing.assert_array_equal(sim.state.velocities, other.state.velocities[::-1])
    np.testing.assert_array_equal(sim.state.positions, other.state.positions[::-1])
    np.testing.assert_array_equal(initial.velocities, [[1, 0], [3, 0]])


def test_two_agents_equal_two_independent_simulations():
    scenario = make_scenario("head_on")
    together = Simulation(scenario.initial, scenario.controller(), scenario.config)
    together.run(scenario.duration)
    data = together.recorder.arrays()
    for i, agent_id in enumerate(scenario.initial.ids):
        alone = Simulation(
            DroneState.at_rest([agent_id], [scenario.initial.positions[i]]),
            scenario.controller(), scenario.config,
        )
        alone.run(scenario.duration)
        independent = alone.recorder.arrays()
        for key in ("positions", "velocities"):
            np.testing.assert_array_equal(data[key][:, i], independent[key][:, 0])
    np.testing.assert_allclose(data["positions"][:, 0], -data["positions"][:, 1], atol=1e-14)
    assert data["positions"][-1, 0, 0] > data["positions"][-1, 1, 0]
    # Exact discrete distance until/after crossing, including startup lag.
    t = data["times"]
    x0 = -10 + 2 * t - 2 * (1 - 0.99 ** np.arange(len(t)))
    distance = np.linalg.norm(data["positions"][:, 0] - data["positions"][:, 1], axis=1)
    np.testing.assert_allclose(distance, 2 * np.abs(x0), atol=1e-12)


@pytest.mark.parametrize("name", SCENARIO_NAMES)
def test_reproducible_full_scenarios_and_neighbor_logs(name):
    scenario = make_scenario(name)
    sims = [Simulation(scenario.initial, scenario.controller(), scenario.config) for _ in range(2)]
    for sim in sims:
        sim.run(scenario.duration)
    first, second = [s.recorder.arrays() for s in sims]
    for key in first:
        np.testing.assert_array_equal(first[key], second[key])
    # Independent per-pair calculation at beginning, midpoint, and end.
    for k in (0, len(first["times"]) // 2, -1):
        expected = []
        for i in range(scenario.initial.count):
            expected.append(sum(
                i != j and sum((first["positions"][k, i] - first["positions"][k, j]) ** 2)
                <= scenario.config.communication_radius**2
                for j in range(scenario.initial.count)
            ))
        np.testing.assert_array_equal(first["neighbor_counts"][k], expected)


def test_observation_provider_extension_reaches_dynamics():
    class BiasedVelocity(PerfectObservation):
        def observe(self, state, time, radius):
            return tuple(
                replace(obs, velocity=obs.velocity + [0.5, 0])
                for obs in super().observe(state, time, radius)
            )

    sim = Simulation(
        DroneState.at_rest([1], [[0, 0]]), FixedVelocityController({1: [2, 0]}),
        SimulationConfig(0.1, 0.1, 10), BiasedVelocity(),
    )
    sim.step()
    np.testing.assert_allclose(sim.state.velocities, [[0.15, 0]])
    np.testing.assert_array_equal(sim.state.positions, [[0, 0]])


def test_rejected_control_output_does_not_advance_state_or_time():
    sim = Simulation(DroneState.at_rest([1], [[0, 0]]), lambda obs, time: [float("nan"), 0])
    original = sim.state
    with pytest.raises(ValueError):
        sim.step()
    assert sim.time == 0
    assert sim.state is original
    assert sim.recorder.arrays()["times"].tolist() == [0]


def test_zero_duration_and_invalid_duration():
    sim = Simulation(DroneState.at_rest([1], [[0, 0]]), FixedVelocityController({1: [0, 0]}))
    sim.run(0)
    assert sim.recorder.arrays()["accelerations"].shape == (0, 1, 2)
    for duration in (-1, 0.015, float("nan")):
        with pytest.raises(ValueError):
            sim.run(duration)


def test_control_period_requires_integral_steps():
    with pytest.raises(ValueError, match="integer multiple"):
        SimulationConfig(dt=0.03, control_period=0.2)
