from dataclasses import replace
import math
import numpy as np
import pytest

from algorithms.geometry import braking_speed
from algorithms.interactions import repulsion, friction, friction_selection
from algorithms.parameters import TrafficParameters
from algorithms.self_drive import self_drive, inspect_neighbors
from controllers.traffic import TrafficController
from simulator.state import DroneState
from simulator.neighbors import PerfectObservation
from simulator.engine import Simulation
from scenarios.traffic_scenarios import make_traffic_scenario


P = TrafficParameters()


def observation(positions, velocities=None, ids=None):
    positions = np.asarray(positions, float)
    state = DroneState(np.arange(len(positions)) if ids is None else ids, positions,
                       np.zeros_like(positions) if velocities is None else velocities,
                       np.zeros_like(positions))
    return PerfectObservation().observe(state, 0, 1000)[0]


def test_no_threat_and_goal_braking_and_stop():
    obs = observation([[0, 0]])
    u, trace = self_drive(obs, [100, 0], P)
    np.testing.assert_allclose(u, [8, 0])
    assert trace['exit_reason'] == 'no_threat'
    u, trace = self_drive(obs, [4, 0], P)
    np.testing.assert_allclose(u, [2, 0])
    assert trace['goal_slowdown']
    u, trace = self_drive(obs, [0, 0], P)
    assert trace['exit_reason'] == 'at_goal'
    np.testing.assert_array_equal(u, [0, 0])


@pytest.mark.parametrize('positions,velocities', [
    ([[0, 0], [30, 0]], [[8, 0], [-8, 0]]),
    ([[0, 0], [20, 0]], [[8, 0], [0, 0]]),
    ([[0, 0], [0, 0]], [[0, 0], [0, 0]]),
    ([[0, 0], [20, 0]], [[8, 0], [8, 0]]),
])
def test_degenerate_states_finite_deterministic_terminate(positions, velocities):
    obs = observation(positions, velocities)
    u, trace = self_drive(obs, [100, 0], P)
    other, repeated = self_drive(obs, [100, 0], P)
    assert np.isfinite(u).all()
    np.testing.assert_array_equal(u, other)
    assert trace == repeated
    assert trace['iterations'] <= P.max_iterations
    if not trace['fallback']:
        assert not any(d.threatening for d in inspect_neighbors(u, obs, trace['horizon_s'], P))


def test_reciprocal_head_on_tie_has_opposite_world_sides():
    state = DroneState([10, 20], [[-15, 0], [15, 0]], [[8, 0], [-8, 0]], [[0, 0], [0, 0]])
    a, b = PerfectObservation().observe(state, 0, 80)
    ua, _ = self_drive(a, [100, 0], P)
    ub, _ = self_drive(b, [-100, 0], P)
    np.testing.assert_allclose(ua, -ub, atol=1e-10)
    assert ua[1] > 0 and ub[1] < 0


def test_multiple_threats_rechecked_and_urgency_not_input_order():
    obs = observation([[0, 0], [24, -6], [24, 6], [40, 0]],
                      [[8, 0], [0, 0], [0, 0], [-8, 0]], ids=[10, 30, 20, 40])
    u, trace = self_drive(obs, [100, 0], P)
    assert trace['iterations'] >= 2
    assert len(trace['steps'][0]['threats']) >= 2
    threats = [d for d in inspect_neighbors(np.array([8., 0]), obs, 12.5, P) if d.threatening]
    assert trace['steps'][0]['selected_id'] == min(threats, key=lambda d: (d.urgency, d.neighbor_id)).neighbor_id
    limited, limit_trace = self_drive(obs, [100, 0], replace(P, max_iterations=1))
    assert limit_trace['exit_reason'] == 'iteration_limit'
    assert limit_trace['fallback']
    np.testing.assert_array_equal(limited, [0, 0])
    assert trace['iterations'] <= P.max_iterations
    assert trace['exit_reason'] in ('resolved', 'cycle', 'infeasible')


def test_infeasible_is_reported_not_claimed_optimal():
    obs = observation([[0, 0], [14, 0]], [[1, 0], [-100, 0]])
    u, trace = self_drive(obs, [100, 0], replace(P, cruise_speed=1))
    assert trace['exit_reason'] == 'infeasible' and trace['fallback']
    assert trace['steps'][0]['after'] is None
    np.testing.assert_array_equal(u, [0, 0])


def test_repulsion_isotropic_limit_amplitude_and_sum():
    obs = observation([[0, 0], [3, 4], [-6, 0], [11, 0]])
    u = repulsion(obs, [100, 0], replace(P, anisotropy=0))
    expected = -1.3*5.5*np.array([.6, .8]) + 1.3*4.5*np.array([1., 0])
    np.testing.assert_allclose(u, expected, atol=1e-10)


@pytest.mark.parametrize('position,neighbor_velocity,rho', [
    ([5, 0], [8, 0], 0),
    ([0, 5], [8, 0], math.pi/4),
    ([-4, 3], [8, 0], math.pi + .5*(math.acos(-.8)-math.pi)),
    ([0, 5], [-8, 0], 5*math.pi/8),
])
def test_repulsion_equation_three_branches(position, neighbor_velocity, rho):
    obs = observation([[0, 0], position], [[0, 0], neighbor_velocity])
    u = repulsion(obs, [100, 0], P)
    assert np.linalg.norm(u) == pytest.approx(1.3*5.5)
    # Unsigned angle to -goal, checked independently.
    angle = math.acos(float(np.clip(-u[0]/np.linalg.norm(u), -1, 1)))
    assert angle == pytest.approx(rho)


def test_repulsion_mirror_and_coincident_antisymmetry():
    a = repulsion(observation([[0, 0], [3, 4]], [[0, 0], [-2, 0]]), [100, 0], P)
    b = repulsion(observation([[0, 0], [3, -4]], [[0, 0], [-2, 0]]), [100, 0], P)
    np.testing.assert_allclose(a, b*np.array([1, -1]))
    state = DroneState.at_rest([10, 20], [[0, 0], [0, 0]])
    a, b = PerfectObservation().observe(state, 0, 80)
    np.testing.assert_allclose(repulsion(a, [100, 0], P), -repulsion(b, [100, 0], P))


@pytest.mark.parametrize('q,v,flags,apply', [
    ([5, 0], [-2, 0], (True, True, False), True),
    ([-5, 0], [-2, 0], (False, False, False), False),
    ([5, 0], [2, 0], (False, True, True), True),
    ([0, 5], [-2, 0], (False, True, False), False),
    ([-5, 0], [2, 0], (True, False, True), True),
    ([5, 0], [0, 0], (False, True, True), True),
])
def test_friction_filter_truth_table(q, v, flags, apply):
    assert friction_selection(np.array(q), np.array(v), np.array([1., 0])) == flags
    obs = observation([[0, 0], q], [[4, 0], v])
    actual = friction(obs, [100, 0], P)
    if apply:
        relative = np.array(v)-[4, 0]
        expected = (np.linalg.norm(relative)-.5)*relative/np.linalg.norm(relative)
        np.testing.assert_allclose(actual, expected)
    else:
        np.testing.assert_array_equal(actual, [0, 0])


def test_friction_gain_allowed_speed_and_zero_difference():
    obs = observation([[0, 0], [20, 0]], [[8, 0], [4, 0]])
    np.testing.assert_array_equal(friction(obs, [100, 0], P), [0, 0])
    obs = observation([[0, 0], [5, 0]], [[8, 0], [4, 0]])
    np.testing.assert_allclose(friction(obs, [100, 0], replace(P, friction_gain=.25)), [-.875, 0])
    obs = observation([[0, 0], [5, 0]], [[4, 0], [4, 0]])
    np.testing.assert_array_equal(friction(obs, [100, 0], P), [0, 0])


def test_full_sum_and_readonly_observation_and_parameter_override():
    obs = observation([[0, 0], [8, 0]], [[8, 0], [4, 0]], ids=[10, 20])
    copies = [x.copy() for x in (obs.position, obs.velocity, obs.neighbors.positions, obs.neighbors.velocities)]
    controller = TrafficController({10: [100, 0]}, cruise_speeds={10: 6})
    actual = controller(obs, 0)
    p = replace(P, cruise_speed=6)
    expected = self_drive(obs, [100, 0], p)[0] + repulsion(obs, [100, 0], p) + friction(obs, [100, 0], p)
    np.testing.assert_allclose(actual, expected)
    for array, copy in zip((obs.position, obs.velocity, obs.neighbors.positions, obs.neighbors.velocities), copies):
        assert not array.flags.writeable
        np.testing.assert_array_equal(array, copy)


def test_id_row_permutation_does_not_change_decisions():
    state = DroneState([10, 20, 30], [[0, 0], [20, 5], [25, -3]], [[8, 0], [-4, 0], [-5, 0]], np.zeros((3, 2)))
    goals = {10: [100, 0], 20: [-100, 5], 30: [-100, -3]}
    c = TrafficController(goals)
    expected = {o.agent_id: c(o, 0) for o in PerfectObservation().observe(state, 0, 80)}
    ix = [2, 0, 1]
    permuted = DroneState(state.ids[ix], state.positions[ix], state.velocities[ix], state.desired_velocities[ix])
    for obs in PerfectObservation().observe(permuted, 0, 80):
        np.testing.assert_allclose(c(obs, 0), expected[obs.agent_id])


def test_head_on_simulation_completes_with_avoidance_and_v1_acceleration_limit():
    scenario = make_traffic_scenario('head_on', dt=.02)
    controller = TrafficController(scenario.goals)
    simulation = Simulation(scenario.initial, controller, scenario.config)
    simulation.run(scenario.duration)
    data = simulation.recorder.arrays()
    assert np.min(np.linalg.norm(data['positions'][:, 0]-data['positions'][:, 1], axis=1)) > 3
    assert np.max(np.linalg.norm(data['positions'][-1]-list(scenario.goals.values()), axis=1)) < P.arrival_radius
    assert np.max(np.linalg.norm(data['accelerations'], axis=2)) <= 6+1e-8
    assert any(d['iterations'] for d in controller.diagnostics)
    assert not any(d['fallback'] for d in controller.diagnostics)


@pytest.mark.parametrize('name', ['crossing_cruise', 'five_agents_cruise', 'close_parallel'])
def test_additional_completed_scenarios_and_component_activation(name):
    scenario = make_traffic_scenario(name, dt=.02)
    controller = TrafficController(scenario.goals, cruise_speeds=scenario.cruise_speeds)
    simulation = Simulation(scenario.initial, controller, scenario.config)
    simulation.run(scenario.duration)
    data = simulation.recorder.arrays()
    assert np.max(np.linalg.norm(data['positions'][-1]-list(scenario.goals.values()), axis=1)) < P.arrival_radius
    for i in range(len(scenario.goals)):
        for j in range(i):
            assert np.min(np.linalg.norm(data['positions'][:, i]-data['positions'][:, j], axis=1)) > 3
    if name == 'close_parallel':
        assert max(np.linalg.norm(d['repulsion']) for d in controller.diagnostics) > 1
        assert max(np.linalg.norm(d['friction']) for d in controller.diagnostics) > .5
    if name == 'five_agents_cruise':
        assert max(d['iterations'] for d in controller.diagnostics) >= 2


def test_real_multi_threat_cycle_is_bounded_and_explicitly_logged():
    # Regression for an observed failure: revisiting a candidate is NOT success.
    scenario = make_traffic_scenario('five_agents', dt=.02)
    controller = TrafficController(scenario.goals)
    simulation = Simulation(scenario.initial, controller, scenario.config)
    simulation.run(12)
    cycles = [d for d in controller.diagnostics if d['exit_reason'] == 'cycle']
    assert cycles
    for diagnostic in cycles:
        assert diagnostic['fallback'] and diagnostic['unresolved_threats']
        assert diagnostic['iterations'] <= P.max_iterations
        np.testing.assert_array_equal(diagnostic['self_drive'], [0, 0])
        after = [np.array(step['after']) for step in diagnostic['steps']]
        assert any(np.linalg.norm(after[-1]-old) <= P.tolerance for old in after[:-1])


def test_goal_direction_restoration_keeps_reduced_speed_and_is_rechecked():
    obs = observation([[0, 0], [6.2836, -17.7983], [-8.0364, 2.7939], [21.0506, -21.2247]],
                      [[-1.8381, 5.1352], [-6.4632, .4464], [-4.5064, -2.5837], [1.783, -3.8208]])
    goal = np.array([-16.0611, -.5043])
    u, trace = self_drive(obs, goal, P)
    assert trace['restored_goal_direction'] and not trace['fallback']
    previous = np.array(trace['steps'][-1]['after'])
    assert previous @ goal < 0
    assert np.linalg.norm(u) == pytest.approx(np.linalg.norm(previous))
    np.testing.assert_allclose(u/np.linalg.norm(u), goal/np.linalg.norm(goal))
    assert not any(d.threatening for d in inspect_neighbors(u, obs, trace['horizon_s'], P))


def test_goal_slowdown_can_create_new_threat_and_requires_second_solve():
    obs = observation([[0, 0], [-8.4664, 20.8346], [-10.9724, 5.1633], [-7.8989, 22.194]],
                      [[2.613, -7.2465], [3.6527, -4.8051], [.9449, 2.559], [3.2184, 4.5857]])
    goal = np.array([2.3211, 10.261])
    u, trace = self_drive(obs, goal, P)
    rechecks = [s for s in trace['steps'] if s['phase'] == 'goal_slowdown_recheck']
    assert rechecks and not trace['fallback']
    assert any(d.threatening for d in inspect_neighbors(np.array(rechecks[0]['before']), obs, trace['horizon_s'], P))
    assert not any(d.threatening for d in inspect_neighbors(u, obs, trace['horizon_s'], P))
    assert np.linalg.norm(u) <= braking_speed(np.linalg.norm(goal), 0, .5, 3)+1e-8
