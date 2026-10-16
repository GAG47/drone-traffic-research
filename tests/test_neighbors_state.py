import numpy as np
import pytest

from simulator import DroneState
from simulator.neighbors import PerfectObservation, neighbors_within


def test_ids_states_boundary_and_distances():
    state = DroneState(
        [100, 9, 42, 3], [[0, 0], [3, 4], [0, 0], [5.000001, 0]],
        [[1, 2], [3, 4], [5, 6], [7, 8]], np.zeros((4, 2)),
    )
    groups = neighbors_within(state, 5)
    np.testing.assert_array_equal(groups[0].ids, [9, 42])
    np.testing.assert_array_equal(groups[0].positions, [[3, 4], [0, 0]])
    np.testing.assert_array_equal(groups[0].velocities, [[3, 4], [5, 6]])
    np.testing.assert_allclose(groups[0].distances, [5, 0])
    for i, group in enumerate(groups):
        assert state.ids[i] not in group.ids
        for agent_id, distance in zip(group.ids, group.distances):
            j = np.flatnonzero(state.ids == agent_id)[0]
            expected = ((state.positions[i, 0] - state.positions[j, 0]) ** 2
                        + (state.positions[i, 1] - state.positions[j, 1]) ** 2) ** 0.5
            assert distance == pytest.approx(expected)
            assert state.ids[i] in groups[j].ids


def test_zero_radius_keeps_distinct_colocated_agent():
    state = DroneState.at_rest([4, 8, 12], [[1, 1], [1, 1], [2, 1]])
    groups = neighbors_within(state, 0)
    assert [g.ids.tolist() for g in groups] == [[8], [4], []]


def test_single_agent_has_empty_neighbors():
    group, = neighbors_within(DroneState.at_rest([2], [[0, 0]]), 100)
    assert group.positions.shape == (0, 2)
    assert group.ids.size == group.distances.size == 0


def test_initial_data_and_observations_do_not_alias():
    positions = np.array([[0.0, 0.0], [1.0, 0.0]])
    state = DroneState.at_rest([1, 2], positions)
    positions[0, 0] = 999
    np.testing.assert_array_equal(state.positions[0], [0, 0])
    with pytest.raises(ValueError):
        state.positions[0, 0] = 2
    observations = PerfectObservation().observe(state, 0, 10)
    for array in (observations[0].position, observations[0].neighbors.positions):
        assert not array.flags.writeable
        assert not np.shares_memory(array, state.positions)


@pytest.mark.parametrize("radius", [-1, float("nan"), float("inf")])
def test_invalid_radius(radius):
    with pytest.raises(ValueError):
        neighbors_within(DroneState.at_rest([1], [[0, 0]]), radius)


@pytest.mark.parametrize("ids,positions", [
    ([1, 1], [[0, 0], [1, 1]]),
    ([1.5], [[0, 0]]),
    ([], []),
    ([1], [[float("nan"), 0]]),
    ([1], [[0, 0, 0]]),
])
def test_invalid_state(ids, positions):
    with pytest.raises(ValueError):
        DroneState.at_rest(ids, positions)
