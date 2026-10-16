import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from experiments.run_traffic import pair_metrics, summarize
from algorithms.parameters import TrafficParameters


def test_between_sample_collision_is_not_missed():
    # Endpoints are 10 m apart; a crossing between samples has exact distance 0.
    positions = np.array([[[-5, 0], [5, 0]], [[5, 0], [-5, 0]]], float)
    pair, = pair_metrics(positions, 1, 3)
    assert pair['minimum_separation_m'] == pytest.approx(0)
    assert pair['sampled_minimum_m'] == 10
    assert pair['violation_seconds'] == pytest.approx(.3)
    assert pair['violation_events'] == 1


def test_violation_duration_and_connected_events_with_stationary_intervals():
    positions = np.zeros((5, 2, 2))
    positions[:, 1, 0] = [5, 1, 1, 5, 1]
    pair, = pair_metrics(positions, 1, 3)
    assert pair['minimum_separation_m'] == 1
    assert pair['violation_seconds'] == pytest.approx(2.5)
    assert pair['violation_events'] == 2
    positions[:, 1, 0] = 3  # Equality is not a danger event.
    pair, = pair_metrics(positions, 1, 3)
    assert pair['violation_seconds'] == pair['violation_events'] == 0


def test_moving_without_goal_progress_is_not_reported_as_success_or_stationary_deadlock():
    times = np.arange(21.)
    angle = times/10
    positions = 10*np.column_stack((np.cos(angle), np.sin(angle)))[:, None, :]
    velocities = np.column_stack((-np.sin(angle), np.cos(angle)))[:, None, :]
    data = {'times': times, 'positions': positions, 'velocities': velocities, 'ids': np.array([10]),
            'accelerations': np.zeros((20, 1, 2))}
    diagnostic = {'compute_seconds': .001, 'self_drive': [1, 0], 'repulsion': [0, 0],
                  'friction': [0, 0], 'desired': [1, 0], 'steps': [], 'iterations': 0,
                  'fallback': False, 'exit_reason': 'no_threat'}
    result = summarize(data, {10: np.zeros(2)}, [diagnostic], TrafficParameters(), 1, .01)
    assert not result['all_completed'] and not result['suspected_deadlock']
    assert result['suspected_livelock']
    assert result['agents'][0]['status'] == 'livelock_candidate'


def test_traffic_cli_outputs_and_replay(tmp_path):
    env = {**os.environ, 'MPLCONFIGDIR': str(tmp_path/'mpl')}
    root = Path(__file__).resolve().parents[1]
    command = [sys.executable, '-m', 'experiments.run_traffic', '--scenario', 'single',
               '--mode', 'full', '--output', str(tmp_path/'runs'), '--dt', '.02']
    subprocess.run(command, cwd=root, env=env, check=True, capture_output=True, text=True)
    run = tmp_path/'runs'/'single'/'full'
    summary = json.loads((run/'summary.json').read_text())
    config = json.loads((run/'config.json').read_text())
    assert summary['all_completed'] and summary['finite']
    assert summary['minimum_separation_m'] is None
    assert summary['danger_events'] == 0
    assert summary['agents'][0]['settled_arrival_s'] < 30
    assert config['traffic_parameters']['anisotropy'] == .5
    assert config['source_sha256']['algorithms/self_drive.py']
    for name in ('trajectories.png', 'speed.png', 'distances.png'):
        assert (run/name).read_bytes().startswith(b'\x89PNG')
    diagnostics = [json.loads(line) for line in (run/'diagnostics.jsonl').read_text().splitlines()]
    assert len(diagnostics) == 150
    with np.load(run/'trajectory.npz', allow_pickle=False) as data:
        assert data['positions'].shape == (1501, 1, 2)
        np.testing.assert_allclose([d['desired'] for d in diagnostics],
                                   data['applied_desired_velocities'][::10, 0])
    subprocess.run([sys.executable, '-m', 'experiments.run_traffic', '--replay', str(run)],
                   cwd=root, env=env, check=True, capture_output=True, text=True)
    assert (run/'replay.gif').read_bytes().startswith(b'GIF8')
