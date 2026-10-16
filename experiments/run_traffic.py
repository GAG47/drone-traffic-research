"""Run, diagnose and replay the deterministic V2 traffic experiments."""

import argparse
from collections import Counter
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from time import perf_counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

from algorithms.parameters import TrafficParameters
from controllers.traffic import TrafficController
from scenarios.traffic_scenarios import SCENARIO_NAMES, make_traffic_scenario
from simulator.engine import Simulation


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def pair_metrics(positions, dt, threshold):
    """Exact within-step distances for V1's piecewise linear position path.

    Count connected open intervals d<threshold and integrate their duration.
    No physical interpolation claim for a different, continuous-time plant.
    """
    pairs = []
    for i in range(positions.shape[1]):
        for j in range(i):
            relative = positions[:, i]-positions[:, j]
            q, delta = relative[:-1], np.diff(relative, axis=0)
            aa = np.sum(delta**2, axis=1)
            bb = np.sum(q*delta, axis=1)
            cc = np.sum(q**2, axis=1)-threshold**2
            fraction = np.clip(np.divide(-bb, aa, out=np.zeros_like(bb), where=aa>0), 0, 1)
            minimum = float(np.min(np.linalg.norm(q+fraction[:, None]*delta, axis=1)))
            discriminant = bb**2-aa*cc
            moving = aa > 1e-24
            inside = moving & (discriminant > 0)
            root = np.sqrt(np.maximum(discriminant, 0))
            low = np.clip(np.divide(-bb-root, aa, out=np.zeros_like(bb), where=moving), 0, 1)
            high = np.clip(np.divide(-bb+root, aa, out=np.zeros_like(bb), where=moving), 0, 1)
            stationary_inside = ~moving & (cc < 0)
            low = np.where(stationary_inside, 0, low)
            high = np.where(stationary_inside, 1, high)
            occupied = (inside & (high > low)) | stationary_inside
            duration = float(np.sum(np.where(occupied, high-low, 0))*dt)
            events, previous_end = 0, -np.inf
            for k in np.flatnonzero(occupied):
                start, end = (k+low[k])*dt, (k+high[k])*dt
                if start > previous_end+1e-10:
                    events += 1
                previous_end = end
            pairs.append({"rows": [j, i], "minimum_separation_m": minimum,
                          "sampled_minimum_m": float(np.min(np.linalg.norm(relative, axis=1))),
                          "violation_seconds": duration, "violation_events": events})
    return pairs


def summarize(data, goals, diagnostics, parameters, dt, wall_time, danger_distance=3.0):
    positions, velocities, times = data['positions'], data['velocities'], data['times']
    target_array = np.array([goals[int(i)] for i in data['ids']])
    errors = np.linalg.norm(positions-target_array[None], axis=2)
    speeds = np.linalg.norm(velocities, axis=2)
    pairs = pair_metrics(positions, dt, danger_distance)
    agents = []
    last_window = times >= max(0, times[-1]-10)
    for row, agent_id in enumerate(data['ids']):
        reached = (errors[:, row] <= parameters.arrival_radius) & (speeds[:, row] <= parameters.arrival_speed)
        hits = np.flatnonzero(reached)
        # Terminal settling time: all later samples also meet the arrival test.
        not_reached = np.flatnonzero(~reached)
        settled = int(not_reached[-1]+1) if len(not_reached) else 0
        completed = bool(reached[-1])
        progress = float(errors[np.flatnonzero(last_window)[0], row]-errors[-1, row])
        waiting = not completed and float(np.max(speeds[last_window, row])) < .2 and abs(progress) < .5
        stalled = not completed and abs(progress) < .5
        agents.append({"id": int(agent_id), "completed": completed,
                       "first_arrival_s": float(times[hits[0]]) if len(hits) else None,
                       "settled_arrival_s": float(times[settled]) if settled < len(times) else None,
                       "final_goal_error_m": float(errors[-1, row]),
                       "final_speed_m_s": float(speeds[-1, row]),
                       "path_length_m": float(np.sum(np.linalg.norm(np.diff(positions[:, row], axis=0), axis=1))),
                       "straight_line_m": float(errors[0, row]),
                       "last_10s_goal_progress_m": progress, "suspected_deadlock": waiting,
                       "suspected_livelock": stalled and not waiting,
                       "status": "completed" if completed else "deadlock_candidate" if waiting
                       else "livelock_candidate" if stalled else "incomplete"})
    compute = np.array([d['compute_seconds'] for d in diagnostics])
    components = {key: float(max(np.linalg.norm(d[key]) for d in diagnostics))
                  for key in ('self_drive', 'repulsion', 'friction', 'desired')}
    side_changes = 0
    previous_side = {}
    for d in diagnostics:
        for step in d['steps']:
            key = (d['agent_id'], step['selected_id'])
            if key in previous_side and previous_side[key] != step['passing_side']:
                side_changes += 1
            previous_side[key] = step['passing_side']
    return {
        "duration_s": float(times[-1]), "danger_distance_m": danger_distance,
        "minimum_separation_m": min((p['minimum_separation_m'] for p in pairs), default=None),
        "danger_pair_seconds": sum(p['violation_seconds'] for p in pairs),
        "danger_events": sum(p['violation_events'] for p in pairs),
        "collision_risk_eq7": sum(p['violation_seconds'] for p in pairs)/(len(pairs)*times[-1]) if pairs else 0.,
        "pair_metrics": pairs, "agents": agents,
        "completed_agents": sum(a['completed'] for a in agents),
        "all_completed": all(a['completed'] for a in agents),
        "suspected_deadlock": any(a['suspected_deadlock'] for a in agents),
        "suspected_livelock": any(a['suspected_livelock'] for a in agents),
        "max_actual_speed_m_s": float(np.max(speeds)),
        "max_acceleration_m_s2": float(np.max(np.linalg.norm(data['accelerations'], axis=2))),
        "finite": bool(np.isfinite(positions).all() and np.isfinite(velocities).all()),
        "control_calls": len(diagnostics), "control_mean_ms": float(np.mean(compute)*1000),
        "control_p95_ms": float(np.quantile(compute, .95)*1000),
        "control_total_s": float(np.sum(compute)), "simulation_wall_s": wall_time,
        "iterations_total": sum(d['iterations'] for d in diagnostics),
        "max_iterations": max(d['iterations'] for d in diagnostics),
        "multi_threat_calls": sum(any(len(s['threats']) > 1 for s in d['steps']) for d in diagnostics),
        "fallback_calls": sum(d['fallback'] for d in diagnostics),
        "exit_reasons": dict(Counter(d['exit_reason'] for d in diagnostics)),
        "passing_side_changes_across_all_steps": side_changes,
        "max_component_speed_m_s": components,
    }


def plot_results(data, goals, directory, title, danger_distance=3.0, avoidance_radius=13.0):
    times, positions = data['times'], data['positions']
    fig, ax = plt.subplots(figsize=(7, 6))
    for row, agent_id in enumerate(data['ids']):
        path = positions[:, row]
        line, = ax.plot(path[:, 0], path[:, 1], label=f'ID {agent_id}')
        ax.scatter(*path[0], marker='o', color=line.get_color())
        ax.scatter(*path[-1], marker='s', color=line.get_color())
        ax.scatter(*goals[int(agent_id)], marker='x', s=80, color=line.get_color())
    ax.set(xlabel='x [m]', ylabel='y [m]', title=title+'\nstart: circle, end: square, goal: cross', aspect='equal')
    ax.grid(alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(directory/'trajectories.png', dpi=140); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4))
    for row, agent_id in enumerate(data['ids']):
        ax.plot(times, np.linalg.norm(data['velocities'][:, row], axis=1), label=f'ID {agent_id}')
    ax.set(xlabel='Time [s]', ylabel='Actual speed [m/s]', title=title)
    ax.grid(alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(directory/'speed.png', dpi=140); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4))
    for i in range(len(data['ids'])):
        for j in range(i):
            ax.plot(times, np.linalg.norm(positions[:, i]-positions[:, j], axis=1),
                    label=f'{data["ids"][j]} / {data["ids"][i]}')
    if len(data['ids']) > 1:
        ax.axhline(danger_distance, ls='--', color='red', label='danger 3 m')
        ax.axhline(avoidance_radius, ls=':', color='gray', label=f'avoidance {avoidance_radius:g} m')
        ax.legend(ncol=2, fontsize=8)
    else:
        ax.text(.5, .5, 'Single agent: no pairs', transform=ax.transAxes, ha='center')
    ax.set(xlabel='Time [s]', ylabel='Pair separation [m]', title=title)
    ax.grid(alpha=.3); fig.tight_layout()
    fig.savefig(directory/'distances.png', dpi=140); plt.close(fig)


def animate(data, goals, path, title):
    """Replay saved states at 5x simulated speed; no algorithm recomputation."""
    positions, times = data['positions'], data['times']
    target = np.array([goals[int(i)] for i in data['ids']])
    cloud = np.concatenate([positions.reshape(-1, 2), target])
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set(xlim=(cloud[:, 0].min()-10, cloud[:, 0].max()+10),
           ylim=(cloud[:, 1].min()-10, cloud[:, 1].max()+10), aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax.grid(alpha=.25)
    trails, markers = [], []
    for row, agent_id in enumerate(data['ids']):
        line, = ax.plot([], [], lw=1, label=f'ID {agent_id}')
        marker, = ax.plot([], [], 'o', color=line.get_color())
        ax.scatter(*target[row], marker='x', color=line.get_color())
        trails.append(line); markers.append(marker)
    ax.legend(loc='upper right')
    frame_times = np.arange(0, times[-1], .5)
    frames = np.unique(np.r_[np.searchsorted(times, frame_times), len(times)-1])
    def update(k):
        for row in range(len(markers)):
            trails[row].set_data(positions[:k+1, row, 0], positions[:k+1, row, 1])
            markers[row].set_data([positions[k, row, 0]], [positions[k, row, 1]])
        ax.set_title(f'{title}\nt = {times[k]:.1f} s (5x replay)')
        return trails+markers
    movie = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)
    movie.save(path, writer=PillowWriter(fps=10))
    plt.close(fig)


def run_experiment(name, mode, output, parameters=None, dt=.01, gif=False):
    parameters = parameters or TrafficParameters()
    scenario = make_traffic_scenario(name, dt=dt)
    directory = Path(output)/name/mode
    directory.mkdir(parents=True, exist_ok=True)
    controller = TrafficController(scenario.goals, parameters, mode, scenario.cruise_speeds)
    simulation = Simulation(scenario.initial, controller, scenario.config)
    start = perf_counter()
    simulation.run(scenario.duration)
    wall_time = perf_counter()-start
    data = simulation.recorder.arrays()
    simulation.recorder.save(directory)
    summary = summarize(data, scenario.goals, controller.diagnostics, parameters, dt, wall_time)
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for folder in ('algorithms', 'controllers', 'scenarios', 'simulator', 'experiments')
                     for path in sorted((ROOT/folder).glob('*.py'))}
    config = {"scenario": name, "description": scenario.description, "mode": mode, "units": "SI",
              "duration_s": scenario.duration, "simulation": asdict(scenario.config),
              "traffic_parameters": asdict(parameters),
              "initial": {"ids": scenario.initial.ids.tolist(), "positions": scenario.initial.positions.tolist(),
                          "velocities": scenario.initial.velocities.tolist()},
              "goals": {str(i): g.tolist() for i, g in scenario.goals.items()},
              "cruise_speeds": scenario.cruise_speeds,
              "assumptions": {"observation": "perfect, no noise or delay", "queueing": False,
                              "horizon": "fixed goal distance / cruise speed within call",
                              "friction_amplitude": "gain * max(relative_speed - allowed, 0)"},
              "source_revision_at_run": revision, "source_sha256": source_hashes,
              "runtime": {"python": platform.python_version(), "executable": sys.executable,
                          "numpy": np.__version__, "matplotlib": matplotlib.__version__}}
    write_json(directory/'config.json', config)
    write_json(directory/'summary.json', summary)
    with (directory/'diagnostics.jsonl').open('w', encoding='utf-8') as file:
        for diagnostic in controller.diagnostics:
            file.write(json.dumps(diagnostic, allow_nan=False)+'\n')
    plot_results(data, scenario.goals, directory, f'{name} / {mode}', avoidance_radius=parameters.avoidance_radius)
    if gif:
        animate(data, scenario.goals, directory/'replay.gif', f'{name} / {mode}')
    print(f'{name}/{mode}: completed={summary["completed_agents"]}/{len(scenario.goals)} '
          f'min_distance={summary["minimum_separation_m"]} fallback={summary["fallback_calls"]}', flush=True)
    return {"scenario": name, "mode": mode, **summary}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('all',)+SCENARIO_NAMES, default='all')
    parser.add_argument('--mode', choices=('all',)+TrafficController.MODES, default='all')
    parser.add_argument('--output', type=Path, default=Path('results/traffic'))
    parser.add_argument('--dt', type=float, default=.01)
    parser.add_argument('--anisotropy', type=float, default=.5)
    parser.add_argument('--friction-gain', type=float, default=1.)
    parser.add_argument('--gif', action='store_true', help='animate full head-on and crossing runs, including cruise start')
    parser.add_argument('--replay', type=Path, help='existing run directory; render replay.gif only')
    args = parser.parse_args()
    if args.replay:
        config = json.loads((args.replay/'config.json').read_text())
        with np.load(args.replay/'trajectory.npz', allow_pickle=False) as archive:
            data = {name: archive[name] for name in archive.files}
        animate(data, {int(i): np.array(g) for i, g in config['goals'].items()},
                args.replay/'replay.gif', f'{config["scenario"]} / {config["mode"]}')
        return
    parameters = replace(TrafficParameters(), anisotropy=args.anisotropy, friction_gain=args.friction_gain)
    names = SCENARIO_NAMES if args.scenario == 'all' else (args.scenario,)
    modes = TrafficController.MODES if args.mode == 'all' else (args.mode,)
    results = [run_experiment(name, mode, args.output, parameters, args.dt,
                              args.gif and name in ('head_on', 'crossing', 'crossing_cruise') and mode == 'full')
               for name in names for mode in modes]
    write_json(args.output/'summary.json', results)
    with (args.output/'summary.csv').open('w', newline='', encoding='utf-8') as file:
        fields = ['scenario', 'mode', 'completed_agents', 'minimum_separation_m', 'danger_events',
                  'danger_pair_seconds', 'suspected_deadlock', 'suspected_livelock', 'max_iterations', 'fallback_calls',
                  'control_mean_ms', 'control_p95_ms']
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(results)


if __name__ == '__main__':
    main()
