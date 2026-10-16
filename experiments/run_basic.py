"""Run with python -m experiments.run_basic --scenario all."""

import argparse
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import platform
import sys

import matplotlib

matplotlib.use("Agg")  # Headless WSL; no display server required.
import matplotlib.pyplot as plt
import numpy as np

from scenarios.basic_scenarios import SCENARIO_NAMES, Scenario, make_scenario
from simulator import Simulation


def plot_results(data: dict, scenario: Scenario, directory: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 5.2), layout="constrained")
    for i, agent_id in enumerate(data["ids"]):
        points = data["positions"][:, i]
        line, = ax.plot(points[:, 0], points[:, 1], label=f"Drone {agent_id}", alpha=0.85)
        ax.scatter(*points[0], marker="o", color=line.get_color(), s=36)
        ax.scatter(*points[-1], marker="x", color=line.get_color(), s=65)
    ax.set(xlabel="x [m]", ylabel="y [m]", title=f"{scenario.name}: trajectories")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.25)
    ax.legend(title="Start: circle; end: cross", fontsize=8)
    fig.savefig(directory / "trajectories.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 6.2), sharex=True, layout="constrained")
    for i, agent_id in enumerate(data["ids"]):
        line, = axes[0].plot(
            data["times"], np.linalg.norm(data["velocities"][:, i], axis=1),
            label=f"Drone {agent_id}",
        )
        axes[0].axhline(
            np.linalg.norm(scenario.commands[i]), color=line.get_color(),
            linestyle=":", alpha=0.55,
        )
        axes[1].plot(
            data["times"][:-1], np.linalg.norm(data["accelerations"][:, i], axis=1),
            color=line.get_color(),
        )
    axes[0].set(ylabel="Speed [m/s]", title=f"{scenario.name}: velocity tracking")
    axes[0].legend(fontsize=8)
    axes[1].axhline(
        scenario.config.dynamics.max_acceleration, color="black", linestyle="--", label="a_max"
    )
    axes[1].set(xlabel="Time [s]", ylabel="Acceleration [m/s²]")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.savefig(directory / "speed.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 6.2), sharex=True, layout="constrained")
    if len(data["ids"]) > 1:
        for i in range(len(data["ids"])):
            for j in range(i):
                axes[0].plot(
                    data["times"],
                    np.linalg.norm(data["positions"][:, i] - data["positions"][:, j], axis=1),
                    label=f'{data["ids"][j]}–{data["ids"][i]}',
                )
        axes[0].axhline(scenario.config.communication_radius, color="black", linestyle=":")
        axes[0].legend(ncol=3, fontsize=7)
    else:
        axes[0].text(0.5, 0.5, "No pairs (one drone)", transform=axes[0].transAxes, ha="center")
    axes[0].set(ylabel="Pair distance [m]", title=f"{scenario.name}: local neighborhoods")
    for i, agent_id in enumerate(data["ids"]):
        axes[1].step(data["times"], data["neighbor_counts"][:, i], where="post", label=str(agent_id))
    axes[1].set(xlabel="Time [s]", ylabel="Neighbor count")
    axes[1].set_yticks(range(len(data["ids"])))
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.savefig(directory / "neighbors.png", dpi=160)
    plt.close(fig)


def summarize(data: dict, scenario: Scenario) -> dict:
    residual = np.diff(data["positions"], axis=0) - scenario.config.dt * data["velocities"][:-1]
    acceleration = np.linalg.norm(data["accelerations"], axis=2)
    finite = all(np.isfinite(value).all() for value in data.values())
    pair_minimum = None
    if len(data["ids"]) > 1:
        pair_minimum = min(
            float(np.linalg.norm(data["positions"][:, i] - data["positions"][:, j], axis=1).min())
            for i in range(len(data["ids"])) for j in range(i)
        )
    errors = np.linalg.norm(data["velocities"][-1] - scenario.commands, axis=1)
    # Fail visibly if the foundation invariants are violated.
    checks = {
        "all_values_finite": bool(finite),
        "acceleration_bound": bool(acceleration.max() <= scenario.config.dynamics.max_acceleration + 1e-10),
        "old_velocity_position_integration": bool(np.max(np.abs(residual)) < 1e-10),
        "final_velocity_error_below_0_001_m_s": bool(errors.max() < 0.001),
    }
    if not all(checks.values()):
        raise RuntimeError(f"experiment validation failed: {checks}")
    return {
        "scenario": scenario.name, "scope": "foundation_only_no_collision_avoidance",
        "agent_count": len(data["ids"]), "steps": len(data["times"]) - 1,
        "state_samples": len(data["times"]), "duration_s": float(data["times"][-1]),
        "max_acceleration_m_s2": float(acceleration.max()),
        "max_position_integration_residual_m": float(np.max(np.abs(residual))),
        "final_positions_m": data["positions"][-1].tolist(),
        "final_velocities_m_s": data["velocities"][-1].tolist(),
        "final_velocity_errors_m_s": errors.tolist(),
        "minimum_sampled_pair_distance_m": pair_minimum,
        "initial_neighbor_counts": data["neighbor_counts"][0].tolist(),
        "final_neighbor_counts": data["neighbor_counts"][-1].tolist(),
        "checks": checks,
    }


def run_experiment(name: str, output: Path, dt: float = 0.01) -> dict:
    scenario = make_scenario(name, dt)
    sim = Simulation(scenario.initial, scenario.controller(), scenario.config)
    sim.run(scenario.duration)
    data = sim.recorder.arrays()
    summary = summarize(data, scenario)
    directory = output / name
    sim.recorder.save(directory)
    config = {
        "scenario": name, "description": scenario.description,
        "scope": "RobotSim foundation migration, not the paper traffic algorithm",
        "units": "m, s, m/s, m/s^2", "dimensions": 2,
        "duration_s": scenario.duration, "simulation": asdict(scenario.config),
        "initial_state": {
            key: getattr(scenario.initial, key).tolist()
            for key in ("ids", "positions", "velocities", "desired_velocities")
        },
        "controller": {"name": "FixedVelocityController", "velocities_m_s": scenario.commands.tolist()},
        "sensor_noise": False, "outer_noise": False, "communication_delay_s": 0.0,
        "packet_loss": False, "randomness": "none",
        "reference_commit": "ea54b97e7e2659449e721ebd4a2285beb9b0e9ce",
        "runtime": {
            "python": platform.python_version(), "executable": sys.executable,
            **{pkg: importlib.metadata.version(pkg) for pkg in ("numpy", "matplotlib")},
        },
    }
    for filename, content in (("config.json", config), ("summary.json", summary)):
        (directory / filename).write_text(
            json.dumps(content, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    plot_results(data, scenario, directory)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*SCENARIO_NAMES, "all"), default="all")
    parser.add_argument("--output", type=Path, default=Path("results/basic"))
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()
    names = SCENARIO_NAMES if args.scenario == "all" else (args.scenario,)
    for name in names:
        summary = run_experiment(name, args.output, args.dt)
        print(
            f"{name}: {summary['steps']} steps, checks passed; "
            f"max acceleration={summary['max_acceleration_m_s2']:.6g} m/s²; "
            f"output={args.output / name}"
        )


if __name__ == "__main__":
    main()
