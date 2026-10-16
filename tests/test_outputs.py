import csv
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np


def test_cli_writes_readable_configs_data_and_images(tmp_path):
    env = {**os.environ, "MPLCONFIGDIR": str(tmp_path / "mpl")}
    subprocess.run(
        [sys.executable, "-m", "experiments.run_basic", "--scenario", "single",
         "--output", str(tmp_path / "output")],
        cwd=Path(__file__).resolve().parents[1], env=env, check=True,
        capture_output=True, text=True,
    )
    result = tmp_path / "output" / "single"
    for filename in ("trajectories.png", "speed.png", "neighbors.png"):
        assert (result / filename).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    config = json.loads((result / "config.json").read_text())
    summary = json.loads((result / "summary.json").read_text())
    assert config["runtime"]["executable"] == sys.executable
    assert all(summary["checks"].values())
    with np.load(result / "trajectory.npz", allow_pickle=False) as data:
        assert data["positions"].shape == (1001, 1, 2)
        assert data["accelerations"].shape == (1000, 1, 2)
        with (result / "trajectory.csv").open() as file:
            rows = list(csv.DictReader(file))
        assert len(rows) == 1001
        assert float(rows[-1]["time_s"]) == 10.0
        assert float(rows[-1]["x_m"]) == data["positions"][-1, 0, 0]
        with (result / "speed.csv").open() as file:
            speeds = list(csv.DictReader(file))
        np.testing.assert_allclose(
            [float(row["speed_m_s"]) for row in speeds],
            np.linalg.norm(data["velocities"][:, 0], axis=1),
        )
