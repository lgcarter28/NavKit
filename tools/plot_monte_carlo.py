# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def add_python_package_to_path() -> None:
    python_root = repo_root() / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))


def load_manifest(campaign_dir: Path) -> dict[str, Any]:
    manifest_path = campaign_dir / "campaign_manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing campaign manifest: {manifest_path}")
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def successful_run_dirs(campaign_dir: Path) -> list[Path]:
    manifest = load_manifest(campaign_dir)
    run_dirs: list[Path] = []
    runs = manifest.get("runs", [])
    if not isinstance(runs, list):
        raise ValueError("campaign manifest 'runs' field must be an array")
    for run in runs:
        if not isinstance(run, dict) or run.get("status") != "passed":
            continue
        run_dir = run.get("run_dir")
        if isinstance(run_dir, str):
            run_dirs.append(Path(run_dir))
    if not run_dirs:
        run_dirs = sorted((campaign_dir / "runs").glob("run_*"))
    return run_dirs


def main() -> int:
    show_requested = "--show" in sys.argv[1:]
    if not show_requested:
        os.environ.setdefault("MPLBACKEND", "Agg")

    add_python_package_to_path()

    from navkit_analysis.monte_carlo import (
        generate_monte_carlo_outputs,
        monte_carlo_plot_names,
    )
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(
        description="Regenerate selected NavKit Monte Carlo aggregate plots from existing runs."
    )
    parser.add_argument("campaign_dir", type=Path, help="Monte Carlo campaign output directory.")
    parser.add_argument(
        "--plot",
        action="append",
        choices=monte_carlo_plot_names(),
        help="Generate only the selected aggregate plot. May be provided multiple times.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Override the summary output directory. Defaults to <campaign>/summary.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=1000,
        help="Maximum points on the common aggregate plotting time grid.",
    )
    parser.add_argument("--start-time", type=float, default=None, help="Only plot samples at/after this time [s].")
    parser.add_argument("--end-time", type=float, default=None, help="Only plot samples at/before this time [s].")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open generated figures interactively after saving them.",
    )
    args = parser.parse_args()

    if not args.show:
        plt.rcParams["figure.max_open_warning"] = 0

    campaign_dir = args.campaign_dir
    summary_dir = args.output_dir or (campaign_dir / "summary")
    output = generate_monte_carlo_outputs(
        successful_run_dirs(campaign_dir),
        summary_dir,
        max_plot_points=args.max_plot_points,
        selected=args.plot,
        start_time_s=args.start_time,
        end_time_s=args.end_time,
    )
    if args.show:
        plt.show()
    else:
        for figure in output.figures:
            plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
