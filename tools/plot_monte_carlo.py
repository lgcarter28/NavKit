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
    parsed = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError(f"campaign manifest '{manifest_path}' must contain an object")
    from navkit_analysis.schema import MONTE_CARLO_CAMPAIGN_SCHEMA, validate_schema

    validate_schema(parsed, MONTE_CARLO_CAMPAIGN_SCHEMA, str(manifest_path))
    return parsed


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
        plot_monte_carlo_series,
        plot_monte_carlo_series_interactive,
    )
    from navkit_analysis.bundle import is_analysis_bundle, load_monte_carlo_series_from_bundle
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(
        description="Regenerate selected NavKit Monte Carlo aggregate plots from existing runs."
    )
    parser.add_argument(
        "campaign_source",
        type=Path,
        help="Monte Carlo campaign directory or a packaged HDF5 campaign bundle.",
    )
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
    parser.add_argument(
        "--renderer",
        choices=["matplotlib", "plotly"],
        default="matplotlib",
        help="Use static Matplotlib PNGs or interactive Plotly HTML output.",
    )
    args = parser.parse_args()

    if not args.show:
        plt.rcParams["figure.max_open_warning"] = 0

    campaign_source = args.campaign_source
    if is_analysis_bundle(campaign_source):
        output_dir = args.output_dir or (
            campaign_source.parent / ("interactive_figures" if args.renderer == "plotly" else "figures")
        )
        selected = set(args.plot or monte_carlo_plot_names())
        for series in load_monte_carlo_series_from_bundle(campaign_source, selected):
            if args.renderer == "matplotlib":
                plot_monte_carlo_series(series, output_dir)
                continue
            output_path = output_dir / series.output_name.removesuffix(".png").replace(
                "monte_carlo_", ""
            )
            output_path = output_path.with_suffix(".html")
            figure = plot_monte_carlo_series_interactive(series, output_path)
            if args.show:
                figure.show()
        return 0

    if args.renderer == "plotly":
        output_dir = args.output_dir or (campaign_source / "summary" / "interactive_figures")
        selected = set(args.plot or monte_carlo_plot_names())
        from navkit_analysis.monte_carlo import aggregate_monte_carlo_series, load_successful_runs

        series_items = aggregate_monte_carlo_series(
            load_successful_runs(successful_run_dirs(campaign_source)),
            max_plot_points=args.max_plot_points,
            selected=list(selected),
            start_time_s=args.start_time,
            end_time_s=args.end_time,
        )
        for series in series_items:
            output_path = output_dir / series.output_name.removesuffix(".png").replace(
                "monte_carlo_", ""
            )
            figure = plot_monte_carlo_series_interactive(series, output_path.with_suffix(".html"))
            if args.show:
                figure.show()
        return 0

    summary_dir = args.output_dir or (campaign_source / "summary")
    output = generate_monte_carlo_outputs(
        successful_run_dirs(campaign_source),
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
