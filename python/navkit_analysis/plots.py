# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Support both:
#   python -m navkit_analysis.plots ...
# and:
#   python python/navkit_analysis/plots.py ...
#
# When this file is executed directly, Python puts python/navkit_analysis on
# sys.path instead of python/. Add python/ explicitly so absolute package imports
# work consistently.
if __package__ is None or __package__ == "":
    python_root = Path(__file__).resolve().parents[1]
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))

import matplotlib.pyplot as plt

from navkit_analysis.sources import load_analysis_run
from navkit_analysis.figures import (
    plot_filter_corrections,
    plot_gnss_position_histograms,
    plot_gnss_position_debug,
    plot_gnss_position_innovation,
    plot_gnss_position_nis,
    plot_gnss_velocity_debug,
    plot_gnss_velocity_innovation,
    plot_gnss_velocity_nis,
    plot_imu_bias_truth_errors,
    plot_imu_debug_terms,
    plot_imu_increment_cumsums,
    plot_imu_increment_time_histories,
    plot_position_error_covariance,
    plot_truth_errors,
    plot_truth_errors_ecef,
    plot_truth_errors_ned,
    plot_truth_errors_ned_dashboard,
)
from navkit_analysis.style import apply_style


def _plot_plan() -> dict[str, tuple]:
    return {
        "position_ecef_legacy": (plot_position_error_covariance,),
        "gnss_position": (
            plot_gnss_position_innovation,
            plot_gnss_position_nis,
            plot_gnss_position_histograms,
            plot_gnss_position_debug,
        ),
        "gnss_velocity": (
            plot_gnss_velocity_debug,
            plot_gnss_velocity_innovation,
            plot_gnss_velocity_nis,
        ),
        "filter_correction": (plot_filter_corrections,),
        "dashboard_ecef": (plot_truth_errors,),
        "dashboard_ned": (plot_truth_errors_ned_dashboard,),
        "imu_increments": (plot_imu_increment_time_histories, plot_imu_increment_cumsums),
        "imu_debug": (plot_imu_debug_terms,),
        "imu_bias": (plot_imu_bias_truth_errors,),
        "state_ecef": (plot_truth_errors_ecef,),
        "state_ned": (plot_truth_errors_ned,),
    }


def _selected_plot_names(selected: list[str] | None) -> set[str]:
    plan = _plot_plan()
    if not selected:
        return set(plan)
    unknown = sorted(set(selected) - set(plan))
    if unknown:
        raise ValueError(f"unknown plot name(s): {unknown}; choices are {sorted(plan)}")
    return set(selected)


def plot_run(
    source: Path,
    save: bool = True,
    selected: list[str] | None = None,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
    run_id: str | None = None,
) -> list[plt.Figure]:
    """Create all standard figures from one CSV run directory or HDF5 bundle.

    Figure functions save outputs and return open figure objects. This function
    intentionally does not call ``plt.show()``; the caller owns figure lifetime.
    """
    apply_style()
    run_data = load_analysis_run(
        source,
        run_id=run_id,
        start_time_s=start_time_s,
        end_time_s=end_time_s,
    )
    selected_names = _selected_plot_names(selected)

    figures: list[plt.Figure] = []
    for plot_name, functions in _plot_plan().items():
        if plot_name not in selected_names:
            continue
        for function in functions:
            result = function(run_data, save=save)
            if isinstance(result, list):
                figures.extend(result)
            elif result is not None:
                figures.append(result)

    return figures


def close_figures(figures: list[plt.Figure]) -> None:
    """Close all generated figures."""
    for fig in figures:
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate NavKit analysis plots.")
    parser.add_argument("source", type=Path, help="Run CSV directory or an HDF5 analysis bundle.")
    parser.add_argument(
        "--run-id",
        default=None,
        help="Run identifier when the source is a multi-run HDF5 campaign bundle.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open all figures interactively after generating/saving them.",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Create figures without saving PNG files.",
    )
    parser.add_argument(
        "--plot",
        action="append",
        choices=sorted(_plot_plan()),
        help="Generate only the selected plot group. May be provided multiple times.",
    )
    parser.add_argument("--start-time", type=float, default=None, help="Only plot data at/after this time [s].")
    parser.add_argument("--end-time", type=float, default=None, help="Only plot data at/before this time [s].")

    args = parser.parse_args(argv)

    if not args.show:
        plt.rcParams["figure.max_open_warning"] = 0

    figures = plot_run(
        args.source,
        save=not args.no_save,
        selected=args.plot,
        start_time_s=args.start_time,
        end_time_s=args.end_time,
        run_id=args.run_id,
    )

    if args.show:
        # Call show once after all figures have been created so the user can
        # toggle between windows instead of closing one plot at a time.
        plt.show()
    else:
        close_figures(figures)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
