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

from navkit_analysis.data import load_run
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


def plot_run(run_dir: Path, save: bool = True) -> list[plt.Figure]:
    """Create all standard figures for one NavKit run.

    Figure functions save outputs and return open figure objects. This function
    intentionally does not call ``plt.show()``; the caller owns figure lifetime.
    """
    apply_style()
    run_data = load_run(run_dir)

    maybe_figures = [
        plot_position_error_covariance(run_data, save=save),
        plot_gnss_position_innovation(run_data, save=save),
        plot_gnss_position_nis(run_data, save=save),
        plot_gnss_position_histograms(run_data, save=save),
        plot_gnss_position_debug(run_data, save=save),
        plot_gnss_velocity_debug(run_data, save=save),
        plot_gnss_velocity_innovation(run_data, save=save),
        plot_gnss_velocity_nis(run_data, save=save),
        plot_filter_corrections(run_data, save=save),
        plot_truth_errors(run_data, save=save),
        plot_truth_errors_ned_dashboard(run_data, save=save),
    ]

    figures = [fig for fig in maybe_figures if fig is not None]
    figures.extend(plot_imu_increment_time_histories(run_data, save=save))
    figures.extend(plot_imu_increment_cumsums(run_data, save=save))
    figures.extend(plot_imu_debug_terms(run_data, save=save))
    figures.extend(plot_imu_bias_truth_errors(run_data, save=save))
    figures.extend(plot_truth_errors_ecef(run_data, save=save))
    figures.extend(plot_truth_errors_ned(run_data, save=save))

    return figures


def close_figures(figures: list[plt.Figure]) -> None:
    """Close all generated figures."""
    for fig in figures:
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate NavKit analysis plots.")
    parser.add_argument("run_dir", type=Path, help="Run directory containing NavKit CSV logs.")
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

    args = parser.parse_args(argv)

    if not args.show:
        plt.rcParams["figure.max_open_warning"] = 0

    figures = plot_run(args.run_dir, save=not args.no_save)

    if args.show:
        # Call show once after all figures have been created so the user can
        # toggle between windows instead of closing one plot at a time.
        plt.show()
    else:
        close_figures(figures)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
