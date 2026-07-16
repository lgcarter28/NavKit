# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.style import (
    BOUND_COLOR,
    ERROR_COLOR,
    apply_nav_axes_style,
    axis_position_error_label,
    ecef_position_error_label,
)


def _position_columns(axis_name: str) -> tuple[str, str]:
    err_col = f"error_p_e_{axis_name}_m"
    sigma_col = f"sigma_p_e_{axis_name}_m"
    return err_col, sigma_col


def plot_position_error_covariance(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot ECEF position error with 1-sigma and 3-sigma covariance bounds."""
    truth_error = run.truth_error
    if truth_error is None:
        print("Skipping position error covariance plot; missing truth/nav estimate logs")
        return None
    time_s = truth_error["time_s"]

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )

    fig.suptitle(r"ECEF Position Error with $1\sigma$ and $3\sigma$ Bounds")

    for ax, axis_name in zip(axes, AXES):
        err_col, sigma_col = _position_columns(axis_name)
        err = truth_error[err_col]
        sigma = truth_error[sigma_col]

        ax.plot(
            time_s,
            err,
            color=ERROR_COLOR,
            label=ecef_position_error_label(axis_name),
        )

        ax.plot(
            time_s,
            sigma,
            color=BOUND_COLOR,
            linestyle="--",
            label=r"$1\sigma$",
        )
        ax.plot(time_s, -sigma, color=BOUND_COLOR, linestyle="--")

        ax.plot(
            time_s,
            3.0 * sigma,
            color=BOUND_COLOR,
            linestyle="-",
            label=r"$3\sigma$",
        )
        ax.plot(time_s, -3.0 * sigma, color=BOUND_COLOR, linestyle="-")

        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(axis_position_error_label(axis_name))
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.figures_dir / "error_covariance_position_ecef.png")

    return fig
