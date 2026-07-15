# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.style import (
    BOUND_COLOR,
    ERROR_COLOR,
    MEASUREMENT_COLOR,
    RESIDUAL_COLOR,
    TRUTH_COLOR,
    apply_nav_axes_style,
)


def _plot_imu_family(
    run: RunData,
    *,
    truth_prefix: str,
    measured_prefix: str,
    suffix: str,
    ylabel: str,
    title: str,
    output_name: str,
    save: bool,
) -> plt.Figure | None:
    imu = run.imu
    if imu is None:
        print(f"Skipping {title}; missing imu_nominal.csv")
        return None

    time_s = imu["time_s"]
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(title)

    for ax, axis_name in zip(axes, AXES):
        truth_col = f"{truth_prefix}_{axis_name}_{suffix}"
        measured_col = f"{measured_prefix}_{axis_name}_{suffix}"
        ax.plot(time_s, imu[truth_col], color=TRUTH_COLOR, label="truth/ideal")
        ax.plot(time_s, imu[measured_col], color=MEASUREMENT_COLOR, label="measured")
        ax.axhline(0.0, color=BOUND_COLOR, linewidth=0.8)
        ax.set_ylabel(f"{axis_name.upper()} {ylabel}")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / output_name)

    return fig


def plot_imu_increment_time_histories(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot truth/ideal and measured IMU increments."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_imu_family(
            run,
            truth_prefix="truth_delta_theta_ib_b",
            measured_prefix="meas_delta_theta_ib_b",
            suffix="rad",
            ylabel=r"$\Delta\theta$ [rad]",
            title="IMU Incremental Angle: Truth/Ideal vs Measured",
            output_name="imu_delta_theta.png",
            save=save,
        ),
        _plot_imu_family(
            run,
            truth_prefix="truth_delta_v_ib_b",
            measured_prefix="meas_delta_v_ib_b",
            suffix="mps",
            ylabel=r"$\Delta v$ [m/s]",
            title="IMU Incremental Velocity: Truth/Ideal vs Measured",
            output_name="imu_delta_v.png",
            save=save,
        ),
    ]:
        if fig is not None:
            figures.append(fig)
    return figures


def plot_imu_increment_cumsums(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot cumulative truth/ideal and measured IMU increments."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_imu_family(
            run,
            truth_prefix="truth_cumsum_delta_theta_ib_b",
            measured_prefix="meas_cumsum_delta_theta_ib_b",
            suffix="rad",
            ylabel=r"$\sum\Delta\theta$ [rad]",
            title="IMU Cumulative Incremental Angle: Truth/Ideal vs Measured",
            output_name="imu_cumsum_delta_theta.png",
            save=save,
        ),
        _plot_imu_family(
            run,
            truth_prefix="truth_cumsum_delta_v_ib_b",
            measured_prefix="meas_cumsum_delta_v_ib_b",
            suffix="mps",
            ylabel=r"$\sum\Delta v$ [m/s]",
            title="IMU Cumulative Incremental Velocity: Truth/Ideal vs Measured",
            output_name="imu_cumsum_delta_v.png",
            save=save,
        ),
    ]:
        if fig is not None:
            figures.append(fig)
    return figures


def _plot_imu_debug_family(
    run: RunData,
    *,
    column_groups: list[tuple[str, str, str]],
    ylabel: str,
    title: str,
    output_name: str,
    save: bool,
) -> plt.Figure | None:
    imu_debug = run.imu_debug
    if imu_debug is None:
        print(f"Skipping {title}; missing imu_debug_ecef.csv")
        return None

    time_s = imu_debug["time_s"]
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(title)

    for ax, axis_name in zip(axes, AXES):
        for prefix, label, color in column_groups:
            column = f"{prefix}_{axis_name}_{ylabel}"
            ax.plot(time_s, imu_debug[column], color=color, label=label)
        ax.axhline(0.0, color=BOUND_COLOR, linewidth=0.8)
        ax.set_ylabel(f"{axis_name.upper()} [{ylabel}]")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / output_name)

    return fig


def plot_imu_debug_terms(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot selected IMU truth-to-increment debug terms."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_imu_debug_family(
            run,
            column_groups=[
                ("a_bar_e", "a_bar_e", RESIDUAL_COLOR),
                ("gravity_e", "gravity_e", TRUTH_COLOR),
                ("specific_force_e", "specific_force_e", ERROR_COLOR),
            ],
            ylabel="mps2",
            title="IMU Debug ECEF Acceleration and Specific-Force Terms",
            output_name="imu_debug_ecef_specific_force.png",
            save=save,
        ),
        _plot_imu_debug_family(
            run,
            column_groups=[
                ("delta_theta_eb_b", "delta theta e2b", TRUTH_COLOR),
                ("delta_theta_ib_b", "delta theta i2b", ERROR_COLOR),
                ("meas_delta_theta_ib_b", "measured", MEASUREMENT_COLOR),
            ],
            ylabel="rad",
            title="IMU Debug Body-Frame Incremental Angle Terms",
            output_name="imu_debug_delta_theta.png",
            save=save,
        ),
        _plot_imu_debug_family(
            run,
            column_groups=[
                ("delta_v_ib_b", "truth/ideal", TRUTH_COLOR),
                ("meas_delta_v_ib_b", "measured", MEASUREMENT_COLOR),
            ],
            ylabel="mps",
            title="IMU Debug Body-Frame Incremental Velocity Terms",
            output_name="imu_debug_delta_v.png",
            save=save,
        ),
    ]:
        if fig is not None:
            figures.append(fig)
    return figures
