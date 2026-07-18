# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt
import pandas as pd

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.style import apply_nav_axes_style


def _plot_truth_measured_error(
    frame: pd.DataFrame,
    truth_columns: list[str],
    measured_columns: list[str],
    sigma_columns: list[str],
    ylabel: str,
    title: str,
    output_name: str,
    run: RunData,
    save: bool,
) -> plt.Figure:
    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=(14.0, 7.5),
        constrained_layout=True,
    )
    fig.suptitle(title)

    time_s = frame["time_s"]
    colors = {"x": "red", "y": "blue", "z": "green"}

    for axis_name, truth_column, measured_column in zip(AXES, truth_columns, measured_columns):
        axes[0].scatter(
            time_s,
            frame[truth_column],
            s=12,
            color="black",
            marker=".",
            label="truth" if axis_name == "x" else None,
        )
        axes[0].scatter(
            time_s,
            frame[measured_column],
            s=16,
            facecolors="none",
            edgecolors="green",
            marker="o",
            label=f"measured {axis_name}",
        )
        axes[1].plot(
            time_s,
            frame[measured_column] - frame[truth_column],
            color=colors[axis_name],
            label=f"{axis_name} error",
        )

    for axis_name, sigma_column in zip(AXES, sigma_columns):
        three_sigma = 3.0 * frame[sigma_column]
        axes[1].plot(
            time_s,
            three_sigma,
            color=colors[axis_name],
            linestyle="--",
            label=f"{axis_name} 3 sigma",
        )
        axes[1].plot(time_s, -three_sigma, color=colors[axis_name], linestyle="--")

    axes[0].set_ylabel(ylabel)
    axes[1].set_ylabel(f"error [{ylabel.split('[')[-1]}")
    axes[1].set_xlabel("Time [s]")
    axes[0].legend(loc="upper right")
    axes[1].legend(loc="upper right")

    for ax in axes:
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        apply_nav_axes_style(ax)

    if save:
        save_figure(fig, run.figures_dir / output_name)

    return fig


def plot_gnss_position_debug(run: RunData, save: bool = True) -> plt.Figure | None:
    if run.gnss_position_debug is None:
        print("Skipping GNSS position debug plot; missing gnss_position_debug_ecef.csv")
        return None

    return _plot_truth_measured_error(
        run.gnss_position_debug,
        ["truth_p_e_x_m", "truth_p_e_y_m", "truth_p_e_z_m"],
        ["measured_p_e_x_m", "measured_p_e_y_m", "measured_p_e_z_m"],
        ["sigma_p_e_x_m", "sigma_p_e_y_m", "sigma_p_e_z_m"],
        "Position [m]",
        "GNSS Position Truth and Measurement",
        "gnss_position_debug_ecef.png",
        run,
        save,
    )


def plot_gnss_velocity_debug(run: RunData, save: bool = True) -> plt.Figure | None:
    if run.gnss_velocity_debug is None:
        print("Skipping GNSS velocity debug plot; missing gnss_velocity_debug_ecef.csv")
        return None

    return _plot_truth_measured_error(
        run.gnss_velocity_debug,
        ["truth_v_e_x_mps", "truth_v_e_y_mps", "truth_v_e_z_mps"],
        ["measured_v_e_x_mps", "measured_v_e_y_mps", "measured_v_e_z_mps"],
        ["sigma_v_e_x_mps", "sigma_v_e_y_mps", "sigma_v_e_z_mps"],
        "Velocity [m/s]",
        "GNSS Velocity Truth and Measurement",
        "gnss_velocity_debug_ecef.png",
        run,
        save,
    )
