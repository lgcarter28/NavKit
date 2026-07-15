# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import save_figure
from navkit_analysis.style import BOUND_COLOR, ERROR_COLOR, RESIDUAL_COLOR, apply_nav_axes_style

AXIS_COLORS = {"x": "red", "y": "blue", "z": "green"}
STATE_GROUPS = [
    (
        "Position Error",
        ["p_e_x_m", "p_e_y_m", "p_e_z_m"],
        "Position [m]",
    ),
    (
        "Velocity Error",
        ["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
        "Velocity [m/s]",
    ),
    (
        "Attitude Error",
        ["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
        "Attitude [rad]",
    ),
    (
        "Gyro Bias Error",
        ["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
        "Gyro Bias [rad/s]",
    ),
    (
        "Accelerometer Bias Error",
        ["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
        "Accel Bias [m/s^2]",
    ),
]


@dataclass(frozen=True)
class StateLogSpec:
    frame: pd.DataFrame | None
    value_prefix: str
    title: str
    output_name: str
    missing_message: str
    color: str


def _state_labels(frame: pd.DataFrame, value_prefix: str) -> list[str]:
    return [
        col.removeprefix(value_prefix)
        for col in frame.columns
        if col.startswith(value_prefix) and f"sigma_{col.removeprefix(value_prefix)}" in frame
    ]


def _pretty_label(label: str) -> str:
    return label.replace("_", " ")


def _axis_name_from_label(label: str) -> str:
    for axis_name in ("x", "y", "z"):
        if f"_{axis_name}_" in label or label.endswith(f"_{axis_name}"):
            return axis_name
    return "x"


def _plot_state_log(spec: StateLogSpec, run: RunData, save: bool) -> plt.Figure | None:
    frame = spec.frame
    if frame is None:
        print(spec.missing_message)
        return None

    labels = _state_labels(frame, spec.value_prefix)
    if not labels:
        print(f"Skipping {spec.title}; no state value/sigma column pairs found")
        return None

    time_s = frame["time_s"]
    nrows = len(labels)
    fig_height = max(8.0, 2.0 * nrows)
    fig, axes = plt.subplots(
        nrows=nrows,
        ncols=1,
        sharex=True,
        figsize=(14.0, fig_height),
        constrained_layout=True,
    )
    if nrows == 1:
        axes = [axes]

    fig.suptitle(spec.title)

    for ax, label in zip(axes, labels):
        value = frame[f"{spec.value_prefix}{label}"]
        sigma = frame[f"sigma_{label}"]
        ax.plot(time_s, value, color=spec.color, label=_pretty_label(label))
        ax.plot(time_s, sigma, color=BOUND_COLOR, linestyle="--", label=r"$1\sigma$")
        ax.plot(time_s, -sigma, color=BOUND_COLOR, linestyle="--")
        ax.plot(time_s, 3.0 * sigma, color=BOUND_COLOR, linestyle="-", label=r"$3\sigma$")
        ax.plot(time_s, -3.0 * sigma, color=BOUND_COLOR, linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(_pretty_label(label))
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / spec.output_name)

    return fig


def plot_filter_corrections(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot real-run filter correction vector components inside covariance bounds."""
    return _plot_state_log(
        StateLogSpec(
            frame=run.filter_correction,
            value_prefix="correction_",
            title=r"Filter Correction Components with $1\sigma$ and $3\sigma$ Bounds",
            output_name="filter_correction_covariance.png",
            missing_message="Skipping filter correction plot; missing filter_correction_ecef.csv",
            color=RESIDUAL_COLOR,
        ),
        run,
        save,
    )


def plot_truth_errors(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot grouped simulation truth errors inside 3-sigma covariance bounds."""
    frame = run.truth_error
    if frame is None:
        print("Skipping truth error dashboard; missing truth/nav estimate logs")
        return None

    fig = plt.figure(figsize=(16.0, 10.0), constrained_layout=True)
    axes = [
        fig.add_subplot(3, 2, 1),
        fig.add_subplot(3, 2, 3),
        fig.add_subplot(3, 2, 5),
        fig.add_subplot(3, 2, 2),
        fig.add_subplot(3, 2, 4),
    ]
    empty_axis = fig.add_subplot(3, 2, 6)
    empty_axis.axis("off")

    fig.suptitle(r"Truth Error Dashboard with $3\sigma$ Bounds")
    time_s = frame["time_s"]

    for ax, (title, labels, ylabel) in zip(axes, STATE_GROUPS):
        for label in labels:
            value_col = f"error_{label}"
            sigma_col = f"sigma_{label}"
            if value_col not in frame or sigma_col not in frame:
                continue
            axis_name = _axis_name_from_label(label)
            color = AXIS_COLORS[axis_name]
            ax.plot(time_s, frame[value_col], color=color, label=f"{axis_name} error")
            ax.plot(time_s, 3.0 * frame[sigma_col], color=color, linestyle="--", label=f"{axis_name} $3\\sigma$")
            ax.plot(time_s, -3.0 * frame[sigma_col], color=color, linestyle="--")

        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", ncols=2)
        apply_nav_axes_style(ax)

    axes[2].set_xlabel("Time [s]")
    axes[4].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / "truth_error_covariance.png")

    return fig


def _plot_truth_error_subset(
    run: RunData,
    *,
    labels: list[str],
    title: str,
    ylabel: str,
    output_name: str,
    save: bool,
) -> plt.Figure | None:
    frame = run.truth_error
    if frame is None:
        print(f"Skipping {title}; missing truth/nav estimate logs")
        return None

    time_s = frame["time_s"]
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(title)

    for ax, label in zip(axes, labels):
        value_col = f"error_{label}"
        sigma_col = f"sigma_{label}"
        if value_col not in frame or sigma_col not in frame:
            continue
        axis_name = _axis_name_from_label(label)
        color = AXIS_COLORS[axis_name]
        ax.plot(time_s, frame[value_col], color=color, label=f"{axis_name} error")
        ax.plot(time_s, frame[sigma_col], color=BOUND_COLOR, linestyle="--", label=r"$1\sigma$")
        ax.plot(time_s, -frame[sigma_col], color=BOUND_COLOR, linestyle="--")
        ax.plot(time_s, 3.0 * frame[sigma_col], color=color, linestyle="-", label=r"$3\sigma$")
        ax.plot(time_s, -3.0 * frame[sigma_col], color=color, linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(f"{axis_name.upper()} {ylabel}")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / output_name)

    return fig


def plot_imu_bias_truth_errors(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot dedicated gyro and accelerometer bias truth-error figures."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_truth_error_subset(
            run,
            labels=["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
            title=r"Gyro Bias Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Bias [rad/s]",
            output_name="imu_gyro_bias_error_covariance.png",
            save=save,
        ),
        _plot_truth_error_subset(
            run,
            labels=["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
            title=r"Accelerometer Bias Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Bias [m/s^2]",
            output_name="imu_accel_bias_error_covariance.png",
            save=save,
        ),
    ]:
        if fig is not None:
            figures.append(fig)
    return figures


def _ecef_to_ned_matrix(p_e: np.ndarray) -> np.ndarray:
    x, y, z = p_e
    lon = math.atan2(y, x)
    hyp = math.hypot(x, y)
    lat = math.atan2(z, hyp)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    return np.array(
        [
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [-sin_lon, cos_lon, 0.0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
        ]
    )


def _full_covariance(frame: pd.DataFrame, labels: list[str], index: int) -> np.ndarray | None:
    covariance = np.zeros((len(labels), len(labels)))
    for row, row_label in enumerate(labels):
        for col, col_label in enumerate(labels[row:], start=row):
            column = f"P_{row_label}__{col_label}"
            reverse_column = f"P_{col_label}__{row_label}"
            if column in frame:
                covariance[row, col] = frame.iloc[index][column]
            elif reverse_column in frame:
                covariance[row, col] = frame.iloc[index][reverse_column]
            else:
                return None
            covariance[col, row] = covariance[row, col]
    return covariance


def _interp_truth_position(run: RunData, time_s: np.ndarray) -> np.ndarray | None:
    truth = run.truth
    if truth is None:
        return None
    return np.column_stack(
        [
            np.interp(time_s, truth["time_s"], truth["p_e_x_m"]),
            np.interp(time_s, truth["time_s"], truth["p_e_y_m"]),
            np.interp(time_s, truth["time_s"], truth["p_e_z_m"]),
        ]
    )


def _plot_ned_group(
    run: RunData,
    *,
    labels: list[str],
    source_title: str,
    output_name: str,
    axes_names: tuple[str, str, str],
    save: bool,
) -> plt.Figure | None:
    frame = run.truth_error
    if frame is None:
        print(f"Skipping {source_title}; missing derived truth error")
        return None

    state_labels = _state_labels(frame, "error_")
    if any(label not in state_labels for label in labels):
        print(f"Skipping {source_title}; missing required labels {labels}")
        return None

    truth_positions = _interp_truth_position(run, frame["time_s"].to_numpy())
    if truth_positions is None:
        print(f"Skipping {source_title}; missing truth_trajectory_ecef.csv")
        return None

    covariances = [_full_covariance(frame, state_labels, idx) for idx in range(len(frame))]
    if any(cov is None for cov in covariances):
        print(f"Skipping {source_title}; full triangular covariance is not available")
        return None

    label_indices = [state_labels.index(label) for label in labels]
    ned_errors = []
    ned_sigmas = []
    for row_idx, (_, row) in enumerate(frame.iterrows()):
        error_e = np.array([row[f"error_{label}"] for label in labels])
        C_e2n = _ecef_to_ned_matrix(truth_positions[row_idx])
        covariance_e = covariances[row_idx][np.ix_(label_indices, label_indices)]  # type: ignore[index]
        covariance_n = C_e2n @ covariance_e @ C_e2n.T
        ned_errors.append(C_e2n @ error_e)
        ned_sigmas.append(np.sqrt(np.diag(covariance_n)))

    ned_errors_array = np.vstack(ned_errors)
    ned_sigmas_array = np.vstack(ned_sigmas)
    time_s = frame["time_s"]
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(source_title)

    for idx, (ax, axis_name) in enumerate(zip(axes, axes_names)):
        err = ned_errors_array[:, idx]
        sigma = ned_sigmas_array[:, idx]
        ax.plot(time_s, err, color=ERROR_COLOR, label=f"{axis_name} error")
        ax.plot(time_s, sigma, color=BOUND_COLOR, linestyle="--", label=r"$1\sigma$")
        ax.plot(time_s, -sigma, color=BOUND_COLOR, linestyle="--")
        ax.plot(time_s, 3.0 * sigma, color=BOUND_COLOR, linestyle="-", label=r"$3\sigma$")
        ax.plot(time_s, -3.0 * sigma, color=BOUND_COLOR, linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(axis_name.title())
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / output_name)

    return fig


def plot_truth_errors_ned(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot NED-transformed position, velocity, and attitude truth errors."""
    figures: list[plt.Figure] = []
    groups = [
        (
            ["p_e_x_m", "p_e_y_m", "p_e_z_m"],
            r"NED Position Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "truth_error_position_ned_covariance.png",
            ("north", "east", "down"),
        ),
        (
            ["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
            r"NED Velocity Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "truth_error_velocity_ned_covariance.png",
            ("north", "east", "down"),
        ),
        (
            ["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
            r"Local Roll/Pitch/Yaw Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "truth_error_attitude_ned_covariance.png",
            ("roll", "pitch", "yaw"),
        ),
    ]
    for labels, title, output_name, axes_names in groups:
        fig = _plot_ned_group(
            run,
            labels=labels,
            source_title=title,
            output_name=output_name,
            axes_names=axes_names,
            save=save,
        )
        if fig is not None:
            figures.append(fig)
    return figures
