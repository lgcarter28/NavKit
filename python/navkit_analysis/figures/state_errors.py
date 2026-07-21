# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import save_figure
from navkit_analysis.style import BOUND_COLOR, ERROR_COLOR, RESIDUAL_COLOR, apply_nav_axes_style

AXIS_COLORS = {
    "x": "red",
    "y": "blue",
    "z": "green",
    "n": "red",
    "e": "blue",
    "d": "green",
    "roll": "red",
    "pitch": "blue",
    "yaw": "green",
}
RAD_TO_DEG = 180.0 / math.pi
RADPS_TO_MDEGPS = 1000.0 * RAD_TO_DEG
MICRO_G_PER_MPS2 = 1.0e6 / 9.80665
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
        "Attitude [deg]",
    ),
    (
        "Gyro Bias Error",
        ["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
        "Gyro Bias [mdeg/s]",
    ),
    (
        "Accelerometer Bias Error",
        ["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
        "Accel Bias [micro-g]",
    ),
]
NED_STATE_GROUPS = [
    (
        "Position Error",
        ["p_n_m", "p_e_m", "p_d_m"],
        "Position [m]",
    ),
    (
        "Velocity Error",
        ["v_n_mps", "v_e_mps", "v_d_mps"],
        "Velocity [m/s]",
    ),
    (
        "Attitude Error",
        ["theta_roll_rad", "theta_pitch_rad", "theta_yaw_rad"],
        "Attitude [deg]",
    ),
    (
        "Gyro Bias Error",
        ["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
        "Gyro Bias [mdeg/s]",
    ),
    (
        "Accelerometer Bias Error",
        ["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
        "Accel Bias [micro-g]",
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
    for axis_name in ("roll", "pitch", "yaw"):
        if axis_name in label:
            return axis_name
    for axis_name in ("x", "y", "z"):
        if f"_{axis_name}_" in label or label.endswith(f"_{axis_name}"):
            return axis_name
    for axis_name in ("n", "e", "d"):
        if f"_{axis_name}_" in label or label.endswith(f"_{axis_name}"):
            return axis_name
    return "x"


def _scale_for_label(label: str) -> float:
    if label.startswith("theta_") and label.endswith("_rad"):
        return RAD_TO_DEG
    if label.startswith("gyro_bias_") and label.endswith("_radps"):
        return RADPS_TO_MDEGPS
    if label.startswith("accel_bias_") and label.endswith("_mps2"):
        return MICRO_G_PER_MPS2
    return 1.0


def state_error_scale(label: str) -> float:
    """Return the plotting scale used for a state-error label."""
    return _scale_for_label(label)


def truth_error_frame_ecef(run: RunData) -> pd.DataFrame | None:
    """Return the derived ECEF truth-error frame for a run."""
    return run.truth_error


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
        scale = _scale_for_label(label)
        value = scale * frame[f"{spec.value_prefix}{label}"]
        sigma = scale * frame[f"sigma_{label}"]
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
        save_figure(fig, run.figures_dir / spec.output_name)

    return fig


def _plot_state_dashboard(
    *,
    frame: pd.DataFrame | None,
    value_prefix: str,
    title: str,
    output_name: str,
    missing_message: str,
    run: RunData,
    save: bool,
    state_groups: list[tuple[str, list[str], str]] | None = None,
    legend_frame: str = "ecef",
) -> plt.Figure | None:
    if frame is None:
        print(missing_message)
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

    fig.suptitle(title)
    time_s = frame["time_s"]
    value_name = "correction" if value_prefix == "correction_" else "error"
    groups = state_groups if state_groups is not None else STATE_GROUPS

    for ax, (group_title, labels, ylabel) in zip(axes, groups):
        for label in labels:
            value_col = f"{value_prefix}{label}"
            sigma_col = f"sigma_{label}"
            if value_col not in frame or sigma_col not in frame:
                continue
            axis_name = _axis_name_from_label(label)
            color = AXIS_COLORS[axis_name]
            scale = _scale_for_label(label)
            ax.plot(time_s, scale * frame[value_col], color=color, label=f"{axis_name} {value_name}")
            ax.plot(
                time_s,
                3.0 * scale * frame[sigma_col],
                color=color,
                linestyle="--",
                label=f"{axis_name} $3\\sigma$",
            )
            ax.plot(time_s, -3.0 * scale * frame[sigma_col], color=color, linestyle="--")

        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_title(group_title)
        ax.set_ylabel(ylabel)
        apply_nav_axes_style(ax)

    axes[2].set_xlabel("Time [s]")
    axes[4].set_xlabel("Time [s]")
    legend_handles, legend_labels, legend_cols = _dashboard_legend(value_name, legend_frame)
    empty_axis.legend(legend_handles, legend_labels, loc="center", ncols=legend_cols)

    if save:
        save_figure(fig, run.figures_dir / output_name)

    return fig


def _legend_line(axis_name: str, bound: bool) -> Line2D:
    return Line2D(
        [0],
        [0],
        color=AXIS_COLORS[axis_name],
        linestyle="--" if bound else "-",
    )


def _dashboard_legend(value_name: str, legend_frame: str) -> tuple[list[Line2D], list[str], int]:
    if legend_frame == "ned":
        local_axes = ("n", "e", "d")
        body_axes = ("x", "y", "z")
        handles = (
            [_legend_line(axis_name, False) for axis_name in local_axes]
            + [_legend_line(axis_name, True) for axis_name in local_axes]
            + [_legend_line(axis_name, False) for axis_name in body_axes]
            + [_legend_line(axis_name, True) for axis_name in body_axes]
        )
        labels = (
            [f"{axis_name.upper()} {value_name}" for axis_name in local_axes]
            + [f"{axis_name.upper()} $3\\sigma$" for axis_name in local_axes]
            + [f"{axis_name} {value_name}" for axis_name in body_axes]
            + [f"{axis_name} $3\\sigma$" for axis_name in body_axes]
        )
        return handles, labels, 4

    axes = ("x", "y", "z")
    handles = [_legend_line(axis_name, False) for axis_name in axes] + [
        _legend_line(axis_name, True) for axis_name in axes
    ]
    labels = [f"{axis_name} {value_name}" for axis_name in axes] + [
        f"{axis_name} $3\\sigma$" for axis_name in axes
    ]
    return handles, labels, 2


def plot_filter_corrections(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot real-run filter correction vector components inside covariance bounds."""
    return _plot_state_dashboard(
        frame=run.filter_correction,
        value_prefix="correction_",
        title=r"Filter Correction Dashboard with $3\sigma$ Bounds",
        output_name="filter_correction_covariance_dashboard_ecef.png",
        missing_message="Skipping filter correction plot; missing filter_correction_ecef.csv",
        run=run,
        save=save,
        legend_frame="ecef",
    )


def plot_truth_errors(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot grouped simulation truth errors inside 3-sigma covariance bounds."""
    frame = run.truth_error
    if frame is None:
        print("Skipping truth error dashboard; missing truth/nav estimate logs")
        return None

    return _plot_state_dashboard(
        frame=frame,
        value_prefix="error_",
        title=r"Truth Error Dashboard with $3\sigma$ Bounds",
        output_name="error_covariance_dashboard_ecef.png",
        missing_message="Skipping truth error dashboard; missing truth/nav estimate logs",
        run=run,
        save=save,
        legend_frame="ecef",
    )


def _copy_truth_error_columns(
    source: pd.DataFrame, labels: list[str], target: pd.DataFrame
) -> None:
    for label in labels:
        value_col = f"error_{label}"
        sigma_col = f"sigma_{label}"
        if value_col in source and sigma_col in source:
            target[value_col] = source[value_col]
            target[sigma_col] = source[sigma_col]


def _add_ned_dashboard_group(
    *,
    source: pd.DataFrame,
    state_labels: list[str],
    truth_positions: np.ndarray,
    covariances: list[np.ndarray | None],
    source_labels: list[str],
    target_labels: list[str],
    target: pd.DataFrame,
) -> bool:
    if any(label not in state_labels for label in source_labels):
        return False
    if any(covariance is None for covariance in covariances):
        return False

    label_indices = [state_labels.index(label) for label in source_labels]
    ned_errors = []
    ned_sigmas = []
    for row_idx, (_, row) in enumerate(source.iterrows()):
        error_e = np.array([row[f"error_{label}"] for label in source_labels])
        C_e2n = _ecef_to_ned_matrix(truth_positions[row_idx])
        covariance = covariances[row_idx]
        assert covariance is not None
        covariance_e = covariance[np.ix_(label_indices, label_indices)]
        covariance_n = C_e2n @ covariance_e @ C_e2n.T
        ned_errors.append(C_e2n @ error_e)
        ned_sigmas.append(np.sqrt(np.diag(covariance_n)))

    ned_errors_array = np.vstack(ned_errors)
    ned_sigmas_array = np.vstack(ned_sigmas)
    for idx, target_label in enumerate(target_labels):
        target[f"error_{target_label}"] = ned_errors_array[:, idx]
        target[f"sigma_{target_label}"] = ned_sigmas_array[:, idx]
    return True


def _ned_dashboard_frame(run: RunData) -> pd.DataFrame | None:
    source = run.truth_error
    if source is None:
        return None

    truth_positions = _interp_truth_position(run, source["time_s"].to_numpy())
    if truth_positions is None:
        return None

    state_labels = _state_labels(source, "error_")
    covariances = [_full_covariance(source, state_labels, idx) for idx in range(len(source))]
    target = pd.DataFrame({"time_s": source["time_s"]})

    groups_added = [
        _add_ned_dashboard_group(
            source=source,
            state_labels=state_labels,
            truth_positions=truth_positions,
            covariances=covariances,
            source_labels=["p_e_x_m", "p_e_y_m", "p_e_z_m"],
            target_labels=["p_n_m", "p_e_m", "p_d_m"],
            target=target,
        ),
        _add_ned_dashboard_group(
            source=source,
            state_labels=state_labels,
            truth_positions=truth_positions,
            covariances=covariances,
            source_labels=["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
            target_labels=["v_n_mps", "v_e_mps", "v_d_mps"],
            target=target,
        ),
        _add_ned_dashboard_group(
            source=source,
            state_labels=state_labels,
            truth_positions=truth_positions,
            covariances=covariances,
            source_labels=["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
            target_labels=["theta_roll_rad", "theta_pitch_rad", "theta_yaw_rad"],
            target=target,
        ),
    ]
    if not all(groups_added):
        return None

    _copy_truth_error_columns(
        source,
        ["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
        target,
    )
    _copy_truth_error_columns(
        source,
        ["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
        target,
    )
    return target


def truth_error_frame_ned(run: RunData) -> pd.DataFrame | None:
    """Return the NED/local-level truth-error frame for a run."""
    return _ned_dashboard_frame(run)


def plot_truth_errors_ned_dashboard(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot grouped simulation truth errors in local NED axes with body IMU-bias states."""
    frame = _ned_dashboard_frame(run)
    if frame is None:
        print("Skipping NED truth error dashboard; missing truth/nav estimate logs or covariance")
        return None

    return _plot_state_dashboard(
        frame=frame,
        value_prefix="error_",
        title=r"NED Error Dashboard with $3\sigma$ Bounds",
        output_name="error_covariance_dashboard_ned.png",
        missing_message="Skipping NED truth error dashboard; missing truth/nav estimate logs",
        run=run,
        save=save,
        state_groups=NED_STATE_GROUPS,
        legend_frame="ned",
    )


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
        scale = _scale_for_label(label)
        ax.plot(time_s, scale * frame[value_col], color=ERROR_COLOR, label=f"{axis_name} error")
        ax.plot(
            time_s,
            scale * frame[sigma_col],
            color=BOUND_COLOR,
            linestyle="--",
            label=r"$1\sigma$",
        )
        ax.plot(time_s, -scale * frame[sigma_col], color=BOUND_COLOR, linestyle="--")
        ax.plot(
            time_s,
            3.0 * scale * frame[sigma_col],
            color=BOUND_COLOR,
            linestyle="-",
            label=r"$3\sigma$",
        )
        ax.plot(time_s, -3.0 * scale * frame[sigma_col], color=BOUND_COLOR, linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(f"{axis_name.upper()} {ylabel}")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.figures_dir / output_name)

    return fig


def plot_imu_bias_truth_errors(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot dedicated gyro and accelerometer bias truth-error figures."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_truth_error_subset(
            run,
            labels=["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"],
            title=r"Gyro Bias Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Bias [mdeg/s]",
            output_name="error_covariance_gyro_bias_body.png",
            save=save,
        ),
        _plot_truth_error_subset(
            run,
            labels=["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"],
            title=r"Accelerometer Bias Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Bias [micro-g]",
            output_name="error_covariance_accel_bias_body.png",
            save=save,
        ),
    ]:
        if fig is not None:
            figures.append(fig)
    return figures


def plot_truth_errors_ecef(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot dedicated ECEF velocity and attitude truth-error figures."""
    figures: list[plt.Figure] = []
    for fig in [
        _plot_truth_error_subset(
            run,
            labels=["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
            title=r"ECEF Velocity Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Velocity [m/s]",
            output_name="error_covariance_velocity_ecef.png",
            save=save,
        ),
        _plot_truth_error_subset(
            run,
            labels=["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
            title=r"Roll/Pitch/Yaw Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            ylabel="Attitude [deg]",
            output_name="error_covariance_attitude_ecef.png",
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
    plot_scale = _scale_for_label(labels[0])
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
        err = plot_scale * ned_errors_array[:, idx]
        sigma = plot_scale * ned_sigmas_array[:, idx]
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
        save_figure(fig, run.figures_dir / output_name)

    return fig


def plot_truth_errors_ned(run: RunData, save: bool = True) -> list[plt.Figure]:
    """Plot NED-transformed position, velocity, and attitude truth errors."""
    figures: list[plt.Figure] = []
    groups = [
        (
            ["p_e_x_m", "p_e_y_m", "p_e_z_m"],
            r"NED Position Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "error_covariance_position_ned.png",
            ("north", "east", "down"),
        ),
        (
            ["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
            r"NED Velocity Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "error_covariance_velocity_ned.png",
            ("north", "east", "down"),
        ),
        (
            ["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
            r"Local Roll/Pitch/Yaw Truth Error with $1\sigma$ and $3\sigma$ Bounds",
            "error_covariance_attitude_ned.png",
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
