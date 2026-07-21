# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection

from navkit_analysis.data import (
    derive_truth_error_frame,
    read_navkit_csv,
    require_columns,
    resolve_run_dirs,
)
from navkit_analysis.figures.state_errors import state_error_scale
from navkit_analysis.style import apply_nav_axes_style, apply_style


@dataclass(frozen=True)
class MonteCarloStateGroup:
    """State-error/covariance group to aggregate across Monte Carlo runs."""

    labels: tuple[str, str, str]
    axis_names: tuple[str, str, str]
    title: str
    ylabel: str
    output_name: str


@dataclass(frozen=True)
class MonteCarloSeries:
    """Aggregate Monte Carlo state-error series for one state group."""

    time_s: np.ndarray
    labels: tuple[str, str, str]
    axis_names: tuple[str, str, str]
    run_errors: np.ndarray
    mean_error: np.ndarray
    empirical_sigma: np.ndarray
    mean_filter_sigma: np.ndarray
    scale: float
    title: str
    ylabel: str
    output_name: str
    run_count: int


@dataclass(frozen=True)
class MonteCarloRunFrames:
    """Minimal per-run frames required by Monte Carlo aggregate plots."""

    ecef: pd.DataFrame
    ned: pd.DataFrame | None


ECEF_GROUPS = (
    MonteCarloStateGroup(
        labels=("p_e_x_m", "p_e_y_m", "p_e_z_m"),
        axis_names=("x", "y", "z"),
        title="Monte Carlo ECEF Position Error/Covariance",
        ylabel="Position [m]",
        output_name="monte_carlo_error_covariance_position_ecef.png",
    ),
    MonteCarloStateGroup(
        labels=("v_e_x_mps", "v_e_y_mps", "v_e_z_mps"),
        axis_names=("x", "y", "z"),
        title="Monte Carlo ECEF Velocity Error/Covariance",
        ylabel="Velocity [m/s]",
        output_name="monte_carlo_error_covariance_velocity_ecef.png",
    ),
    MonteCarloStateGroup(
        labels=("theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"),
        axis_names=("x", "y", "z"),
        title="Monte Carlo ECEF Attitude Error/Covariance",
        ylabel="Attitude [deg]",
        output_name="monte_carlo_error_covariance_attitude_ecef.png",
    ),
    MonteCarloStateGroup(
        labels=("gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"),
        axis_names=("x", "y", "z"),
        title="Monte Carlo Gyro Bias Error/Covariance",
        ylabel="Gyro Bias [mdeg/s]",
        output_name="monte_carlo_error_covariance_gyro_bias_body.png",
    ),
    MonteCarloStateGroup(
        labels=("accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"),
        axis_names=("x", "y", "z"),
        title="Monte Carlo Accelerometer Bias Error/Covariance",
        ylabel="Accel Bias [micro-g]",
        output_name="monte_carlo_error_covariance_accel_bias_body.png",
    ),
)

NED_GROUPS = (
    MonteCarloStateGroup(
        labels=("p_n_m", "p_e_m", "p_d_m"),
        axis_names=("north", "east", "down"),
        title="Monte Carlo NED Position Error/Covariance",
        ylabel="Position [m]",
        output_name="monte_carlo_error_covariance_position_ned.png",
    ),
    MonteCarloStateGroup(
        labels=("v_n_mps", "v_e_mps", "v_d_mps"),
        axis_names=("north", "east", "down"),
        title="Monte Carlo NED Velocity Error/Covariance",
        ylabel="Velocity [m/s]",
        output_name="monte_carlo_error_covariance_velocity_ned.png",
    ),
    MonteCarloStateGroup(
        labels=("theta_roll_rad", "theta_pitch_rad", "theta_yaw_rad"),
        axis_names=("roll", "pitch", "yaw"),
        title="Monte Carlo Local Attitude Error/Covariance",
        ylabel="Attitude [deg]",
        output_name="monte_carlo_error_covariance_attitude_ned.png",
    ),
)


def _read_first_existing_csv(
    paths: list[Path], usecols: set[str] | None = None
) -> tuple[pd.DataFrame | None, Path | None]:
    for path in paths:
        if path.exists():
            frame = read_navkit_csv(path, usecols)
            if not frame.empty:
                return frame, path
    return None, None


def _required_nav_columns() -> list[str]:
    state_columns = [
        "time_s",
        "p_e_x_m",
        "p_e_y_m",
        "p_e_z_m",
        "v_e_x_mps",
        "v_e_y_mps",
        "v_e_z_mps",
        "q_b2e_w",
        "q_b2e_x",
        "q_b2e_y",
        "q_b2e_z",
        "gyro_bias_b_x_radps",
        "gyro_bias_b_y_radps",
        "gyro_bias_b_z_radps",
        "accel_bias_b_x_mps2",
        "accel_bias_b_y_mps2",
        "accel_bias_b_z_mps2",
    ]
    sigma_columns = [f"sigma_{label}" for group in ECEF_GROUPS for label in group.labels]
    return state_columns + sigma_columns


def _covariance_columns_for_labels(labels: tuple[str, str, str]) -> set[str]:
    return {f"P_{row_label}__{col_label}" for row_label in labels for col_label in labels}


def _monte_carlo_nav_columns() -> set[str]:
    columns = set(_required_nav_columns())
    for group in ECEF_GROUPS[:3]:
        columns.update(_covariance_columns_for_labels(group.labels))
    return columns


def _required_truth_columns() -> list[str]:
    return [
        "time_s",
        "p_e_x_m",
        "p_e_y_m",
        "p_e_z_m",
        "v_e_x_mps",
        "v_e_y_mps",
        "v_e_z_mps",
        "q_b2e_w",
        "q_b2e_x",
        "q_b2e_y",
        "q_b2e_z",
    ]


def _required_imu_columns() -> list[str]:
    return [
        "time_s",
        "truth_gyro_bias_b_x_radps",
        "truth_gyro_bias_b_y_radps",
        "truth_gyro_bias_b_z_radps",
        "truth_accel_bias_b_x_mps2",
        "truth_accel_bias_b_y_mps2",
        "truth_accel_bias_b_z_mps2",
    ]


def load_monte_carlo_run_frames(run_dir: Path) -> MonteCarloRunFrames:
    """Load only the logs needed for Monte Carlo aggregate covariance plots."""
    _, data_dir, _ = resolve_run_dirs(run_dir)

    nav, nav_path = _read_first_existing_csv(
        [data_dir / "nav_estimate_ecef.csv", run_dir / "nav_estimate_ecef.csv"],
        _monte_carlo_nav_columns(),
    )
    if nav is None or nav_path is None:
        raise FileNotFoundError(f"missing navigation estimate log in {run_dir}")
    require_columns(nav, _required_nav_columns(), nav_path)

    truth, truth_path = _read_first_existing_csv(
        [
            data_dir / "truth_trajectory_ecef.csv",
            data_dir / "truth.csv",
            run_dir / "truth_trajectory_ecef.csv",
        ],
        set(_required_truth_columns()),
    )
    if truth is None or truth_path is None:
        raise FileNotFoundError(f"missing truth trajectory log in {run_dir}")
    require_columns(truth, _required_truth_columns(), truth_path)

    imu, imu_path = _read_first_existing_csv(
        [data_dir / "imu_nominal.csv", data_dir / "imu.csv", run_dir / "imu_nominal.csv"],
        set(_required_imu_columns()),
    )
    if imu is not None and imu_path is not None:
        require_columns(imu, _required_imu_columns(), imu_path)

    ecef = derive_truth_error_frame(nav, truth, imu)
    if ecef is None:
        raise ValueError(f"could not derive truth-error frame for {run_dir}")

    return MonteCarloRunFrames(ecef=ecef, ned=build_monte_carlo_ned_frame(ecef, truth))


def load_successful_runs(run_dirs: list[Path]) -> list[MonteCarloRunFrames]:
    """Load minimal run data for successful Monte Carlo runs."""
    return [load_monte_carlo_run_frames(run_dir) for run_dir in run_dirs]


def _ecef_to_ned_matrices(p_e: np.ndarray) -> np.ndarray:
    x = p_e[:, 0]
    y = p_e[:, 1]
    z = p_e[:, 2]
    lon = np.arctan2(y, x)
    hyp = np.hypot(x, y)
    lat = np.arctan2(z, hyp)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    matrices = np.zeros((len(p_e), 3, 3))
    matrices[:, 0, 0] = -sin_lat * cos_lon
    matrices[:, 0, 1] = -sin_lat * sin_lon
    matrices[:, 0, 2] = cos_lat
    matrices[:, 1, 0] = -sin_lon
    matrices[:, 1, 1] = cos_lon
    matrices[:, 2, 0] = -cos_lat * cos_lon
    matrices[:, 2, 1] = -cos_lat * sin_lon
    matrices[:, 2, 2] = -sin_lat
    return matrices


def _covariance_series(frame: pd.DataFrame, source_labels: list[str]) -> np.ndarray | None:
    covariance = np.zeros((len(frame), len(source_labels), len(source_labels)))
    for row, row_label in enumerate(source_labels):
        for col, col_label in enumerate(source_labels[row:], start=row):
            column = f"P_{row_label}__{col_label}"
            reverse_column = f"P_{col_label}__{row_label}"
            if column in frame:
                values = frame[column].to_numpy()
            elif reverse_column in frame:
                values = frame[reverse_column].to_numpy()
            else:
                return None
            covariance[:, row, col] = values
            covariance[:, col, row] = values
    return covariance


def _interp_truth_position(truth: pd.DataFrame, time_s: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [
            np.interp(time_s, truth["time_s"], truth["p_e_x_m"]),
            np.interp(time_s, truth["time_s"], truth["p_e_y_m"]),
            np.interp(time_s, truth["time_s"], truth["p_e_z_m"]),
        ]
    )


def _add_ned_group(
    *,
    source: pd.DataFrame,
    truth_positions: np.ndarray,
    source_labels: list[str],
    target_labels: tuple[str, str, str],
    target: pd.DataFrame,
) -> bool:
    error_columns = [f"error_{label}" for label in source_labels]
    if any(column not in source for column in error_columns):
        return False

    covariance_e = _covariance_series(source, source_labels)
    if covariance_e is None:
        return False

    errors_e = source[error_columns].to_numpy()
    C_e2n = _ecef_to_ned_matrices(truth_positions)
    ned_errors = np.einsum("tij,tj->ti", C_e2n, errors_e)
    CP = np.matmul(C_e2n, covariance_e)
    covariance_n = np.matmul(CP, np.swapaxes(C_e2n, 1, 2))
    ned_sigmas = np.sqrt(np.diagonal(covariance_n, axis1=1, axis2=2))

    for index, target_label in enumerate(target_labels):
        target[f"error_{target_label}"] = ned_errors[:, index]
        target[f"sigma_{target_label}"] = ned_sigmas[:, index]
    return True


def build_monte_carlo_ned_frame(
    ecef: pd.DataFrame, truth: pd.DataFrame
) -> pd.DataFrame | None:
    """Build the NED/local-level truth-error frame needed by MC plots."""
    time_s = ecef["time_s"].to_numpy()
    truth_positions = _interp_truth_position(truth, time_s)
    target = pd.DataFrame({"time_s": ecef["time_s"]})

    groups_added = [
        _add_ned_group(
            source=ecef,
            truth_positions=truth_positions,
            source_labels=["p_e_x_m", "p_e_y_m", "p_e_z_m"],
            target_labels=("p_n_m", "p_e_m", "p_d_m"),
            target=target,
        ),
        _add_ned_group(
            source=ecef,
            truth_positions=truth_positions,
            source_labels=["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"],
            target_labels=("v_n_mps", "v_e_mps", "v_d_mps"),
            target=target,
        ),
        _add_ned_group(
            source=ecef,
            truth_positions=truth_positions,
            source_labels=["theta_b2e_x_rad", "theta_b2e_y_rad", "theta_b2e_z_rad"],
            target_labels=("theta_roll_rad", "theta_pitch_rad", "theta_yaw_rad"),
            target=target,
        ),
    ]
    if not all(groups_added):
        return None
    return target


def aggregate_monte_carlo_group(
    frames: list[pd.DataFrame],
    group: MonteCarloStateGroup,
    max_plot_points: int | None = None,
) -> MonteCarloSeries | None:
    """Aggregate one state group across runs on the first run's time grid."""
    if not frames:
        return None

    reference = frames[0]
    if "time_s" not in reference:
        return None

    for label in group.labels:
        if f"error_{label}" not in reference or f"sigma_{label}" not in reference:
            return None

    time_s = reference["time_s"].to_numpy()
    if max_plot_points is not None and len(time_s) > max_plot_points:
        indices = np.unique(np.linspace(0, len(time_s) - 1, max_plot_points, dtype=int))
        time_s = time_s[indices]
    error_runs = []
    sigma_runs = []

    for frame in frames:
        if "time_s" not in frame:
            return None
        run_time_s = frame["time_s"].to_numpy()
        run_errors = []
        run_sigmas = []
        for label in group.labels:
            error_col = f"error_{label}"
            sigma_col = f"sigma_{label}"
            if error_col not in frame or sigma_col not in frame:
                return None
            run_errors.append(np.interp(time_s, run_time_s, frame[error_col].to_numpy()))
            run_sigmas.append(np.interp(time_s, run_time_s, frame[sigma_col].to_numpy()))
        error_runs.append(np.column_stack(run_errors))
        sigma_runs.append(np.column_stack(run_sigmas))

    error_array = np.stack(error_runs, axis=0)
    sigma_array = np.stack(sigma_runs, axis=0)
    empirical_sigma = (
        np.std(error_array, axis=0, ddof=1)
        if error_array.shape[0] > 1
        else np.zeros_like(error_array[0])
    )
    mean_filter_sigma = np.sqrt(np.mean(np.square(sigma_array), axis=0))
    scale = state_error_scale(group.labels[0])

    return MonteCarloSeries(
        time_s=time_s,
        labels=group.labels,
        axis_names=group.axis_names,
        run_errors=error_array,
        mean_error=np.mean(error_array, axis=0),
        empirical_sigma=empirical_sigma,
        mean_filter_sigma=mean_filter_sigma,
        scale=scale,
        title=group.title,
        ylabel=group.ylabel,
        output_name=group.output_name,
        run_count=error_array.shape[0],
    )


def aggregate_monte_carlo_series(
    runs: list[MonteCarloRunFrames], max_plot_points: int | None = None
) -> list[MonteCarloSeries]:
    """Build all first-pass Monte Carlo error/covariance aggregate series."""
    ecef_frames = [run.ecef for run in runs]
    ned_frames = [run.ned for run in runs if run.ned is not None]

    series: list[MonteCarloSeries] = []
    for group in ECEF_GROUPS:
        aggregate = aggregate_monte_carlo_group(ecef_frames, group, max_plot_points)
        if aggregate is not None:
            series.append(aggregate)
    for group in NED_GROUPS:
        aggregate = aggregate_monte_carlo_group(ned_frames, group, max_plot_points)
        if aggregate is not None:
            series.append(aggregate)
    return series


def plot_monte_carlo_series(series: MonteCarloSeries, figures_dir: Path) -> plt.Figure:
    """Plot one Monte Carlo aggregate series with one subplot per axis."""
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(f"{series.title} ({series.run_count} runs)")

    for axis_index, (ax, axis_name) in enumerate(zip(axes, series.axis_names)):
        run_errors = series.scale * series.run_errors[:, :, axis_index]
        mean_error = series.scale * series.mean_error[:, axis_index]
        empirical_bound = 3.0 * series.scale * series.empirical_sigma[:, axis_index]
        filter_bound = 3.0 * series.scale * series.mean_filter_sigma[:, axis_index]

        run_segments = [
            np.column_stack((series.time_s, run_errors[run_index]))
            for run_index in range(run_errors.shape[0])
        ]
        run_collection = LineCollection(
            run_segments,
            colors="tab:red",
            alpha=0.40,
            linewidths=0.9,
            label="individual run error",
        )
        ax.add_collection(run_collection)
        ax.update_datalim(
            np.column_stack((np.tile(series.time_s, run_errors.shape[0]), run_errors.ravel()))
        )
        ax.autoscale_view()
        ax.plot(
            series.time_s,
            mean_error,
            color="tab:blue",
            linewidth=2.0,
            label="ensemble mean error",
        )
        ax.plot(
            series.time_s,
            mean_error + empirical_bound,
            color="tab:blue",
            linestyle="--",
            label=r"empirical $3\sigma$",
        )
        ax.plot(series.time_s, mean_error - empirical_bound, color="tab:blue", linestyle="--")
        ax.plot(
            series.time_s,
            filter_bound,
            color="black",
            linestyle="-",
            label=r"mean filter $3\sigma$",
        )
        ax.plot(series.time_s, -filter_bound, color="black", linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(f"{axis_name.title()} {series.ylabel}")
        handles, labels = ax.get_legend_handles_labels()
        legend_order = [0, 3, 1, 2]
        ax.legend(
            [handles[index] for index in legend_order],
            [labels[index] for index in legend_order],
            loc="upper right",
        )
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")
    save_monte_carlo_figure(fig, figures_dir / series.output_name)
    return fig


def save_monte_carlo_figure(fig: plt.Figure, out: Path) -> Path:
    """Save a Monte Carlo figure with a faster report-oriented PNG path."""
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, bbox_inches=None)
    print(f"Wrote {out}")
    return out


def plot_monte_carlo_aggregates(
    run_dirs: list[Path], figures_dir: Path, max_plot_points: int | None = None
) -> list[plt.Figure]:
    """Generate all first-pass Monte Carlo aggregate covariance figures."""
    apply_style()
    started_s = time.perf_counter()
    runs = load_successful_runs(run_dirs)
    load_elapsed_s = time.perf_counter() - started_s

    aggregate_started_s = time.perf_counter()
    series = aggregate_monte_carlo_series(runs, max_plot_points)
    aggregate_elapsed_s = time.perf_counter() - aggregate_started_s

    plot_started_s = time.perf_counter()
    figures_dir.mkdir(parents=True, exist_ok=True)
    figures = [plot_monte_carlo_series(item, figures_dir) for item in series]
    plot_elapsed_s = time.perf_counter() - plot_started_s
    print(
        "Monte Carlo analysis timing: "
        f"load={load_elapsed_s:.3f} s, "
        f"aggregate={aggregate_elapsed_s:.3f} s, "
        f"plot={plot_elapsed_s:.3f} s"
    )
    return figures
