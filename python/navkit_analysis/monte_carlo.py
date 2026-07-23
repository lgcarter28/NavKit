# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from navkit_analysis.data import (
    derive_truth_error_frame,
    read_navkit_csv,
    require_columns,
    resolve_run_dirs,
)
from navkit_analysis.plot_spec import PlotAxis, PlotSpec, PlotTrace
from navkit_analysis.renderers import render_matplotlib, render_plotly
from navkit_analysis.figures.state_errors import state_error_scale
from navkit_analysis.schema import MONTE_CARLO_REPORT_SCHEMA
from navkit_analysis.statistics import chi_square_threshold
from navkit_analysis.style import apply_style

REPORT_SCHEMA = MONTE_CARLO_REPORT_SCHEMA


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

    run_dir: Path
    ecef: pd.DataFrame
    ned: pd.DataFrame | None


@dataclass(frozen=True)
class MonteCarloOutputSummary:
    """Summary of generated Monte Carlo campaign-level analysis artifacts."""

    figures: list[object]
    report_paths: dict[str, Path]
    load_elapsed_s: float
    aggregate_elapsed_s: float
    plot_elapsed_s: float
    report_elapsed_s: float


def _quantity_name_from_output(output_name: str) -> str:
    name = output_name.removeprefix("monte_carlo_error_covariance_")
    return name.removesuffix(".png")


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
        ylabel="Gyro Bias [deg/hr]",
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


def monte_carlo_plot_names() -> list[str]:
    """Return supported Monte Carlo aggregate plot selector names."""
    groups = (*ECEF_GROUPS, *NED_GROUPS)
    return sorted(_quantity_name_from_output(group.output_name) for group in groups)


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
    for group in ECEF_GROUPS:
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

    return MonteCarloRunFrames(
        run_dir=run_dir,
        ecef=ecef,
        ned=build_monte_carlo_ned_frame(ecef, truth),
    )


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
    start_time_s: float | None = None,
    end_time_s: float | None = None,
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
    if start_time_s is not None:
        time_s = time_s[time_s >= start_time_s]
    if end_time_s is not None:
        time_s = time_s[time_s <= end_time_s]
    if len(time_s) == 0:
        return None
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
    runs: list[MonteCarloRunFrames],
    max_plot_points: int | None = None,
    selected: list[str] | None = None,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
) -> list[MonteCarloSeries]:
    """Build all first-pass Monte Carlo error/covariance aggregate series."""
    ecef_frames = [run.ecef for run in runs]
    ned_frames = [run.ned for run in runs if run.ned is not None]
    selected_names = set(selected or [])

    series: list[MonteCarloSeries] = []
    for group in ECEF_GROUPS:
        if selected_names and _quantity_name_from_output(group.output_name) not in selected_names:
            continue
        aggregate = aggregate_monte_carlo_group(
            ecef_frames, group, max_plot_points, start_time_s, end_time_s
        )
        if aggregate is not None:
            series.append(aggregate)
    for group in NED_GROUPS:
        if selected_names and _quantity_name_from_output(group.output_name) not in selected_names:
            continue
        aggregate = aggregate_monte_carlo_group(
            ned_frames, group, max_plot_points, start_time_s, end_time_s
        )
        if aggregate is not None:
            series.append(aggregate)
    return series


def build_monte_carlo_plot_spec(series: MonteCarloSeries) -> PlotSpec:
    """Prepare one Monte Carlo series once for static or interactive rendering."""
    axes: list[PlotAxis] = []
    for axis_index, axis_name in enumerate(series.axis_names):
        run_errors = series.scale * series.run_errors[:, :, axis_index]
        mean_error = series.scale * series.mean_error[:, axis_index]
        empirical_bound = 3.0 * series.scale * series.empirical_sigma[:, axis_index]
        filter_bound = 3.0 * series.scale * series.mean_filter_sigma[:, axis_index]
        traces = (
            PlotTrace(
                x=series.time_s,
                y=run_errors,
                label="individual run error",
                color="tab:red",
                line_width=0.9,
                opacity=0.40,
            ),
            PlotTrace(
                x=series.time_s,
                y=np.vstack((filter_bound, -filter_bound)),
                label="mean filter ±3σ",
                color="black",
                line_width=1.5,
            ),
            PlotTrace(
                x=series.time_s,
                y=mean_error,
                label="ensemble mean error",
                color="tab:blue",
                line_width=2.0,
            ),
            PlotTrace(
                x=series.time_s,
                y=np.vstack((mean_error + empirical_bound, mean_error - empirical_bound)),
                label="empirical ±3σ",
                color="tab:blue",
                line_style="dash",
                line_width=1.8,
            ),
        )
        axes.append(
            PlotAxis(
                title="",
                y_label=f"{axis_name.title()} {series.ylabel}",
                traces=traces,
            )
        )
    return PlotSpec(
        title=f"{series.title} ({series.run_count} runs)",
        x_label="Time [s]",
        axes=tuple(axes),
        output_name=series.output_name,
        metadata={
            "run_count": series.run_count,
            "quantity_labels": series.labels,
            "axis_names": series.axis_names,
        },
    )


def plot_monte_carlo_series(series: MonteCarloSeries, figures_dir: Path) -> plt.Figure:
    """Render one aggregate series through the shared static plot-spec backend."""
    spec = build_monte_carlo_plot_spec(series)
    return render_matplotlib(spec, figures_dir / spec.output_name)


def plot_monte_carlo_series_interactive(series: MonteCarloSeries, output_path: Path):
    """Render one cached aggregate series as an interactive Plotly HTML document."""
    return render_plotly(build_monte_carlo_plot_spec(series), output_path)


def _quantity_name(series: MonteCarloSeries) -> str:
    return _quantity_name_from_output(series.output_name)


def _safe_mean(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.mean(finite))


def _safe_percentile(values: np.ndarray, percentile: float) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.percentile(finite, percentile))


def _safe_max(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.max(finite))


def _safe_ratio(numerator: np.ndarray, denominator: np.ndarray) -> np.ndarray:
    ratio = np.full_like(numerator, np.nan)
    nonzero = denominator > 0.0
    ratio[nonzero] = numerator[nonzero] / denominator[nonzero]
    return ratio


def _summary_stats(values: np.ndarray) -> dict[str, float | int | None]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {
            "count": 0,
            "mean": None,
            "min": None,
            "max": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "sum": None,
        }
    return {
        "count": int(finite.size),
        "mean": float(np.mean(finite)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
        "p50": float(np.percentile(finite, 50.0)),
        "p95": float(np.percentile(finite, 95.0)),
        "p99": float(np.percentile(finite, 99.0)),
        "sum": float(np.sum(finite)),
    }


def _state_axis_metric_rows(series_items: list[MonteCarloSeries]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for series in series_items:
        for axis_index, axis_name in enumerate(series.axis_names):
            run_errors = series.scale * series.run_errors[:, :, axis_index]
            mean_error = series.scale * series.mean_error[:, axis_index]
            empirical_sigma = series.scale * series.empirical_sigma[:, axis_index]
            filter_sigma = series.scale * series.mean_filter_sigma[:, axis_index]
            filter_bound = 3.0 * filter_sigma
            empirical_bound = 3.0 * empirical_sigma
            sigma_ratio = _safe_ratio(empirical_sigma, filter_sigma)
            within_filter = np.abs(run_errors) <= filter_bound
            within_empirical = (
                np.abs(run_errors - mean_error[None, :]) <= empirical_bound[None, :]
            )
            rows.append(
                {
                    "quantity": _quantity_name(series),
                    "axis": axis_name,
                    "unit": series.ylabel,
                    "run_count": series.run_count,
                    "sample_count": int(run_errors.shape[1]),
                    "rmse": float(np.sqrt(np.mean(np.square(run_errors)))),
                    "final_rmse": float(np.sqrt(np.mean(np.square(run_errors[:, -1])))),
                    "final_mean_error": float(mean_error[-1]),
                    "max_abs_mean_error": _safe_max(np.abs(mean_error)),
                    "final_empirical_3sigma": float(empirical_bound[-1]),
                    "final_mean_filter_3sigma": float(filter_bound[-1]),
                    "mean_empirical_to_filter_sigma_ratio": _safe_mean(sigma_ratio),
                    "filter_3sigma_coverage": float(np.mean(within_filter)),
                    "empirical_3sigma_coverage": float(np.mean(within_empirical)),
                }
            )
    return rows


def _group_nees_from_frame(frame: pd.DataFrame, labels: tuple[str, str, str]) -> np.ndarray | None:
    covariance = _covariance_series(frame, list(labels))
    if covariance is None:
        return None
    error_columns = [f"error_{label}" for label in labels]
    if any(column not in frame for column in error_columns):
        return None
    errors = frame[error_columns].to_numpy()
    try:
        solved = np.linalg.solve(covariance, errors[:, :, None])[:, :, 0]
    except np.linalg.LinAlgError:
        return None
    return np.sum(errors * solved, axis=1)


def _state_group_metric_rows(
    runs: list[MonteCarloRunFrames], series_items: list[MonteCarloSeries]
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    ecef_frames = [run.ecef for run in runs]
    series_by_labels = {series.labels: series for series in series_items}
    for group in ECEF_GROUPS:
        nees_runs = [
            nees
            for frame in ecef_frames
            if (nees := _group_nees_from_frame(frame, group.labels)) is not None
        ]
        if not nees_runs:
            continue
        nees_array = np.concatenate(nees_runs)
        series = series_by_labels.get(group.labels)
        dof = 3
        final_sigma_ratio = (
            _safe_mean(_safe_ratio(series.empirical_sigma[-1], series.mean_filter_sigma[-1]))
            if series is not None
            else None
        )
        rows.append(
            {
                "quantity": group.output_name.removeprefix(
                    "monte_carlo_error_covariance_"
                ).removesuffix(".png"),
                "frame": "ecef" if not group.output_name.endswith("_body.png") else "body",
                "dof": dof,
                "run_count": len(nees_runs),
                "sample_count": int(nees_array.size),
                "mean_nees": _safe_mean(nees_array),
                "normalized_mean_nees": _safe_mean(nees_array / dof),
                "p95_nees": _safe_percentile(nees_array, 95.0),
                "p99_nees": _safe_percentile(nees_array, 99.0),
                "chi2_95_coverage": float(
                    np.mean(nees_array <= chi_square_threshold(0.95, dof))
                ),
                "chi2_99_coverage": float(
                    np.mean(nees_array <= chi_square_threshold(0.99, dof))
                ),
                "final_mean_empirical_to_filter_sigma_ratio": final_sigma_ratio,
            }
        )
    return rows


def _read_json(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _file_size_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def _run_timing_rows(run_dirs: list[Path]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run_dir in run_dirs:
        run_manifest = _read_json(run_dir / "run_manifest.json") or {}
        timing = _read_json(run_dir / "data" / "timing.json") or {}
        commands = timing.get("commands", {})
        simulation_elapsed_s = None
        if isinstance(commands, dict):
            command = commands.get("stationary_simulation")
            if isinstance(command, dict):
                simulation_elapsed_s = command.get("elapsed_s")
        rows.append(
            {
                "run_id": run_dir.name,
                "status": run_manifest.get("status"),
                "return_code": run_manifest.get("return_code"),
                "runner_elapsed_s": run_manifest.get("elapsed_s"),
                "simulation_elapsed_s": simulation_elapsed_s,
                "output_bytes": _file_size_bytes(run_dir),
                "data_bytes": _file_size_bytes(run_dir / "data"),
            }
        )
    return rows


def _output_size_rows(run_dirs: list[Path]) -> list[dict[str, object]]:
    totals: dict[str, int] = {}
    for run_dir in run_dirs:
        data_dir = run_dir / "data"
        if not data_dir.exists():
            continue
        for item in data_dir.iterdir():
            if item.is_file():
                totals[item.name] = totals.get(item.name, 0) + item.stat().st_size
    return [
        {
            "file_name": file_name,
            "total_bytes": total_bytes,
            "mean_bytes_per_run": total_bytes / max(len(run_dirs), 1),
        }
        for file_name, total_bytes in sorted(totals.items())
    ]


def _nis_metric_rows(run_dirs: list[Path]) -> list[dict[str, object]]:
    specs = [
        ("gnss_position", "gnss_pos_update.csv", 3),
        ("gnss_velocity", "gnss_vel_update.csv", 3),
    ]
    rows: list[dict[str, object]] = []
    for product, file_name, dof in specs:
        nis_values: list[np.ndarray] = []
        accepted_values: list[np.ndarray] = []
        populated_runs = 0
        for run_dir in run_dirs:
            path = run_dir / "data" / file_name
            if not path.exists():
                continue
            frame = read_navkit_csv(path, {"accepted", "nis"})
            if frame.empty or "nis" not in frame:
                continue
            populated_runs += 1
            nis_values.append(frame["nis"].to_numpy())
            if "accepted" in frame:
                accepted_values.append(frame["accepted"].to_numpy())
        if nis_values:
            nis = np.concatenate(nis_values)
            accepted = (
                np.concatenate(accepted_values).astype(bool)
                if accepted_values
                else np.ones_like(nis, dtype=bool)
            )
            rows.append(
                {
                    "product": product,
                    "dof": dof,
                    "run_count": populated_runs,
                    "sample_count": int(nis.size),
                    "accepted_count": int(np.sum(accepted)),
                    "accepted_fraction": float(np.mean(accepted)),
                    "mean_nis": _safe_mean(nis),
                    "normalized_mean_nis": _safe_mean(nis / dof),
                    "p95_nis": _safe_percentile(nis, 95.0),
                    "p99_nis": _safe_percentile(nis, 99.0),
                    "chi2_95_coverage": float(
                        np.mean(nis <= chi_square_threshold(0.95, dof))
                    ),
                    "chi2_99_coverage": float(
                        np.mean(nis <= chi_square_threshold(0.99, dof))
                    ),
                }
            )
        else:
            rows.append(
                {
                    "product": product,
                    "dof": dof,
                    "run_count": 0,
                    "sample_count": 0,
                    "accepted_count": 0,
                    "accepted_fraction": None,
                    "mean_nis": None,
                    "normalized_mean_nis": None,
                    "p95_nis": None,
                    "p99_nis": None,
                    "chi2_95_coverage": None,
                    "chi2_99_coverage": None,
                }
            )
    return rows


def _write_csv(path: Path, rows: list[dict[str, object]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return path


def _markdown_optional_float(value: object, precision: int = 6) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{precision}g}"
    return str(value)


def write_monte_carlo_reports(
    run_dirs: list[Path],
    runs: list[MonteCarloRunFrames],
    series_items: list[MonteCarloSeries],
    reports_dir: Path,
    campaign_metadata: dict[str, object] | None = None,
) -> dict[str, Path]:
    """Write first-pass Monte Carlo aggregate metrics and report artifacts."""
    reports_dir.mkdir(parents=True, exist_ok=True)
    metadata = campaign_metadata or {}
    state_axis_metrics = _state_axis_metric_rows(series_items)
    state_group_metrics = _state_group_metric_rows(runs, series_items)
    nis_metrics = _nis_metric_rows(run_dirs)
    run_timing = _run_timing_rows(run_dirs)
    output_sizes = _output_size_rows(run_dirs)

    runner_elapsed = np.array(
        [
            row["runner_elapsed_s"]
            for row in run_timing
            if isinstance(row["runner_elapsed_s"], (int, float))
        ],
        dtype=float,
    )
    simulation_elapsed = np.array(
        [
            row["simulation_elapsed_s"]
            for row in run_timing
            if isinstance(row["simulation_elapsed_s"], (int, float))
        ],
        dtype=float,
    )
    output_bytes = np.array([row["output_bytes"] for row in run_timing], dtype=float)
    data_bytes = np.array([row["data_bytes"] for row in run_timing], dtype=float)

    report: dict[str, object] = {
        "schema": REPORT_SCHEMA,
        "campaign_name": metadata.get("campaign_name"),
        "run_count": metadata.get("run_count", len(run_dirs)),
        "passed_count": metadata.get("passed_count", len(run_dirs)),
        "failed_count": metadata.get("failed_count", 0),
        "successful_aggregate_count": len(run_dirs),
        "state_axis_metrics": state_axis_metrics,
        "state_group_metrics": state_group_metrics,
        "nis_metrics": nis_metrics,
        "timing_summary": {
            "runner_elapsed_s": _summary_stats(runner_elapsed),
            "simulation_elapsed_s": _summary_stats(simulation_elapsed),
        },
        "output_size_summary": {
            "output_bytes": _summary_stats(output_bytes),
            "data_bytes": _summary_stats(data_bytes),
        },
    }

    json_path = reports_dir / "monte_carlo_summary.json"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    paths = {
        "summary_json": json_path,
        "state_axis_metrics_csv": _write_csv(
            reports_dir / "state_axis_metrics.csv", state_axis_metrics
        ),
        "state_group_metrics_csv": _write_csv(
            reports_dir / "state_group_metrics.csv", state_group_metrics
        ),
        "nis_metrics_csv": _write_csv(reports_dir / "nis_metrics.csv", nis_metrics),
        "run_timing_csv": _write_csv(reports_dir / "run_timing.csv", run_timing),
        "output_sizes_csv": _write_csv(reports_dir / "output_sizes.csv", output_sizes),
    }

    # Keep the Markdown writer deliberately simple and stable; the JSON/CSV files
    # are the machine-readable qualification artifacts.
    report_path = reports_dir / "monte_carlo_report.md"
    lines = [
        "# Monte Carlo Aggregate Report",
        "",
        f"- Runs: {report['run_count']}",
        f"- Passed: {report['passed_count']}",
        f"- Failed: {report['failed_count']}",
        "- Mean runner elapsed: "
        f"{_markdown_optional_float(report['timing_summary']['runner_elapsed_s']['mean'], 3)} s",
        "- Total output: "
        f"{_markdown_optional_float(report['output_size_summary']['output_bytes']['sum'], 3)} bytes",
        "",
        "## State axis metrics",
        "",
        "| Quantity | Axis | RMSE | Final RMSE | Final mean | Final empirical 3 sigma | Final filter 3 sigma | Filter 3 sigma coverage |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in state_axis_metrics:
        lines.append(
            "| "
            f"{row['quantity']} | {row['axis']} | "
            f"{_markdown_optional_float(row['rmse'])} | "
            f"{_markdown_optional_float(row['final_rmse'])} | "
            f"{_markdown_optional_float(row['final_mean_error'])} | "
            f"{_markdown_optional_float(row['final_empirical_3sigma'])} | "
            f"{_markdown_optional_float(row['final_mean_filter_3sigma'])} | "
            f"{_markdown_optional_float(row['filter_3sigma_coverage'], 4)} |"
        )
    lines.extend(
        [
            "",
            "## NIS metrics",
            "",
            "| Product | Samples | Mean NIS | Normalized mean NIS | chi2 95% coverage | chi2 99% coverage |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in nis_metrics:
        lines.append(
            "| "
            f"{row['product']} | {row['sample_count']} | "
            f"{_markdown_optional_float(row['mean_nis'])} | "
            f"{_markdown_optional_float(row['normalized_mean_nis'])} | "
            f"{_markdown_optional_float(row['chi2_95_coverage'], 4)} | "
            f"{_markdown_optional_float(row['chi2_99_coverage'], 4)} |"
        )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    paths["markdown_report"] = report_path
    return paths


def generate_monte_carlo_outputs(
    run_dirs: list[Path],
    summary_dir: Path,
    max_plot_points: int | None = None,
    campaign_metadata: dict[str, object] | None = None,
    selected: list[str] | None = None,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
    renderer: str = "matplotlib",
) -> MonteCarloOutputSummary:
    """Generate campaign-level Monte Carlo figures and reports from run logs."""
    apply_style()
    started_s = time.perf_counter()
    runs = load_successful_runs(run_dirs)
    load_elapsed_s = time.perf_counter() - started_s

    aggregate_started_s = time.perf_counter()
    series = aggregate_monte_carlo_series(
        runs, max_plot_points, selected, start_time_s, end_time_s
    )
    aggregate_elapsed_s = time.perf_counter() - aggregate_started_s

    plot_started_s = time.perf_counter()
    if renderer == "matplotlib":
        figures_dir = summary_dir / "figures"
        figures_dir.mkdir(parents=True, exist_ok=True)
        figures = [plot_monte_carlo_series(item, figures_dir) for item in series]
    elif renderer == "plotly":
        figures_dir = summary_dir / "interactive_figures"
        figures_dir.mkdir(parents=True, exist_ok=True)
        figures = [
            plot_monte_carlo_series_interactive(
                item,
                (
                    figures_dir
                    / item.output_name.removesuffix(".png").replace("monte_carlo_", "")
                ).with_suffix(".html"),
            )
            for item in series
        ]
    else:
        raise ValueError(f"unsupported Monte Carlo renderer '{renderer}'")
    plot_elapsed_s = time.perf_counter() - plot_started_s

    report_started_s = time.perf_counter()
    report_paths = write_monte_carlo_reports(
        run_dirs, runs, series, summary_dir / "reports", campaign_metadata
    )
    report_elapsed_s = time.perf_counter() - report_started_s

    print(
        "Monte Carlo analysis timing: "
        f"load={load_elapsed_s:.3f} s, "
        f"aggregate={aggregate_elapsed_s:.3f} s, "
        f"plot={plot_elapsed_s:.3f} s, "
        f"report={report_elapsed_s:.3f} s"
    )
    return MonteCarloOutputSummary(
        figures=figures,
        report_paths=report_paths,
        load_elapsed_s=load_elapsed_s,
        aggregate_elapsed_s=aggregate_elapsed_s,
        plot_elapsed_s=plot_elapsed_s,
        report_elapsed_s=report_elapsed_s,
    )


def plot_monte_carlo_aggregates(
    run_dirs: list[Path],
    figures_dir: Path,
    max_plot_points: int | None = None,
    selected: list[str] | None = None,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
) -> list[plt.Figure]:
    """Generate all first-pass Monte Carlo aggregate covariance figures."""
    apply_style()
    started_s = time.perf_counter()
    runs = load_successful_runs(run_dirs)
    load_elapsed_s = time.perf_counter() - started_s

    aggregate_started_s = time.perf_counter()
    series = aggregate_monte_carlo_series(
        runs, max_plot_points, selected, start_time_s, end_time_s
    )
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
