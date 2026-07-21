# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class RunData:
    """Container for NavKit run CSV data.

    Optional logs are represented as None so plotting functions can decide
    whether to skip a figure cleanly.
    """

    run_dir: Path
    data_dir: Path
    figures_dir: Path
    nav: pd.DataFrame
    gnss_pos_update: pd.DataFrame | None = None
    gnss_vel_update: pd.DataFrame | None = None
    gnss_position_debug: pd.DataFrame | None = None
    gnss_velocity_debug: pd.DataFrame | None = None
    truth: pd.DataFrame | None = None
    imu: pd.DataFrame | None = None
    imu_debug: pd.DataFrame | None = None
    filter_correction: pd.DataFrame | None = None
    truth_error: pd.DataFrame | None = None


def _read_optional_csv(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        return None
    frame = _read_navkit_csv(path)
    if frame.empty:
        return None
    return frame


def _read_first_existing_csv(paths: list[Path]) -> tuple[pd.DataFrame | None, Path | None]:
    for path in paths:
        if path.exists():
            frame = _read_navkit_csv(path)
            if not frame.empty:
                return frame, path
    return None, None


def _read_navkit_csv(path: Path, usecols: set[str] | None = None) -> pd.DataFrame:
    selected_columns = (lambda column: column in usecols) if usecols is not None else None
    return _coerce_numeric_columns(
        pd.read_csv(
            path,
            na_values=["nan", "-nan", "nan(ind)", "-nan(ind)"],
            usecols=selected_columns,
        )
    )


def read_navkit_csv(path: Path, usecols: set[str] | None = None) -> pd.DataFrame:
    """Read a NavKit CSV log and coerce numeric columns."""
    return _read_navkit_csv(path, usecols)


def _coerce_numeric_columns(frame: pd.DataFrame) -> pd.DataFrame:
    result = frame.copy()
    for column in result.columns:
        converted = pd.to_numeric(result[column], errors="coerce")
        original_non_null = result[column].notna()
        converted_non_null = converted.notna()
        if bool((converted_non_null | ~original_non_null).all()):
            result[column] = converted
    return result


def _resolve_run_dirs(path: Path) -> tuple[Path, Path, Path]:
    resolved = path.resolve()
    if (resolved / "data").is_dir():
        run_dir = resolved
        data_dir = resolved / "data"
        figures_dir = resolved / "figures"
    elif resolved.name == "data":
        data_dir = resolved
        run_dir = resolved.parent
        figures_dir = run_dir / "figures"
    else:
        run_dir = resolved
        data_dir = resolved
        figures_dir = resolved
    figures_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, data_dir, figures_dir


def resolve_run_dirs(path: Path) -> tuple[Path, Path, Path]:
    """Resolve a run path into run, data, and figures directories."""
    return _resolve_run_dirs(path)


def _require_columns(frame: pd.DataFrame, columns: list[str], source: Path) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise KeyError(f"{source} is missing required columns: {missing}")


def require_columns(frame: pd.DataFrame, columns: list[str], source: Path) -> None:
    """Validate required columns for a loaded NavKit log frame."""
    _require_columns(frame, columns, source)


def _normalized_quaternion_wxyz(values: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(values, axis=1)
    norms[norms == 0.0] = 1.0
    normalized = values / norms[:, None]
    negative_scalar = normalized[:, 0] < 0.0
    normalized[negative_scalar] *= -1.0
    return normalized


def _quaternion_conjugate_wxyz(values: np.ndarray) -> np.ndarray:
    result = values.copy()
    result[:, 1:] *= -1.0
    return result


def _quaternion_multiply_wxyz(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    w1 = lhs[:, 0]
    x1 = lhs[:, 1]
    y1 = lhs[:, 2]
    z1 = lhs[:, 3]
    w2 = rhs[:, 0]
    x2 = rhs[:, 1]
    y2 = rhs[:, 2]
    z2 = rhs[:, 3]
    return np.column_stack(
        [
            (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
            (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
            (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
            (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2),
        ]
    )


def _rotvec_from_quaternion_wxyz(values: np.ndarray) -> np.ndarray:
    q = _normalized_quaternion_wxyz(values)
    sin_half_angle = np.linalg.norm(q[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(sin_half_angle, q[:, 0])
    scale = np.empty_like(angle)
    small = sin_half_angle <= 1.0e-15
    scale[small] = 2.0
    scale[~small] = angle[~small] / sin_half_angle[~small]
    return q[:, 1:] * scale[:, None]


def _interp_columns(source: pd.DataFrame, time_s: np.ndarray, columns: list[str]) -> np.ndarray:
    return np.column_stack([np.interp(time_s, source["time_s"], source[column]) for column in columns])


def _truth_attitude_columns(truth: pd.DataFrame) -> list[str]:
    return ["q_b2e_w", "q_b2e_x", "q_b2e_y", "q_b2e_z"]


def _derive_truth_error(nav: pd.DataFrame, truth: pd.DataFrame | None, imu: pd.DataFrame | None) -> pd.DataFrame | None:
    if truth is None:
        return None

    time_s = nav["time_s"].to_numpy()
    derived_columns: dict[str, np.ndarray | pd.Series] = {"time_s": time_s}

    truth_p_e = _interp_columns(truth, time_s, ["p_e_x_m", "p_e_y_m", "p_e_z_m"])
    truth_v_e = _interp_columns(truth, time_s, ["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"])
    nav_p_e = nav[["p_e_x_m", "p_e_y_m", "p_e_z_m"]].to_numpy()
    nav_v_e = nav[["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"]].to_numpy()

    for axis_index, axis_name in enumerate(("x", "y", "z")):
        derived_columns[f"error_p_e_{axis_name}_m"] = (
            nav_p_e[:, axis_index] - truth_p_e[:, axis_index]
        )
        derived_columns[f"error_v_e_{axis_name}_mps"] = (
            nav_v_e[:, axis_index] - truth_v_e[:, axis_index]
        )

    truth_q = _normalized_quaternion_wxyz(_interp_columns(truth, time_s, _truth_attitude_columns(truth)))
    nav_q = _normalized_quaternion_wxyz(
        nav[["q_b2e_w", "q_b2e_x", "q_b2e_y", "q_b2e_z"]].to_numpy()
    )
    q_error = _quaternion_multiply_wxyz(truth_q, _quaternion_conjugate_wxyz(nav_q))
    theta_error = _rotvec_from_quaternion_wxyz(q_error)
    for axis_index, axis_name in enumerate(("x", "y", "z")):
        derived_columns[f"error_theta_b2e_{axis_name}_rad"] = theta_error[:, axis_index]

    if imu is not None:
        gyro_truth = _interp_columns(
            imu,
            time_s,
            [
                "truth_gyro_bias_b_x_radps",
                "truth_gyro_bias_b_y_radps",
                "truth_gyro_bias_b_z_radps",
            ],
        )
        accel_truth = _interp_columns(
            imu,
            time_s,
            [
                "truth_accel_bias_b_x_mps2",
                "truth_accel_bias_b_y_mps2",
                "truth_accel_bias_b_z_mps2",
            ],
        )
    else:
        gyro_truth = np.zeros((len(nav), 3))
        accel_truth = np.zeros((len(nav), 3))

    nav_gyro = nav[
        ["gyro_bias_b_x_radps", "gyro_bias_b_y_radps", "gyro_bias_b_z_radps"]
    ].to_numpy()
    nav_accel = nav[
        ["accel_bias_b_x_mps2", "accel_bias_b_y_mps2", "accel_bias_b_z_mps2"]
    ].to_numpy()
    for axis_index, axis_name in enumerate(("x", "y", "z")):
        derived_columns[f"error_gyro_bias_b_{axis_name}_radps"] = (
            nav_gyro[:, axis_index] - gyro_truth[:, axis_index]
        )
        derived_columns[f"error_accel_bias_b_{axis_name}_mps2"] = (
            nav_accel[:, axis_index] - accel_truth[:, axis_index]
        )

    for column in nav.columns:
        if column.startswith("sigma_") or column.startswith("P_"):
            derived_columns[column] = nav[column]

    return pd.DataFrame(derived_columns)


def derive_truth_error_frame(
    nav: pd.DataFrame, truth: pd.DataFrame | None, imu: pd.DataFrame | None
) -> pd.DataFrame | None:
    """Derive truth-relative state errors from the selected navigation logs."""
    return _derive_truth_error(nav, truth, imu)


def _attach_nav_covariance(
    correction: pd.DataFrame | None, nav: pd.DataFrame
) -> pd.DataFrame | None:
    if correction is None:
        return None
    result = correction.copy()
    covariance_columns = [
        column for column in nav.columns if column.startswith("sigma_") or column.startswith("P_")
    ]
    correction_time_s = pd.to_numeric(result["time_s"], errors="coerce")
    nav_time_s = pd.to_numeric(nav["time_s"], errors="coerce")
    interpolated_columns: dict[str, np.ndarray] = {}
    for column in covariance_columns:
        if column in result:
            continue
        nav_values = pd.to_numeric(nav[column], errors="coerce")
        if nav_values.isna().any() or nav_time_s.isna().any() or correction_time_s.isna().any():
            continue
        interpolated_columns[column] = np.interp(correction_time_s, nav_time_s, nav_values)
    if not interpolated_columns:
        return result
    return pd.concat([result, pd.DataFrame(interpolated_columns)], axis=1)


def load_run(run_dir: Path) -> RunData:
    """Load standard NavKit run logs from a run directory."""
    run_dir, data_dir, figures_dir = _resolve_run_dirs(run_dir)

    nav, nav_path = _read_first_existing_csv(
        [data_dir / "nav_estimate_ecef.csv", run_dir / "nav_estimate_ecef.csv"]
    )
    if nav is None or nav_path is None:
        raise FileNotFoundError(f"missing navigation estimate log in {run_dir}")

    _require_columns(
        nav,
        [
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
            "sigma_p_e_x_m",
            "sigma_p_e_y_m",
            "sigma_p_e_z_m",
        ],
        nav_path,
    )

    gnss_pos_update, gnss_pos_update_path_optional = _read_first_existing_csv(
        [data_dir / "gnss_pos_update.csv", run_dir / "gnss_pos_update.csv"]
    )
    gnss_pos_update_path = gnss_pos_update_path_optional or (data_dir / "gnss_pos_update.csv")
    gnss_vel_update, gnss_vel_update_path_optional = _read_first_existing_csv(
        [data_dir / "gnss_vel_update.csv", run_dir / "gnss_vel_update.csv"]
    )
    gnss_vel_update_path = gnss_vel_update_path_optional or (data_dir / "gnss_vel_update.csv")
    gnss_position_debug, gnss_position_debug_path = _read_first_existing_csv(
        [
            data_dir / "gnss_position_debug_ecef.csv",
            run_dir / "gnss_position_debug_ecef.csv",
        ]
    )
    gnss_velocity_debug, gnss_velocity_debug_path = _read_first_existing_csv(
        [
            data_dir / "gnss_velocity_debug_ecef.csv",
            run_dir / "gnss_velocity_debug_ecef.csv",
        ]
    )
    truth, truth_path = _read_first_existing_csv(
        [data_dir / "truth_trajectory_ecef.csv", data_dir / "truth.csv", run_dir / "truth_trajectory_ecef.csv"]
    )
    imu, imu_path = _read_first_existing_csv(
        [data_dir / "imu_nominal.csv", data_dir / "imu.csv", run_dir / "imu_nominal.csv"]
    )
    imu_debug, imu_debug_path = _read_first_existing_csv(
        [
            data_dir / "imu_debug_body.csv",
            run_dir / "imu_debug_body.csv",
        ]
    )
    filter_correction, filter_correction_path = _read_first_existing_csv(
        [
            data_dir / "filter_correction_ecef.csv",
            data_dir / "filter_correction.csv",
            run_dir / "filter_correction_ecef.csv",
        ]
    )

    if gnss_pos_update is not None:
        _require_columns(
            gnss_pos_update,
            [
                "time_s",
                "nu_p_e_x_m",
                "nu_p_e_y_m",
                "nu_p_e_z_m",
                "sigma_nu_p_e_x_m",
                "sigma_nu_p_e_y_m",
                "sigma_nu_p_e_z_m",
                "nis",
            ],
            gnss_pos_update_path,
        )

    if gnss_vel_update is not None:
        _require_columns(
            gnss_vel_update,
            [
                "time_s",
                "nu_v_e_x_mps",
                "nu_v_e_y_mps",
                "nu_v_e_z_mps",
                "sigma_nu_v_e_x_mps",
                "sigma_nu_v_e_y_mps",
                "sigma_nu_v_e_z_mps",
                "nis",
            ],
            gnss_vel_update_path,
        )

    if gnss_position_debug is not None and gnss_position_debug_path is not None:
        _require_columns(
            gnss_position_debug,
            [
                "time_s",
                "truth_p_e_x_m",
                "truth_p_e_y_m",
                "truth_p_e_z_m",
                "measured_p_e_x_m",
                "measured_p_e_y_m",
                "measured_p_e_z_m",
                "sigma_p_e_x_m",
                "sigma_p_e_y_m",
                "sigma_p_e_z_m",
            ],
            gnss_position_debug_path,
        )

    if gnss_velocity_debug is not None and gnss_velocity_debug_path is not None:
        _require_columns(
            gnss_velocity_debug,
            [
                "time_s",
                "truth_v_e_x_mps",
                "truth_v_e_y_mps",
                "truth_v_e_z_mps",
                "measured_v_e_x_mps",
                "measured_v_e_y_mps",
                "measured_v_e_z_mps",
                "sigma_v_e_x_mps",
                "sigma_v_e_y_mps",
                "sigma_v_e_z_mps",
            ],
            gnss_velocity_debug_path,
        )

    if truth is not None and truth_path is not None:
        _require_columns(
            truth,
            [
                "time_s",
                "p_e_x_m",
                "p_e_y_m",
                "p_e_z_m",
                "v_e_x_mps",
                "v_e_y_mps",
                "v_e_z_mps",
                *_truth_attitude_columns(truth),
            ],
            truth_path,
        )

    if imu is not None and imu_path is not None:
        _require_columns(
            imu,
            [
                "time_s",
                "truth_delta_theta_ib_b_x_rad",
                "meas_delta_theta_ib_b_x_rad",
                "truth_cumsum_delta_theta_ib_b_x_rad",
                "meas_cumsum_delta_theta_ib_b_x_rad",
                "truth_delta_v_ib_b_x_mps",
                "meas_delta_v_ib_b_x_mps",
                "truth_cumsum_delta_v_ib_b_x_mps",
                "meas_cumsum_delta_v_ib_b_x_mps",
            ],
            imu_path,
        )

    if imu_debug is not None and imu_debug_path is not None:
        _require_columns(
            imu_debug,
            [
                "time_s",
                "omega_ib_b_x_radps",
                "specific_force_ib_b_x_mps2",
                "truth_delta_theta_ib_b_x_rad",
                "meas_delta_theta_ib_b_x_rad",
                "truth_delta_v_ib_b_x_mps",
                "meas_delta_v_ib_b_x_mps",
                "truth_gyro_bias_b_x_radps",
                "truth_accel_bias_b_x_mps2",
            ],
            imu_debug_path,
        )

    truth_error = _derive_truth_error(nav, truth, imu)
    filter_correction = _attach_nav_covariance(filter_correction, nav)

    return RunData(
        run_dir=run_dir,
        data_dir=data_dir,
        figures_dir=figures_dir,
        nav=nav,
        gnss_pos_update=gnss_pos_update,
        gnss_vel_update=gnss_vel_update,
        gnss_position_debug=gnss_position_debug,
        gnss_velocity_debug=gnss_velocity_debug,
        truth=truth,
        imu=imu,
        imu_debug=imu_debug,
        filter_correction=filter_correction,
        truth_error=truth_error,
    )
