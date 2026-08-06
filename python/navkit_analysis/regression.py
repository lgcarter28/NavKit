# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Deterministic truth-reconstruction regression contracts and metrics."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp

from navkit_analysis.data import RunData
from navkit_analysis.schema import (
    DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
    validate_schema,
)


TRUTH_RECONSTRUCTION_METRIC_UNITS: dict[str, str] = {
    "position_max_norm_m": "m",
    "position_rms_norm_m": "m",
    "velocity_max_norm_mps": "m/s",
    "velocity_rms_norm_mps": "m/s",
    "attitude_max_norm_rad": "rad",
    "attitude_rms_norm_rad": "rad",
}
SENSOR_UPDATE_LOG_FILES: dict[str, str] = {
    "gnss_position": "gnss_pos_update.csv",
    "gnss_velocity": "gnss_vel_update.csv",
}


@dataclass(frozen=True)
class UpdateCountContract:
    """Allowed logged-update count for one sensor observation family."""

    minimum: int
    maximum: int | None


@dataclass(frozen=True)
class DeterministicRegressionCase:
    """One scenario and its deterministic truth-reconstruction contract."""

    name: str
    scenario: Path
    minimum_duration_s: float
    minimum_sample_count: int
    thresholds: dict[str, float]
    sensor_update_counts: dict[str, UpdateCountContract]


@dataclass(frozen=True)
class DeterministicRegressionSuite:
    """A named collection of deterministic scenario regressions."""

    name: str
    source: Path
    output_root: Path
    build_type: str
    navkit_config: str
    cases: tuple[DeterministicRegressionCase, ...]


def _required_string(value: object, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{path} must be a nonempty string")
    return value


def _required_nonnegative_float(value: object, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a nonnegative number")
    converted = float(value)
    if not np.isfinite(converted) or converted < 0.0:
        raise ValueError(f"{path} must be a finite nonnegative number")
    return converted


def _required_positive_integer(value: object, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{path} must be a positive integer")
    return value


def _required_nonnegative_integer(value: object, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{path} must be a nonnegative integer")
    return value


def load_deterministic_regression_suite(path: Path) -> DeterministicRegressionSuite:
    """Load and validate a deterministic regression-suite JSON document."""
    source = path.resolve()
    document = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("deterministic regression suite root must be an object")
    validate_schema(
        document,
        DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
        str(source),
    )

    suite_name = _required_string(document.get("suite_name"), "suite_name")
    execution = document.get("execution")
    if not isinstance(execution, dict):
        raise ValueError("execution must be an object")
    build_type = _required_string(execution.get("build_type"), "execution.build_type")
    if build_type not in ("Debug", "Release"):
        raise ValueError("execution.build_type must be 'Debug' or 'Release'")
    navkit_config = _required_string(
        execution.get("navkit_config"), "execution.navkit_config"
    )
    output = document.get("output")
    if not isinstance(output, dict):
        raise ValueError("output must be an object")
    output_root = Path(_required_string(output.get("root"), "output.root"))

    cases_value = document.get("cases")
    if not isinstance(cases_value, list) or not cases_value:
        raise ValueError("cases must be a nonempty array")

    cases: list[DeterministicRegressionCase] = []
    case_names: set[str] = set()
    required_thresholds = set(TRUTH_RECONSTRUCTION_METRIC_UNITS)
    for case_index, case_value in enumerate(cases_value):
        case_path = f"cases[{case_index}]"
        if not isinstance(case_value, dict):
            raise ValueError(f"{case_path} must be an object")
        name = _required_string(case_value.get("name"), f"{case_path}.name")
        if name in case_names:
            raise ValueError(f"duplicate deterministic regression case name '{name}'")
        case_names.add(name)

        scenario_value = _required_string(
            case_value.get("scenario"), f"{case_path}.scenario"
        )
        scenario = (source.parent / scenario_value).resolve()
        if not scenario.is_file():
            raise ValueError(f"{case_path}.scenario does not exist: {scenario}")

        thresholds_value = case_value.get("thresholds")
        if not isinstance(thresholds_value, dict):
            raise ValueError(f"{case_path}.thresholds must be an object")
        observed_thresholds = set(thresholds_value)
        missing = sorted(required_thresholds - observed_thresholds)
        unknown = sorted(observed_thresholds - required_thresholds)
        if missing or unknown:
            raise ValueError(
                f"{case_path}.thresholds must contain exactly "
                f"{sorted(required_thresholds)}; missing={missing}, unknown={unknown}"
            )
        thresholds = {
            metric: _required_nonnegative_float(
                thresholds_value[metric], f"{case_path}.thresholds.{metric}"
            )
            for metric in TRUTH_RECONSTRUCTION_METRIC_UNITS
        }
        update_counts_value = case_value.get("sensor_update_counts")
        if not isinstance(update_counts_value, dict):
            raise ValueError(f"{case_path}.sensor_update_counts must be an object")
        observed_sensor_names = set(update_counts_value)
        expected_sensor_names = set(SENSOR_UPDATE_LOG_FILES)
        if observed_sensor_names != expected_sensor_names:
            raise ValueError(
                f"{case_path}.sensor_update_counts must contain exactly "
                f"{sorted(expected_sensor_names)}"
            )
        sensor_update_counts: dict[str, UpdateCountContract] = {}
        for sensor_name in SENSOR_UPDATE_LOG_FILES:
            sensor_path = f"{case_path}.sensor_update_counts.{sensor_name}"
            contract_value = update_counts_value[sensor_name]
            if not isinstance(contract_value, dict):
                raise ValueError(f"{sensor_path} must be an object")
            unknown_contract_fields = set(contract_value) - {"minimum", "maximum"}
            if unknown_contract_fields:
                raise ValueError(
                    f"{sensor_path} has unknown fields {sorted(unknown_contract_fields)}"
                )
            minimum = _required_nonnegative_integer(
                contract_value.get("minimum"), f"{sensor_path}.minimum"
            )
            maximum_value = contract_value.get("maximum")
            maximum = (
                None
                if maximum_value is None
                else _required_nonnegative_integer(maximum_value, f"{sensor_path}.maximum")
            )
            if maximum is not None and maximum < minimum:
                raise ValueError(f"{sensor_path}.maximum must be at least minimum")
            sensor_update_counts[sensor_name] = UpdateCountContract(minimum, maximum)

        cases.append(
            DeterministicRegressionCase(
                name=name,
                scenario=scenario,
                minimum_duration_s=_required_nonnegative_float(
                    case_value.get("minimum_duration_s"),
                    f"{case_path}.minimum_duration_s",
                ),
                minimum_sample_count=_required_positive_integer(
                    case_value.get("minimum_sample_count"),
                    f"{case_path}.minimum_sample_count",
                ),
                thresholds=thresholds,
                sensor_update_counts=sensor_update_counts,
            )
        )

    return DeterministicRegressionSuite(
        name=suite_name,
        source=source,
        output_root=output_root,
        build_type=build_type,
        navkit_config=navkit_config,
        cases=tuple(cases),
    )


def _vector_error_metrics(values: np.ndarray, quantity: str, units: str) -> dict[str, float]:
    norms = np.linalg.norm(values, axis=1)
    return {
        f"{quantity}_max_norm_{units}": float(np.max(norms)),
        f"{quantity}_rms_norm_{units}": float(np.sqrt(np.mean(np.square(norms)))),
    }


def _interpolate_columns(
    source_time_s: np.ndarray,
    source_values: np.ndarray,
    target_time_s: np.ndarray,
) -> np.ndarray:
    return np.column_stack(
        [
            np.interp(target_time_s, source_time_s, source_values[:, column])
            for column in range(source_values.shape[1])
        ]
    )


def _interpolate_truth_attitude(
    truth_time_s: np.ndarray,
    truth_q_wxyz: np.ndarray,
    nav_time_s: np.ndarray,
) -> Rotation:
    truth_q_xyzw = truth_q_wxyz[:, [1, 2, 3, 0]]
    rotations = Rotation.from_quat(truth_q_xyzw)
    if len(truth_time_s) == 1:
        if len(nav_time_s) != 1 or nav_time_s[0] != truth_time_s[0]:
            raise ValueError("one-sample truth attitude can only serve the same single epoch")
        return rotations
    return Slerp(truth_time_s, rotations)(nav_time_s)


def truth_reconstruction_metrics(run: RunData) -> dict[str, float | int]:
    """Calculate strictly time-aligned PVA truth-reconstruction metrics."""
    if run.truth is None:
        raise ValueError("truth reconstruction requires truth and navigation estimate logs")
    if run.nav.empty or run.truth.empty:
        raise ValueError("truth reconstruction logs must contain at least one sample")

    nav_time_s = run.nav["time_s"].to_numpy(dtype=float)
    truth_time_s = run.truth["time_s"].to_numpy(dtype=float)
    if not np.isfinite(nav_time_s).all() or not np.isfinite(truth_time_s).all():
        raise ValueError("truth reconstruction timestamps must be finite")
    if len(nav_time_s) > 1 and np.any(np.diff(nav_time_s) <= 0.0):
        raise ValueError("navigation estimate timestamps must be strictly increasing")
    if len(truth_time_s) > 1 and np.any(np.diff(truth_time_s) <= 0.0):
        raise ValueError("truth timestamps must be strictly increasing")

    time_tolerance_s = 32.0 * np.finfo(float).eps * max(
        1.0, abs(float(truth_time_s[0])), abs(float(truth_time_s[-1]))
    )
    if nav_time_s[0] < truth_time_s[0] - time_tolerance_s:
        raise ValueError("truth does not bracket the first navigation estimate timestamp")
    if nav_time_s[-1] > truth_time_s[-1] + time_tolerance_s:
        raise ValueError("truth does not bracket the final navigation estimate timestamp")
    interpolation_time_s = np.clip(nav_time_s, truth_time_s[0], truth_time_s[-1])

    truth_position = _interpolate_columns(
        truth_time_s,
        run.truth[["p_e_x_m", "p_e_y_m", "p_e_z_m"]].to_numpy(dtype=float),
        interpolation_time_s,
    )
    truth_velocity = _interpolate_columns(
        truth_time_s,
        run.truth[["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"]].to_numpy(dtype=float),
        interpolation_time_s,
    )
    nav_position = run.nav[["p_e_x_m", "p_e_y_m", "p_e_z_m"]].to_numpy(dtype=float)
    nav_velocity = run.nav[["v_e_x_mps", "v_e_y_mps", "v_e_z_mps"]].to_numpy(dtype=float)
    position_error = truth_position - nav_position
    velocity_error = truth_velocity - nav_velocity

    truth_attitude = _interpolate_truth_attitude(
        truth_time_s,
        run.truth[["q_b2e_w", "q_b2e_x", "q_b2e_y", "q_b2e_z"]].to_numpy(
            dtype=float
        ),
        interpolation_time_s,
    )
    nav_q_wxyz = run.nav[
        ["q_b2e_w", "q_b2e_x", "q_b2e_y", "q_b2e_z"]
    ].to_numpy(dtype=float)
    nav_attitude = Rotation.from_quat(nav_q_wxyz[:, [1, 2, 3, 0]])
    attitude_error = (truth_attitude * nav_attitude.inv()).as_rotvec()
    if not all(
        np.isfinite(values).all()
        for values in (position_error, velocity_error, attitude_error)
    ):
        raise ValueError("truth reconstruction errors must be finite")

    metrics: dict[str, float | int] = {
        "sample_count": int(len(run.nav)),
        "duration_s": float(nav_time_s[-1] - nav_time_s[0]),
    }
    metrics.update(_vector_error_metrics(position_error, "position", "m"))
    metrics.update(_vector_error_metrics(velocity_error, "velocity", "mps"))
    metrics.update(_vector_error_metrics(attitude_error, "attitude", "rad"))
    return metrics


def load_truth_reconstruction_metrics(run_dir: Path) -> dict[str, float | int]:
    """Load one run directory and calculate truth-reconstruction metrics."""
    resolved = run_dir.resolve()
    data_dir = resolved / "data" if (resolved / "data").is_dir() else resolved
    nav_path = data_dir / "nav_estimate_ecef.csv"
    truth_path = data_dir / "truth_trajectory_ecef.csv"
    if not nav_path.is_file():
        raise FileNotFoundError(f"missing navigation estimate log: {nav_path}")
    if not truth_path.is_file():
        raise FileNotFoundError(f"missing truth trajectory log: {truth_path}")
    run = RunData(
        run_dir=resolved,
        data_dir=data_dir,
        figures_dir=resolved / "figures",
        nav=pd.read_csv(nav_path),
        truth=pd.read_csv(truth_path),
    )
    metrics = truth_reconstruction_metrics(run)
    for sensor_name, log_file in SENSOR_UPDATE_LOG_FILES.items():
        update_path = data_dir / log_file
        if not update_path.is_file():
            raise FileNotFoundError(f"missing required sensor-update log: {update_path}")
        update_frame = pd.read_csv(update_path)
        required_columns = {"time_s", "accepted"}
        missing_columns = sorted(required_columns - set(update_frame.columns))
        if missing_columns:
            raise ValueError(
                f"sensor-update log {update_path} is missing columns {missing_columns}"
            )
        update_time_s = update_frame["time_s"].to_numpy(dtype=float)
        accepted = update_frame["accepted"].to_numpy(dtype=float)
        if not np.isfinite(update_time_s).all() or not np.isfinite(accepted).all():
            raise ValueError(f"sensor-update evidence must be finite: {update_path}")
        if not np.isin(accepted, (0.0, 1.0)).all():
            raise ValueError(
                f"sensor-update accepted flags must be zero or one: {update_path}"
            )
        accepted_time_s = update_time_s[accepted == 1.0]
        update_count = len(np.unique(accepted_time_s))
        metrics[f"{sensor_name}_update_count"] = update_count
    return metrics


def evaluate_truth_reconstruction(
    metrics: Mapping[str, float | int], case: DeterministicRegressionCase
) -> tuple[bool, dict[str, dict[str, float | bool | str]]]:
    """Evaluate one metric set against a deterministic regression contract."""
    checks: dict[str, dict[str, float | bool | str]] = {}
    passed = True
    for metric, maximum in case.thresholds.items():
        measured = float(metrics[metric])
        metric_passed = measured <= maximum
        passed = passed and metric_passed
        checks[metric] = {
            "measured": measured,
            "maximum": maximum,
            "units": TRUTH_RECONSTRUCTION_METRIC_UNITS[metric],
            "passed": metric_passed,
        }

    duration_s = float(metrics["duration_s"])
    duration_passed = duration_s >= case.minimum_duration_s
    passed = passed and duration_passed
    checks["minimum_duration_s"] = {
        "measured": duration_s,
        "minimum": case.minimum_duration_s,
        "units": "s",
        "passed": duration_passed,
    }

    sample_count = int(metrics["sample_count"])
    sample_count_passed = sample_count >= case.minimum_sample_count
    passed = passed and sample_count_passed
    checks["minimum_sample_count"] = {
        "measured": float(sample_count),
        "minimum": float(case.minimum_sample_count),
        "units": "samples",
        "passed": sample_count_passed,
    }
    for sensor_name, contract in case.sensor_update_counts.items():
        update_count = int(metrics[f"{sensor_name}_update_count"])
        update_count_passed = update_count >= contract.minimum and (
            contract.maximum is None or update_count <= contract.maximum
        )
        passed = passed and update_count_passed
        update_count_check: dict[str, float | bool | str] = {
            "measured": float(update_count),
            "minimum": float(contract.minimum),
            "units": "updates",
            "passed": update_count_passed,
        }
        if contract.maximum is not None:
            update_count_check["maximum"] = float(contract.maximum)
        checks[f"{sensor_name}_update_count"] = update_count_check
    return passed, checks
