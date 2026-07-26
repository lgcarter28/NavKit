# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Monte Carlo NEES/NIS consistency data, cache, and report helpers."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence

import h5py
import numpy as np
import pandas as pd
from scipy.stats import chi2

from navkit_analysis.analysis_performance import StageTimer
from navkit_analysis.schema import ANALYSIS_BUNDLE_SCHEMA, validate_schema


DEFAULT_CONFIDENCE = 0.95
SIGMA_COVERAGE_LEVELS = (
    ("1sigma", 0.682689492137086),
    ("2sigma", 0.954499736103642),
    ("3sigma", 0.997300203936740),
)
CONSISTENCY_SERIES_KINDS = ("nees", "nis", "marginal")


@dataclass(frozen=True)
class ConsistencyGroup:
    """One joint error-state or observation family consistency definition."""

    name: str
    title: str
    labels: tuple[str, ...]
    source: str

    @property
    def dof(self) -> int:
        """Return the chi-square degrees of freedom for the joint statistic."""
        return len(self.labels)


ERROR_STATE_LABELS = (
    "p_e_x_m",
    "p_e_y_m",
    "p_e_z_m",
    "v_e_x_mps",
    "v_e_y_mps",
    "v_e_z_mps",
    "theta_b2e_x_rad",
    "theta_b2e_y_rad",
    "theta_b2e_z_rad",
    "gyro_bias_b_x_radps",
    "gyro_bias_b_y_radps",
    "gyro_bias_b_z_radps",
    "accel_bias_b_x_mps2",
    "accel_bias_b_y_mps2",
    "accel_bias_b_z_mps2",
)

NEES_GROUPS = (
    ConsistencyGroup(
        "full_ins",
        "Full INS Error-State NEES",
        ERROR_STATE_LABELS,
        "truth_error",
    ),
    ConsistencyGroup(
        "pva",
        "PVA NEES",
        ERROR_STATE_LABELS[:9],
        "truth_error",
    ),
    ConsistencyGroup(
        "position",
        "Position NEES",
        ERROR_STATE_LABELS[:3],
        "truth_error",
    ),
    ConsistencyGroup(
        "velocity",
        "Velocity NEES",
        ERROR_STATE_LABELS[3:6],
        "truth_error",
    ),
    ConsistencyGroup(
        "attitude",
        "Attitude NEES",
        ERROR_STATE_LABELS[6:9],
        "truth_error",
    ),
    ConsistencyGroup(
        "imu_bias",
        "Combined IMU-Bias NEES",
        ERROR_STATE_LABELS[9:],
        "truth_error",
    ),
    ConsistencyGroup(
        "gyro_bias",
        "Gyro-Bias NEES",
        ERROR_STATE_LABELS[9:12],
        "truth_error",
    ),
    ConsistencyGroup(
        "accel_bias",
        "Accelerometer-Bias NEES",
        ERROR_STATE_LABELS[12:],
        "truth_error",
    ),
)

NIS_GROUPS = (
    ConsistencyGroup(
        "gnss_position",
        "GNSS Position NIS",
        ("nu_p_e_x_m", "nu_p_e_y_m", "nu_p_e_z_m"),
        "gnss_pos_update",
    ),
    ConsistencyGroup(
        "gnss_velocity",
        "GNSS Velocity NIS",
        ("nu_v_e_x_mps", "nu_v_e_y_mps", "nu_v_e_z_mps"),
        "gnss_vel_update",
    ),
)


# These are explicitly marginal 1-DOF diagnostics, not substitutes for the
# joint NEES products above. They make each physical body/ECEF error axis easy
# to inspect while the joint products retain the covariance cross terms.
MARGINAL_NSE_GROUPS = (
    (
        "position_axes",
        "Position Marginal Normalized Squared Error",
        ERROR_STATE_LABELS[:3],
    ),
    (
        "velocity_axes",
        "Velocity Marginal Normalized Squared Error",
        ERROR_STATE_LABELS[3:6],
    ),
    (
        "attitude_axes",
        "Attitude Marginal Normalized Squared Error",
        ERROR_STATE_LABELS[6:9],
    ),
    (
        "gyro_bias_axes",
        "Gyro-Bias Marginal Normalized Squared Error",
        ERROR_STATE_LABELS[9:12],
    ),
    (
        "accel_bias_axes",
        "Accelerometer-Bias Marginal Normalized Squared Error",
        ERROR_STATE_LABELS[12:],
    ),
)


@dataclass(frozen=True)
class ConsistencySeries:
    """Time-indexed Monte Carlo samples for one joint NEES or NIS statistic."""

    name: str
    title: str
    kind: str
    labels: tuple[str, ...]
    time_s: np.ndarray
    values: np.ndarray
    accepted: np.ndarray | None = None

    @property
    def dof(self) -> int:
        """Return the chi-square degrees of freedom for this statistic."""
        return len(self.labels)

    @property
    def run_count(self) -> int:
        """Return the number of stored Monte Carlo histories."""
        return self.values.shape[0]


def chi_square_mean_bounds(
    dof: int,
    sample_count: np.ndarray,
    confidence: float = DEFAULT_CONFIDENCE,
) -> tuple[np.ndarray, np.ndarray]:
    """Return confidence bounds for an epoch-wise ensemble mean chi-square statistic."""
    counts = np.asarray(sample_count, dtype=float)
    lower = np.full(counts.shape, np.nan, dtype=float)
    upper = np.full(counts.shape, np.nan, dtype=float)
    valid = counts > 0.0
    total_dof = dof * counts[valid]
    alpha = 1.0 - confidence
    lower[valid] = chi2.ppf(alpha / 2.0, total_dof) / counts[valid]
    upper[valid] = chi2.ppf(1.0 - alpha / 2.0, total_dof) / counts[valid]
    return lower, upper


def _select_time_grid(time_s: np.ndarray, max_points: int | None) -> np.ndarray:
    if max_points is None or len(time_s) <= max_points:
        return np.asarray(time_s, dtype=float)
    indices = np.linspace(0, len(time_s) - 1, max_points, dtype=int)
    return np.asarray(time_s[indices], dtype=float)


def _covariance_from_frame(frame: pd.DataFrame, labels: tuple[str, ...]) -> np.ndarray | None:
    covariance = np.zeros((len(frame), len(labels), len(labels)), dtype=float)
    for row, row_label in enumerate(labels):
        for col, col_label in enumerate(labels[row:], start=row):
            column = f"P_{row_label}__{col_label}"
            reverse_column = f"P_{col_label}__{row_label}"
            if column in frame:
                values = frame[column].to_numpy(dtype=float)
            elif reverse_column in frame:
                values = frame[reverse_column].to_numpy(dtype=float)
            else:
                return None
            covariance[:, row, col] = values
            covariance[:, col, row] = values
    return covariance


def nees_from_frame(frame: pd.DataFrame, labels: tuple[str, ...]) -> np.ndarray | None:
    """Calculate one joint NEES time history without discarding cross-covariance terms."""
    error_columns = [f"error_{label}" for label in labels]
    if any(column not in frame for column in error_columns):
        return None
    covariance = _covariance_from_frame(frame, labels)
    if covariance is None:
        return None
    errors = frame[error_columns].to_numpy(dtype=float)
    finite = np.isfinite(errors).all(axis=1) & np.isfinite(covariance).all(axis=(1, 2))
    values = np.full(len(frame), np.nan, dtype=float)
    if not np.any(finite):
        return values
    try:
        solved = np.linalg.solve(covariance[finite], errors[finite, :, None])[:, :, 0]
        values[finite] = np.einsum("ij,ij->i", errors[finite], solved)
    except np.linalg.LinAlgError:
        for index in np.flatnonzero(finite):
            try:
                values[index] = float(errors[index] @ np.linalg.solve(covariance[index], errors[index]))
            except np.linalg.LinAlgError:
                continue
    return values


def marginal_nse_from_frame(frame: pd.DataFrame, label: str) -> np.ndarray | None:
    """Calculate one scalar normalized squared-error history from a covariance diagonal."""
    error_column = f"error_{label}"
    covariance_column = f"P_{label}__{label}"
    if error_column not in frame or covariance_column not in frame:
        return None
    errors = frame[error_column].to_numpy(dtype=float)
    variance = frame[covariance_column].to_numpy(dtype=float)
    values = np.full(len(frame), np.nan, dtype=float)
    valid = np.isfinite(errors) & np.isfinite(variance) & (variance > 0.0)
    values[valid] = np.square(errors[valid]) / variance[valid]
    return values


def _interpolate_history(time_s: np.ndarray, values: np.ndarray, target_time_s: np.ndarray) -> np.ndarray:
    finite = np.isfinite(values)
    if np.count_nonzero(finite) == 0:
        return np.full(target_time_s.shape, np.nan, dtype=float)
    if np.count_nonzero(finite) == 1:
        result = np.full(target_time_s.shape, np.nan, dtype=float)
        nearest = int(np.argmin(np.abs(target_time_s - time_s[finite][0])))
        result[nearest] = values[finite][0]
        return result
    return np.interp(target_time_s, time_s[finite], values[finite], left=np.nan, right=np.nan)


def build_truth_error_consistency_series(
    frames: Iterable[pd.DataFrame],
    max_points: int | None = None,
    *,
    include_nees: bool = True,
    include_marginal: bool = True,
) -> tuple[list[ConsistencySeries], list[ConsistencySeries]]:
    """Build requested NEES and marginal products in one truth-error frame pass."""
    frame_list = list(frames)
    if not frame_list:
        return [], []
    first_frame = frame_list[0]
    if "time_s" not in first_frame:
        raise ValueError("truth-error frame is missing time_s")
    time_s = _select_time_grid(first_frame["time_s"].to_numpy(dtype=float), max_points)
    nees_histories: dict[str, list[np.ndarray]] = (
        {group.name: [] for group in NEES_GROUPS} if include_nees else {}
    )
    marginal_histories: dict[str, list[np.ndarray]] = (
        {
            f"{group_name}_{axis}": []
            for group_name, _, labels in MARGINAL_NSE_GROUPS
            for axis, _ in enumerate(labels)
        }
        if include_marginal
        else {}
    )
    for frame in frame_list:
        if "time_s" not in frame:
            raise ValueError("truth-error frame is missing time_s")
        source_time_s = frame["time_s"].to_numpy(dtype=float)
        if include_nees:
            for group in NEES_GROUPS:
                values = nees_from_frame(frame, group.labels)
                if values is None:
                    nees_histories[group.name].append(
                        np.full(time_s.shape, np.nan, dtype=float)
                    )
                else:
                    nees_histories[group.name].append(
                        _interpolate_history(source_time_s, values, time_s)
                    )
        if include_marginal:
            for group_name, _, labels in MARGINAL_NSE_GROUPS:
                for axis, label in enumerate(labels):
                    name = f"{group_name}_{axis}"
                    values = marginal_nse_from_frame(frame, label)
                    if values is None:
                        marginal_histories[name].append(
                            np.full(time_s.shape, np.nan, dtype=float)
                        )
                    else:
                        marginal_histories[name].append(
                            _interpolate_history(source_time_s, values, time_s)
                        )

    nees_series: list[ConsistencySeries] = []
    if include_nees:
        for group in NEES_GROUPS:
            values = np.stack(nees_histories[group.name], axis=0)
            if not np.isfinite(values).any():
                continue
            nees_series.append(
                ConsistencySeries(
                    name=group.name,
                    title=group.title,
                    kind="nees",
                    labels=group.labels,
                    time_s=time_s,
                    values=values,
                )
            )

    marginal_series: list[ConsistencySeries] = []
    if include_marginal:
        for group_name, group_title, labels in MARGINAL_NSE_GROUPS:
            for axis, label in enumerate(labels):
                name = f"{group_name}_{axis}"
                values = np.stack(marginal_histories[name], axis=0)
                if not np.isfinite(values).any():
                    continue
                marginal_series.append(
                    ConsistencySeries(
                        name=name,
                        title=f"{group_title}: {'XYZ'[axis]} Axis",
                        kind="marginal_nse",
                        labels=(label,),
                        time_s=time_s,
                        values=values,
                    )
                )
    return nees_series, marginal_series


def build_nees_series(
    frames: Iterable[pd.DataFrame],
    max_points: int | None = None,
) -> list[ConsistencySeries]:
    """Build the full hierarchy of joint NEES samples from truth-error frames."""
    nees_series, _ = build_truth_error_consistency_series(
        frames,
        max_points,
        include_marginal=False,
    )
    return nees_series


def build_marginal_nse_series(
    frames: Iterable[pd.DataFrame],
    max_points: int | None = None,
) -> list[ConsistencySeries]:
    """Build 1-DOF per-axis normalized squared-error drill-down series."""
    _, marginal_series = build_truth_error_consistency_series(
        frames,
        max_points,
        include_nees=False,
    )
    return marginal_series


def _interpolate_nearest(
    time_s: np.ndarray,
    values: np.ndarray,
    target_time_s: np.ndarray,
) -> np.ndarray:
    finite = np.isfinite(values)
    if np.count_nonzero(finite) == 0:
        return np.full(target_time_s.shape, np.nan, dtype=float)
    source_time_s = time_s[finite]
    source_values = values[finite]
    right = np.searchsorted(source_time_s, target_time_s, side="left")
    right = np.clip(right, 0, len(source_time_s) - 1)
    left = np.clip(right - 1, 0, len(source_time_s) - 1)
    choose_left = np.abs(target_time_s - source_time_s[left]) <= np.abs(
        source_time_s[right] - target_time_s
    )
    indices = np.where(choose_left, left, right)
    return source_values[indices]


def build_nis_series(
    update_frames: dict[str, Sequence[pd.DataFrame | None]],
    max_points: int | None = None,
) -> list[ConsistencySeries]:
    """Build per-observation-family NIS histories and update acceptance masks."""
    series: list[ConsistencySeries] = []
    for group in NIS_GROUPS:
        frames = update_frames.get(group.name, [])
        first = next((frame for frame in frames if frame is not None and "time_s" in frame), None)
        if first is None:
            continue
        time_s = _select_time_grid(first["time_s"].to_numpy(dtype=float), max_points)
        histories: list[np.ndarray] = []
        acceptance_histories: list[np.ndarray] = []
        for frame in frames:
            if frame is None or "time_s" not in frame or "nis" not in frame:
                histories.append(np.full(time_s.shape, np.nan, dtype=float))
                acceptance_histories.append(np.full(time_s.shape, np.nan, dtype=float))
                continue
            source_time_s = frame["time_s"].to_numpy(dtype=float)
            nis = pd.to_numeric(frame["nis"], errors="coerce").to_numpy(dtype=float)
            # NIS is defined at an observation epoch, not continuously between
            # observations. Preserve the nearest actual update value rather than
            # manufacturing a linearly interpolated statistic.
            histories.append(_interpolate_nearest(source_time_s, nis, time_s))
            if "accepted" in frame:
                accepted = pd.to_numeric(frame["accepted"], errors="coerce").to_numpy(dtype=float)
                acceptance_histories.append(_interpolate_nearest(source_time_s, accepted, time_s))
            else:
                acceptance_histories.append(np.full(time_s.shape, np.nan, dtype=float))
        values = np.stack(histories, axis=0)
        if not np.isfinite(values).any():
            continue
        series.append(
            ConsistencySeries(
                name=group.name,
                title=group.title,
                kind="nis",
                labels=group.labels,
                time_s=time_s,
                values=values,
                accepted=np.stack(acceptance_histories, axis=0),
            )
        )
    return series


def load_nis_frames_from_run_dirs(run_dirs: Sequence[Path]) -> dict[str, list[pd.DataFrame | None]]:
    """Load the minimal GNSS update columns needed for per-family NIS analysis."""
    frames: dict[str, list[pd.DataFrame | None]] = {group.name: [] for group in NIS_GROUPS}
    for group in NIS_GROUPS:
        csv_name = f"{group.source}.csv"
        for run_dir in run_dirs:
            source = run_dir / "data" / csv_name
            if not source.exists():
                frames[group.name].append(None)
                continue
            frame = pd.read_csv(
                source,
                usecols=lambda column: column in {"time_s", "nis", "accepted"},
            )
            frames[group.name].append(frame if "time_s" in frame and "nis" in frame else None)
    return frames


def _series_dataset_options(
    values: np.ndarray,
    compression: str,
) -> dict[str, object]:
    """Return explicit chunking suited to full-history and epoch-slice reads."""
    if values.ndim == 1:
        chunks: tuple[int, ...] = (min(len(values), 256),)
    elif values.ndim == 2:
        chunks = (min(values.shape[0], 64), min(values.shape[1], 256))
    else:
        raise ValueError("consistency cache values must be one- or two-dimensional")
    options: dict[str, object] = {"chunks": chunks}
    if compression != "none":
        options["compression"] = compression
        options["shuffle"] = True
    return options


def _write_series_group(
    parent: h5py.Group,
    series: ConsistencySeries,
    compression: str,
) -> None:
    group = parent.create_group(series.name)
    group.attrs["title"] = series.title
    group.attrs["kind"] = series.kind
    group.attrs["labels"] = json.dumps(series.labels)
    group.attrs["dof"] = series.dof
    group.create_dataset(
        "time_s",
        data=series.time_s,
        **_series_dataset_options(series.time_s, compression),
    )
    group.create_dataset(
        "values",
        data=series.values,
        **_series_dataset_options(series.values, compression),
    )
    if series.accepted is not None:
        group.create_dataset(
            "accepted",
            data=series.accepted,
            **_series_dataset_options(series.accepted, compression),
        )


def write_consistency_cache(
    aggregate_group: h5py.Group,
    nees_series: Sequence[ConsistencySeries],
    nis_series: Sequence[ConsistencySeries],
    marginal_series: Sequence[ConsistencySeries],
    *,
    selected_kinds: Sequence[str] = CONSISTENCY_SERIES_KINDS,
    compression: str = "lzf",
) -> None:
    """Write selected consistency families without replacing unrelated cached data."""
    invalid_kinds = set(selected_kinds).difference(CONSISTENCY_SERIES_KINDS)
    if invalid_kinds:
        raise ValueError(f"unsupported consistency series kinds: {sorted(invalid_kinds)}")
    if compression not in {"lzf", "gzip", "none"}:
        raise ValueError("consistency cache compression must be 'lzf', 'gzip', or 'none'")
    consistency_group = aggregate_group.require_group("consistency")
    series_group = consistency_group.require_group("series")
    grouped_series = {
        "nees": nees_series,
        "nis": nis_series,
        "marginal": marginal_series,
    }
    for kind in selected_kinds:
        if kind in series_group:
            del series_group[kind]
        destination = series_group.create_group(kind)
        for series in grouped_series[kind]:
            _write_series_group(destination, series, compression)


def _read_frame_columns(group: h5py.Group, columns: Sequence[str]) -> pd.DataFrame:
    missing = [column for column in columns if column not in group]
    if missing:
        raise ValueError(f"bundle frame is missing required columns: {', '.join(missing)}")
    return pd.DataFrame({column: group[column][()] for column in columns}, columns=columns)


def _bundle_storage_compression(bundle: h5py.File) -> str:
    """Read the selected numeric HDF5 compression without importing bundle writers."""
    encoded_metadata = bundle.attrs.get("metadata")
    if isinstance(encoded_metadata, bytes):
        encoded_metadata = encoded_metadata.decode("utf-8")
    if not isinstance(encoded_metadata, str):
        return "lzf"
    try:
        metadata = json.loads(encoded_metadata)
    except json.JSONDecodeError:
        return "lzf"
    storage = metadata.get("storage", {}) if isinstance(metadata, dict) else {}
    compression = storage.get("compression", "lzf") if isinstance(storage, dict) else "lzf"
    return compression if compression in {"lzf", "gzip", "none"} else "lzf"


def _nees_required_columns() -> tuple[str, ...]:
    columns: list[str] = ["time_s"]
    columns.extend(f"error_{label}" for label in ERROR_STATE_LABELS)
    for row, row_label in enumerate(ERROR_STATE_LABELS):
        for col_label in ERROR_STATE_LABELS[row:]:
            columns.append(f"P_{row_label}__{col_label}")
    return tuple(columns)


def _iter_bundle_truth_error_frames(bundle: h5py.File) -> Iterator[pd.DataFrame]:
    runs = bundle.get("runs")
    if not isinstance(runs, h5py.Group):
        raise ValueError("analysis bundle is missing runs")
    columns = _nees_required_columns()
    for run_id in sorted(runs.keys()):
        derived = runs[run_id].get("derived")
        if not isinstance(derived, h5py.Group):
            continue
        truth_error = derived.get("truth_error")
        if not isinstance(truth_error, h5py.Group):
            continue
        yield _read_frame_columns(truth_error, columns)


def _iter_bundle_nis_frames(
    bundle: h5py.File,
    source: str,
) -> Iterator[pd.DataFrame | None]:
    runs = bundle.get("runs")
    if not isinstance(runs, h5py.Group):
        raise ValueError("analysis bundle is missing runs")
    for run_id in sorted(runs.keys()):
        data = runs[run_id].get("data")
        if not isinstance(data, h5py.Group):
            yield None
            continue
        updates = data.get(source)
        if not isinstance(updates, h5py.Group) or "time_s" not in updates or "nis" not in updates:
            yield None
            continue
        columns = ["time_s", "nis"]
        if "accepted" in updates:
            columns.append("accepted")
        yield _read_frame_columns(updates, columns)


def refresh_consistency_cache(
    bundle_path: Path,
    max_points: int | None = None,
    *,
    selected_kinds: Sequence[str] = CONSISTENCY_SERIES_KINDS,
) -> tuple[list[ConsistencySeries], list[ConsistencySeries], list[ConsistencySeries]]:
    """Build requested time-indexed consistency families in one owner process."""
    invalid_kinds = set(selected_kinds).difference(CONSISTENCY_SERIES_KINDS)
    if invalid_kinds:
        raise ValueError(f"unsupported consistency series kinds: {sorted(invalid_kinds)}")
    selected = set(selected_kinds)
    timer = StageTimer()
    nees_series: list[ConsistencySeries] = []
    nis_series: list[ConsistencySeries] = []
    marginal_series: list[ConsistencySeries] = []
    with h5py.File(bundle_path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(bundle_path))
        if {"nees", "marginal"}.intersection(selected):
            truth_error_frames = list(_iter_bundle_truth_error_frames(bundle))
            timer.mark("truth_error_frame_load")
            nees_series, marginal_series = build_truth_error_consistency_series(
                truth_error_frames,
                max_points,
                include_nees="nees" in selected,
                include_marginal="marginal" in selected,
            )
            timer.mark("truth_error_nees_derivation")
        if "nis" in selected:
            nis_series = build_nis_series(
                {
                    group.name: list(_iter_bundle_nis_frames(bundle, group.source))
                    for group in NIS_GROUPS
                },
                max_points,
            )
            timer.mark("nis_frame_load_and_derivation")
    if "truth_error_frame_load" not in timer.snapshot():
        timer.mark("truth_error_frame_load")
    if "truth_error_nees_derivation" not in timer.snapshot():
        timer.mark("truth_error_nees_derivation")
    if "nis_frame_load_and_derivation" not in timer.snapshot():
        timer.mark("nis_frame_load_and_derivation")
    with h5py.File(bundle_path, "r+") as bundle:
        aggregate_group = bundle.require_group("aggregate")
        compression = _bundle_storage_compression(bundle)
        write_consistency_cache(
            aggregate_group,
            nees_series,
            nis_series,
            marginal_series,
            selected_kinds=selected_kinds,
            compression=compression,
        )
        timer.mark("hdf5_cache_write")
        consistency_group = aggregate_group.require_group("consistency")
        consistency_group.attrs["cache_performance"] = json.dumps(
            {
                "selected_kinds": sorted(selected),
                "stages": timer.snapshot(),
            },
            sort_keys=True,
        )
    return nees_series, nis_series, marginal_series


def consistency_cache_performance(bundle_path: Path) -> dict[str, object]:
    """Return persisted named-stage timing from the most recent cache refresh."""
    with h5py.File(bundle_path, "r") as bundle:
        consistency_group = bundle.get("aggregate/consistency")
        if not isinstance(consistency_group, h5py.Group):
            return {}
        encoded = consistency_group.attrs.get("cache_performance")
        if isinstance(encoded, bytes):
            encoded = encoded.decode("utf-8")
        if not isinstance(encoded, str):
            return {}
        try:
            value = json.loads(encoded)
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}


def load_consistency_cache(
    bundle_path: Path,
) -> tuple[list[ConsistencySeries], list[ConsistencySeries], list[ConsistencySeries]]:
    """Load time-indexed NEES/NIS consistency samples from a packaged campaign."""
    nees_series: list[ConsistencySeries] = []
    nis_series: list[ConsistencySeries] = []
    marginal_series: list[ConsistencySeries] = []
    with h5py.File(bundle_path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(bundle_path))
        root = bundle.get("aggregate/consistency/series")
        if not isinstance(root, h5py.Group):
            raise ValueError(
                f"bundle '{bundle_path}' has no time-indexed consistency cache; refresh it first"
            )
        for kind, destination in (
            ("nees", nees_series),
            ("nis", nis_series),
            ("marginal", marginal_series),
        ):
            parent = root.get(kind)
            if not isinstance(parent, h5py.Group):
                continue
            for name in sorted(parent.keys()):
                group = parent[name]
                if not isinstance(group, h5py.Group):
                    continue
                labels = tuple(json.loads(group.attrs["labels"]))
                accepted = group["accepted"][()] if "accepted" in group else None
                destination.append(
                    ConsistencySeries(
                        name=name,
                        title=str(group.attrs["title"]),
                        kind=str(group.attrs["kind"]),
                        labels=labels,
                        time_s=group["time_s"][()],
                        values=group["values"][()],
                        accepted=accepted,
                    )
                )
    return nees_series, nis_series, marginal_series


def series_summary_rows(series_items: Sequence[ConsistencySeries]) -> list[dict[str, object]]:
    """Summarize final and steady-state joint consistency evidence by group."""
    rows: list[dict[str, object]] = []
    for series in series_items:
        start = max(0, int(0.8 * len(series.time_s)))
        for window_name, indices in (
            ("final", np.asarray([len(series.time_s) - 1])),
            ("steady_state", np.arange(start, len(series.time_s))),
        ):
            samples = series.values[:, indices]
            sample_counts = np.sum(np.isfinite(samples), axis=0)
            mean = np.nanmean(samples, axis=0)
            lower, upper = chi_square_mean_bounds(series.dof, sample_counts)
            coverage: dict[str, float] = {}
            for label, probability in SIGMA_COVERAGE_LEVELS:
                threshold = chi2.ppf(probability, series.dof)
                coverage[label] = float(np.nanmean(samples <= threshold))
            row: dict[str, object] = {
                "kind": series.kind,
                "group": series.name,
                "title": series.title,
                "dof": series.dof,
                "window": window_name,
                "mean_value": float(np.nanmean(mean)),
                "normalized_mean": float(np.nanmean(mean) / series.dof),
                "mean_lower_bound": float(np.nanmean(lower)),
                "mean_upper_bound": float(np.nanmean(upper)),
                "mean_within_bounds": bool(np.nanmean(mean) >= np.nanmean(lower) and np.nanmean(mean) <= np.nanmean(upper)),
                "joint_1sigma_coverage": coverage["1sigma"],
                "joint_2sigma_coverage": coverage["2sigma"],
                "joint_3sigma_coverage": coverage["3sigma"],
            }
            if series.accepted is not None:
                row["acceptance_rate"] = float(np.nanmean(series.accepted[:, indices] > 0.5))
            else:
                row["acceptance_rate"] = None
            rows.append(row)
    return rows


def scalar_coverage_rows(bundle_path: Path) -> list[dict[str, object]]:
    """Calculate scalar 1/2/3-sigma coverage from cached aggregate error series."""
    rows: list[dict[str, object]] = []
    with h5py.File(bundle_path, "r") as bundle:
        groups = bundle.get("aggregate/monte_carlo_series")
        if not isinstance(groups, h5py.Group):
            return rows
        for name in sorted(groups.keys()):
            group = groups[name]
            if not isinstance(group, h5py.Group):
                continue
            labels = tuple(json.loads(group.attrs["labels"]))
            time_s = group["time_s"][()]
            errors = group["run_errors"][()]
            sigma = group["mean_filter_sigma"][()]
            start = max(0, int(0.8 * len(time_s)))
            for axis, label in enumerate(labels):
                for window_name, indices in (
                    ("final", np.asarray([len(time_s) - 1])),
                    ("steady_state", np.arange(start, len(time_s))),
                ):
                    error = errors[:, indices, axis]
                    bounds = sigma[indices, axis]
                    row: dict[str, object] = {
                        "quantity": name,
                        "axis": label,
                        "window": window_name,
                    }
                    for coverage_name, probability in SIGMA_COVERAGE_LEVELS:
                        multiplier = float(chi2.ppf(probability, 1) ** 0.5)
                        row[f"{coverage_name}_coverage"] = float(
                            np.nanmean(np.abs(error) <= multiplier * bounds[None, :])
                        )
                    rows.append(row)
    return rows


def write_consistency_reports(
    bundle_path: Path,
    reports_dir: Path,
    series_items: Sequence[ConsistencySeries],
) -> dict[str, Path]:
    """Write compact CSV/JSON/Markdown consistency summaries for one campaign."""
    reports_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = series_summary_rows(series_items)
    scalar_rows = scalar_coverage_rows(bundle_path)
    summary_path = reports_dir / "consistency_summary.json"
    summary_path.write_text(
        json.dumps(
            {"schema": "navkit.consistency_report.v1", "joint_metrics": summary_rows, "scalar_coverage": scalar_rows},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    paths = {"summary_json": summary_path}
    for name, rows in (("joint_consistency_metrics.csv", summary_rows), ("scalar_coverage_metrics.csv", scalar_rows)):
        path = reports_dir / name
        fields = list(rows[0].keys()) if rows else []
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            if fields:
                writer.writeheader()
                writer.writerows(rows)
        paths[name.removesuffix(".csv")] = path

    report_path = reports_dir / "consistency_report.md"
    lines = ["# Monte Carlo Consistency Report", "", "## Joint NEES/NIS evidence", ""]
    lines.append(
        "| Kind | Group | Window | DOF | Normalized mean | Mean interval | In bounds | Joint 3σ coverage | Acceptance |"
    )
    lines.append("| --- | --- | --- | ---: | ---: | --- | --- | ---: | ---: |")
    for row in summary_rows:
        acceptance = row["acceptance_rate"]
        acceptance_text = "-" if acceptance is None else f"{float(acceptance):.3f}"
        lines.append(
            f"| {row['kind']} | {row['group']} | {row['window']} | {row['dof']} | "
            f"{float(row['normalized_mean']):.3f} | "
            f"[{float(row['mean_lower_bound']):.3f}, {float(row['mean_upper_bound']):.3f}] | "
            f"{row['mean_within_bounds']} | {float(row['joint_3sigma_coverage']):.3f} | {acceptance_text} |"
        )
    lines.extend(
        [
            "",
            "Joint metrics use one chi-square statistic per run and epoch; the steady-state row summarizes the final 20% of analysis epochs. Scalar coverage is descriptive marginal evidence and does not replace joint cross-correlation-aware NEES.",
        ]
    )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    paths["markdown_report"] = report_path
    return paths


def generate_consistency_outputs(
    bundle_path: Path,
    summary_dir: Path,
    *,
    refresh_cache: bool = False,
    max_plot_points: int | None = None,
    heatmap_modes: Sequence[str] | None = None,
    parallel_jobs: int = 1,
    force: bool = False,
) -> dict[str, object]:
    """Generate cached joint-consistency dashboards and reports for one campaign bundle."""
    from navkit_analysis.analysis_cache import (
        cache_fingerprint,
        cached_output_paths,
        write_cache_marker,
    )
    from navkit_analysis.bundle import bundle_metadata
    from navkit_analysis.consistency_plots import HEATMAP_MODES, write_consistency_dashboards

    started_s = time.perf_counter()
    selected_modes = tuple(heatmap_modes or HEATMAP_MODES)
    cache_inputs = {
        "bundle_fingerprint": bundle_metadata(bundle_path).get("package_fingerprint"),
        "max_plot_points": max_plot_points,
        "heatmap_modes": selected_modes,
    }
    artifact_fingerprint = cache_fingerprint("consistency_dashboards", cache_inputs)
    artifact_marker = summary_dir / ".consistency_artifact_cache.json"
    cached_paths = (
        None
        if refresh_cache or force
        else cached_output_paths(artifact_marker, artifact_fingerprint)
    )
    if cached_paths is not None:
        nees_series, nis_series, marginal_series = load_consistency_cache(bundle_path)
        return {
            "cache_elapsed_s": 0.0,
            "plot_elapsed_s": 0.0,
            "report_elapsed_s": 0.0,
            "total_elapsed_s": time.perf_counter() - started_s,
            "figure_paths": {path.stem: str(path) for path in cached_paths},
            "report_paths": {},
            "nees_group_count": len(nees_series),
            "nis_group_count": len(nis_series),
            "marginal_group_count": len(marginal_series),
            "reused": True,
        }
    cache_started_s = time.perf_counter()
    if refresh_cache:
        nees_series, nis_series, marginal_series = refresh_consistency_cache(
            bundle_path,
            max_plot_points,
        )
    else:
        try:
            nees_series, nis_series, marginal_series = load_consistency_cache(bundle_path)
            if not marginal_series:
                nees_series, nis_series, marginal_series = refresh_consistency_cache(
                    bundle_path,
                    max_plot_points,
                )
        except ValueError:
            nees_series, nis_series, marginal_series = refresh_consistency_cache(
                bundle_path,
                max_plot_points,
            )
    cache_elapsed_s = time.perf_counter() - cache_started_s

    plot_started_s = time.perf_counter()
    figure_paths = write_consistency_dashboards(
        nees_series,
        nis_series,
        marginal_series,
        summary_dir / "consistency_figures",
        heatmap_modes=selected_modes,
        write_index=heatmap_modes is None,
        parallel_jobs=parallel_jobs,
    )
    plot_elapsed_s = time.perf_counter() - plot_started_s
    report_started_s = time.perf_counter()
    report_paths = write_consistency_reports(
        bundle_path,
        summary_dir / "consistency_reports",
        [*nees_series, *nis_series],
    )
    output_paths = [*figure_paths.values(), *report_paths.values()]
    write_cache_marker(
        artifact_marker,
        kind="consistency_dashboards",
        fingerprint=artifact_fingerprint,
        inputs=cache_inputs,
        outputs=output_paths,
    )
    report_elapsed_s = time.perf_counter() - report_started_s
    return {
        "cache_elapsed_s": cache_elapsed_s,
        "plot_elapsed_s": plot_elapsed_s,
        "report_elapsed_s": report_elapsed_s,
        "total_elapsed_s": time.perf_counter() - started_s,
        "figure_paths": {name: str(path) for name, path in figure_paths.items()},
        "report_paths": {name: str(path) for name, path in report_paths.items()},
        "nees_group_count": len(nees_series),
        "nis_group_count": len(nis_series),
        "marginal_group_count": len(marginal_series),
    }
