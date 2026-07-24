# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""HDF5 packaging and access for portable NavKit analysis artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

import h5py
import numpy as np
import pandas as pd

from navkit_analysis.data import RunData, load_run
from navkit_analysis.schema import (
    ANALYSIS_BUNDLE_SCHEMA,
    MONTE_CARLO_CAMPAIGN_SCHEMA,
    validate_schema,
)


RUN_DATA_FIELDS = (
    "nav",
    "gnss_pos_update",
    "gnss_vel_update",
    "gnss_position_debug",
    "gnss_velocity_debug",
    "truth",
    "imu",
    "imu_debug",
    "filter_correction",
)
RUN_DERIVED_FIELDS = ("truth_error",)


def _json_attr(group: h5py.Group | h5py.File, name: str, value: Mapping[str, object]) -> None:
    group.attrs[name] = json.dumps(value, sort_keys=True)


def _read_json_attr(group: h5py.Group | h5py.File, name: str) -> dict[str, object]:
    encoded = group.attrs.get(name)
    if encoded is None:
        return {}
    if isinstance(encoded, bytes):
        encoded = encoded.decode("utf-8")
    if not isinstance(encoded, str):
        raise ValueError(f"HDF5 attribute '{name}' must contain JSON text")
    parsed = json.loads(encoded)
    if not isinstance(parsed, dict):
        raise ValueError(f"HDF5 attribute '{name}' must contain a JSON object")
    return parsed


def _write_frame(parent: h5py.Group, name: str, frame: pd.DataFrame) -> None:
    group = parent.create_group(name)
    group.attrs["columns"] = json.dumps(list(frame.columns))
    group.attrs["row_count"] = len(frame)
    for column in frame.columns:
        values = frame[column].to_numpy()
        if values.dtype.kind in {"O", "U", "S"}:
            string_values = np.asarray(["" if pd.isna(value) else str(value) for value in values])
            group.create_dataset(column, data=string_values, dtype=h5py.string_dtype("utf-8"))
        else:
            group.create_dataset(
                column,
                data=values,
                compression="lzf",
                shuffle=True,
            )


def _read_frame(parent: h5py.Group, name: str) -> pd.DataFrame | None:
    if name not in parent:
        return None
    group = parent[name]
    if not isinstance(group, h5py.Group):
        raise ValueError(f"bundle table '{name}' is not an HDF5 group")
    encoded_columns = group.attrs.get("columns")
    if isinstance(encoded_columns, bytes):
        encoded_columns = encoded_columns.decode("utf-8")
    if not isinstance(encoded_columns, str):
        raise ValueError(f"bundle table '{name}' is missing its ordered column metadata")
    columns = json.loads(encoded_columns)
    if not isinstance(columns, list) or not all(isinstance(column, str) for column in columns):
        raise ValueError(f"bundle table '{name}' has invalid ordered column metadata")
    values: dict[str, np.ndarray] = {}
    for column in columns:
        data = group[column][()]
        if isinstance(data, np.ndarray) and data.dtype.kind == "S":
            values[column] = data.astype(str)
        else:
            values[column] = data
    return pd.DataFrame(values, columns=columns)


def _run_metadata(run_dir: Path) -> dict[str, object]:
    metadata: dict[str, object] = {"source_run_dir": str(run_dir.resolve())}
    for output_name, source_name in (
        ("campaign_run_manifest", "run_manifest.json"),
        ("simulation_run_manifest", "data/run_manifest.json"),
        ("runtime_config", "input.effective.json"),
        ("timing", "data/timing.json"),
    ):
        source = run_dir / source_name
        if source.exists():
            parsed = json.loads(source.read_text(encoding="utf-8"))
            if isinstance(parsed, dict):
                metadata[output_name] = parsed
    log_schemas: dict[str, object] = {}
    for metadata_path in sorted((run_dir / "data").glob("*.meta.json")):
        parsed = json.loads(metadata_path.read_text(encoding="utf-8"))
        if isinstance(parsed, dict):
            log_schemas[metadata_path.name] = parsed
    metadata["log_schemas"] = log_schemas
    return metadata


def _write_run(group: h5py.Group, run_id: str, run_dir: Path) -> RunData:
    """Write one run and return its already-loaded analysis data."""
    run = load_run(run_dir)
    run_group = group.create_group(run_id)
    _json_attr(run_group, "metadata", _run_metadata(run.run_dir))
    data_group = run_group.create_group("data")
    derived_group = run_group.create_group("derived")
    for field_name in RUN_DATA_FIELDS:
        frame = getattr(run, field_name)
        if frame is not None:
            _write_frame(data_group, field_name, frame)
    for field_name in RUN_DERIVED_FIELDS:
        frame = getattr(run, field_name)
        if frame is not None:
            _write_frame(derived_group, field_name, frame)
    return run


def _run_ids_from_manifest(campaign_dir: Path) -> list[tuple[str, Path]]:
    manifest_path = campaign_dir / "campaign_manifest.json"
    if not manifest_path.exists():
        return [(campaign_dir.name, campaign_dir)]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError(f"campaign manifest '{manifest_path}' must be an object")
    validate_schema(manifest, MONTE_CARLO_CAMPAIGN_SCHEMA, str(manifest_path))
    entries = manifest.get("runs")
    if not isinstance(entries, list):
        raise ValueError(f"campaign manifest '{manifest_path}' is missing its run list")
    run_items: list[tuple[str, Path]] = []
    for entry in entries:
        if not isinstance(entry, dict) or entry.get("status") != "passed":
            continue
        run_dir = entry.get("run_dir")
        if not isinstance(run_dir, str):
            raise ValueError(f"campaign manifest '{manifest_path}' has a run without a directory")
        run_id = entry.get("run_index")
        name = f"run_{run_id:06d}" if isinstance(run_id, int) else Path(run_dir).name
        run_items.append((name, Path(run_dir)))
    if not run_items:
        raise ValueError(f"campaign manifest '{manifest_path}' has no successful runs to package")
    return run_items


def _campaign_metadata(campaign_dir: Path) -> dict[str, object]:
    metadata: dict[str, object] = {"source_campaign_dir": str(campaign_dir.resolve())}
    for output_name, source_name in (
        ("campaign_manifest", "campaign_manifest.json"),
        ("campaign_config", "campaign_config.effective.json"),
    ):
        source = campaign_dir / source_name
        if source.exists():
            parsed = json.loads(source.read_text(encoding="utf-8"))
            if isinstance(parsed, dict):
                metadata[output_name] = parsed
    return metadata


def _write_monte_carlo_series(
    aggregate_group: h5py.Group,
    runs: list[object],
    max_plot_points: int | None,
) -> None:
    from navkit_analysis.monte_carlo import aggregate_monte_carlo_series

    series_group = aggregate_group.create_group("monte_carlo_series")
    for series in aggregate_monte_carlo_series(runs, max_plot_points=max_plot_points):
        name = series.output_name.removesuffix(".png")
        group = series_group.create_group(name)
        group.attrs["labels"] = json.dumps(series.labels)
        group.attrs["axis_names"] = json.dumps(series.axis_names)
        group.attrs["scale"] = series.scale
        group.attrs["title"] = series.title
        group.attrs["ylabel"] = series.ylabel
        group.attrs["output_name"] = series.output_name
        group.attrs["run_count"] = series.run_count
        for dataset_name, values in (
            ("time_s", series.time_s),
            ("run_errors", series.run_errors),
            ("mean_error", series.mean_error),
            ("empirical_sigma", series.empirical_sigma),
            ("mean_filter_sigma", series.mean_filter_sigma),
        ):
            group.create_dataset(dataset_name, data=values, compression="lzf", shuffle=True)
        centered_errors = series.run_errors - series.mean_error[None, :, :]
        denominator = max(series.run_count - 1, 1)
        empirical_covariance = np.einsum(
            "rti,rtj->tij", centered_errors, centered_errors, optimize=True
        ) / denominator
        group.create_dataset(
            "empirical_covariance",
            data=empirical_covariance,
            compression="lzf",
            shuffle=True,
        )


def _write_consistency_arrays(
    aggregate_group: h5py.Group,
    runs: list[object],
    run_dirs: list[Path],
) -> None:
    """Cache NIS/NEES samples that otherwise require repeated raw-CSV scans."""
    from navkit_analysis.monte_carlo import ECEF_GROUPS, _group_nees_from_frame

    consistency_group = aggregate_group.create_group("consistency")
    nees_group = consistency_group.create_group("nees")
    for group in ECEF_GROUPS:
        values = [
            nees
            for run in runs
            if (nees := _group_nees_from_frame(run.ecef, group.labels)) is not None
        ]
        if values:
            nees_group.create_dataset(
                group.output_name.removesuffix(".png"),
                data=np.concatenate(values),
                compression="lzf",
                shuffle=True,
            )
    nis_group = consistency_group.create_group("nis")
    for output_name, csv_name in (
        ("gnss_position", "gnss_pos_update.csv"),
        ("gnss_velocity", "gnss_vel_update.csv"),
    ):
        values: list[np.ndarray] = []
        for run_dir in run_dirs:
            source = run_dir / "data" / csv_name
            if not source.exists():
                continue
            frame = pd.read_csv(source, usecols=lambda column: column in {"nis", "accepted"})
            if "nis" in frame:
                nis = pd.to_numeric(frame["nis"], errors="coerce").to_numpy(dtype=float)
                finite_nis = nis[np.isfinite(nis)]
                if finite_nis.size > 0:
                    values.append(finite_nis)
        if values:
            nis_group.create_dataset(
                output_name,
                data=np.concatenate(values).astype(np.float64, copy=False),
                compression="lzf",
                shuffle=True,
            )


def package_analysis(
    source: Path,
    output_path: Path,
    *,
    max_plot_points: int | None = None,
) -> Path:
    """Package one CSV run or campaign into a versioned HDF5 analysis bundle."""
    source = source.resolve()
    output_path = output_path.resolve()
    run_items = _run_ids_from_manifest(source)
    campaign = (source / "campaign_manifest.json").exists()
    bundle_metadata: dict[str, object] = {
        "schema": ANALYSIS_BUNDLE_SCHEMA,
        "source_kind": "monte_carlo_campaign" if campaign else "single_run",
        "source_path": str(source),
        "derivation_assumptions": {
            "truth_alignment": "linear interpolation with numpy.interp",
            "ned_error_transform": "ECEF-to-NED DCM evaluated from truth position at each sample",
            "covariance_transform": "P_n = C_e2n P_e C_e2n^T",
            "plot_decimation": "uniform index sampling on the first successful run time grid",
        },
        "max_plot_points": max_plot_points,
    }
    if campaign:
        bundle_metadata.update(_campaign_metadata(source))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as bundle:
        bundle.attrs["schema"] = ANALYSIS_BUNDLE_SCHEMA
        _json_attr(bundle, "metadata", bundle_metadata)
        runs_group = bundle.create_group("runs")
        run_data_by_id: dict[str, RunData] = {}
        for run_id, run_dir in run_items:
            run_data_by_id[run_id] = _write_run(runs_group, run_id, run_dir)
        if campaign:
            aggregate_group = bundle.create_group("aggregate")
            from navkit_analysis.monte_carlo import load_successful_runs

            run_dirs = [run_dir for _, run_dir in run_items]
            monte_carlo_runs = load_successful_runs(run_dirs)
            _write_monte_carlo_series(
                aggregate_group,
                monte_carlo_runs,
                max_plot_points,
            )
            _write_consistency_arrays(aggregate_group, monte_carlo_runs, run_dirs)
            from navkit_analysis.consistency import (
                build_marginal_nse_series,
                build_nees_series,
                build_nis_series,
                load_nis_frames_from_run_dirs,
                write_consistency_cache,
            )

            truth_error_frames = [
                run_data_by_id[run_id].truth_error
                for run_id, _ in run_items
                if run_data_by_id[run_id].truth_error is not None
            ]
            write_consistency_cache(
                aggregate_group,
                build_nees_series(truth_error_frames, max_plot_points),
                build_nis_series(load_nis_frames_from_run_dirs(run_dirs), max_plot_points),
                build_marginal_nse_series(truth_error_frames, max_plot_points),
            )
        else:
            bundle.create_group("aggregate")
    print(f"Wrote {output_path}")
    return output_path


def bundle_metadata(path: Path) -> dict[str, object]:
    """Read validated bundle-level metadata without loading sample arrays."""
    with h5py.File(path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(path))
        return _read_json_attr(bundle, "metadata")


def bundle_run_ids(path: Path) -> list[str]:
    """Return the explicitly stored run identifiers in one analysis bundle."""
    with h5py.File(path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(path))
        runs = bundle.get("runs")
        if not isinstance(runs, h5py.Group):
            raise ValueError(f"bundle '{path}' is missing its runs group")
        return sorted(runs.keys())


def load_run_from_bundle(path: Path, run_id: str | None = None) -> RunData:
    """Load one packaged run through the same RunData contract as CSV analysis."""
    with h5py.File(path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(path))
        runs = bundle.get("runs")
        if not isinstance(runs, h5py.Group):
            raise ValueError(f"bundle '{path}' is missing its runs group")
        selected_run_id = run_id or next(iter(sorted(runs.keys())), None)
        if selected_run_id is None or selected_run_id not in runs:
            raise ValueError(f"bundle '{path}' does not contain run '{run_id}'")
        run_group = runs[selected_run_id]
        if not isinstance(run_group, h5py.Group):
            raise ValueError(f"bundle run '{selected_run_id}' is not a group")
        data = run_group.get("data")
        derived = run_group.get("derived")
        if not isinstance(data, h5py.Group) or not isinstance(derived, h5py.Group):
            raise ValueError(f"bundle run '{selected_run_id}' is missing data or derived groups")
        nav = _read_frame(data, "nav")
        if nav is None:
            raise ValueError(f"bundle run '{selected_run_id}' is missing nav data")
        run_dir = path.resolve().parent / selected_run_id
        return RunData(
            run_dir=run_dir,
            data_dir=run_dir,
            figures_dir=run_dir / "figures",
            nav=nav,
            gnss_pos_update=_read_frame(data, "gnss_pos_update"),
            gnss_vel_update=_read_frame(data, "gnss_vel_update"),
            gnss_position_debug=_read_frame(data, "gnss_position_debug"),
            gnss_velocity_debug=_read_frame(data, "gnss_velocity_debug"),
            truth=_read_frame(data, "truth"),
            imu=_read_frame(data, "imu"),
            imu_debug=_read_frame(data, "imu_debug"),
            filter_correction=_read_frame(data, "filter_correction"),
            truth_error=_read_frame(derived, "truth_error"),
        )


def load_monte_carlo_series_from_bundle(
    path: Path,
    selected: set[str] | None = None,
) -> list[object]:
    """Load selected cached aggregate series from a campaign bundle."""
    from navkit_analysis.monte_carlo import MonteCarloSeries

    series_items: list[MonteCarloSeries] = []
    with h5py.File(path, "r") as bundle:
        schema = bundle.attrs.get("schema")
        if isinstance(schema, bytes):
            schema = schema.decode("utf-8")
        validate_schema({"schema": schema}, ANALYSIS_BUNDLE_SCHEMA, str(path))
        aggregate = bundle.get("aggregate/monte_carlo_series")
        if not isinstance(aggregate, h5py.Group):
            raise ValueError(f"bundle '{path}' does not contain cached Monte Carlo aggregates")
        for name in sorted(aggregate.keys()):
            group = aggregate[name]
            if not isinstance(group, h5py.Group):
                continue
            output_name = str(group.attrs["output_name"])
            quantity_name = output_name.removesuffix(".png").removeprefix(
                "monte_carlo_error_covariance_"
            )
            if selected is not None and quantity_name not in selected:
                continue
            labels = tuple(json.loads(group.attrs["labels"]))
            axis_names = tuple(json.loads(group.attrs["axis_names"]))
            series_items.append(
                MonteCarloSeries(
                    time_s=group["time_s"][()],
                    labels=labels,
                    axis_names=axis_names,
                    run_errors=group["run_errors"][()],
                    mean_error=group["mean_error"][()],
                    empirical_sigma=group["empirical_sigma"][()],
                    mean_filter_sigma=group["mean_filter_sigma"][()],
                    scale=float(group.attrs["scale"]),
                    title=str(group.attrs["title"]),
                    ylabel=str(group.attrs["ylabel"]),
                    output_name=output_name,
                    run_count=int(group.attrs["run_count"]),
                )
            )
    return series_items


def is_analysis_bundle(path: Path) -> bool:
    """Return whether a path names an HDF5 analysis bundle by file extension."""
    return path.is_file() and path.suffix.lower() in {".h5", ".hdf5"}
