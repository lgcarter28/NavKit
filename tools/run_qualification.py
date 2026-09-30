# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Run or reuse NavKit qualification evidence and emit one compact report."""

from __future__ import annotations

import argparse
import copy
import json
import platform
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import h5py
import numpy as np

from navkit_analysis.analysis_performance import canonical_json_digest, file_digest
from navkit_analysis.bundle import bundle_metadata
from navkit_analysis.consistency import (
    ERROR_STATE_LABELS,
    ConsistencySeries,
    consistency_cache_provenance,
    load_consistency_cache,
    refresh_consistency_cache,
)
from navkit_analysis.covariance_diagnostics import two_block_schur_nees
from navkit_analysis.qualification import (
    QualificationCampaignSpec,
    QualificationCheck,
    QualificationCriterionSpec,
    QualificationDisposition,
    QualificationReport,
    QualificationStatus,
    QualificationSuite,
    QualificationWindow,
    aggregate_qualification,
    evaluate_equivalence,
    evaluate_lower_bound,
    evaluate_upper_bound,
    load_qualification_suite,
    qualification_baseline_deltas,
    qualification_check_claim_passes,
    reduce_window_mean_per_run,
    select_qualification_window,
    student_t_mean_confidence_interval,
    write_qualification_baseline,
)
from navkit_analysis.regression import (
    evaluate_truth_reconstruction,
    load_deterministic_regression_suite,
)
from navkit_analysis.schema import (
    DETERMINISTIC_REGRESSION_REPORT_SCHEMA,
    QUALIFICATION_BASELINE_SCHEMA,
    QUALIFICATION_REPORT_SCHEMA,
    validate_schema,
)

from internal.evidence_provenance import build_artifact_provenance, git_provenance
from internal.navkit_build_dirs import DEFAULT_GENERATOR, resolve_build_dir
from internal.monte_carlo_seeds import derive_seed
from internal.perf_artifacts import DEFAULT_NAVKIT_CONFIG
from internal.runtime_config import load_runtime_config


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f"{path.suffix}.tmp")
    try:
        serialized = json.dumps(value, indent=2, allow_nan=False) + "\n"
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"qualification report is not strict-JSON serializable: {path}"
        ) from error
    temporary.write_text(serialized, encoding="utf-8")
    temporary.replace(path)


def _resolve_inside_repository(root: Path, candidate: Path) -> Path:
    resolved = candidate if candidate.is_absolute() else root / candidate
    resolved = resolved.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"qualification output must stay inside the repository: {candidate}")
    return resolved


def _run_command(command: list[str], log_prefix: Path) -> None:
    print("Running " + " ".join(command))
    log_prefix.parent.mkdir(parents=True, exist_ok=True)
    stdout_path = log_prefix.with_suffix(".stdout.txt")
    stderr_path = log_prefix.with_suffix(".stderr.txt")
    stderr_path.write_text(
        "Standard error is merged into the streamed standard-output log.\n",
        encoding="utf-8",
    )
    with stdout_path.open("w", encoding="utf-8") as stdout_file:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        if process.stdout is None:
            raise RuntimeError("failed to capture qualification prerequisite output")
        for line in process.stdout:
            print(line, end="")
            stdout_file.write(line)
            stdout_file.flush()
        return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(
            f"qualification prerequisite returned {return_code}: "
            + " ".join(command)
        )


def _execute_deterministic_suite(
    suite: QualificationSuite,
    output_dir: Path,
) -> Path:
    root = Path(__file__).resolve().parents[1]
    report_dir = output_dir / "deterministic"
    command = [
        sys.executable,
        str(root / "tools" / "run_regression.py"),
        str(suite.deterministic_suite),
        "--output-dir",
        str(report_dir),
    ]
    _run_command(command, output_dir / "logs" / "deterministic")
    return report_dir / "report.json"


def _execute_campaigns(
    suite: QualificationSuite,
    tier: str,
    output_dir: Path,
    parallel_jobs: int | None,
) -> Path:
    root = Path(__file__).resolve().parents[1]
    campaign_root = output_dir / "campaigns"
    for campaign in suite.campaigns:
        command = [
            sys.executable,
            str(root / "tools" / "run_monte_carlo.py"),
            str(campaign.config),
            "--run-count",
            str(suite.campaign_sizes[tier]),
            "--output-root",
            str(campaign_root),
            "--no-aggregate-plots",
            "--no-consistency-dashboards",
        ]
        if parallel_jobs is not None:
            command.extend(["--parallel-jobs", str(parallel_jobs)])
        _run_command(command, output_dir / "logs" / campaign.name)
    return campaign_root


def _load_json(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def _canonical_json_file_digest(path: Path) -> str:
    """Return a formatting-insensitive digest for one JSON object file."""
    return canonical_json_digest(_load_json(path))


def _is_sha256(value: object) -> bool:
    """Return whether ``value`` is one lowercase hexadecimal SHA-256 digest."""
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _escape_json_pointer_token(token: str) -> str:
    return token.replace("~", "~0").replace("/", "~1")


def _unescape_json_pointer_token(token: str) -> str:
    return token.replace("~1", "/").replace("~0", "~")


def _discover_seed_paths(node: object, path: str = "") -> list[str]:
    paths: list[str] = []
    if isinstance(node, Mapping):
        if "seed" in node:
            paths.append(f"{path}/seed")
        for key, value in node.items():
            child = f"{path}/{_escape_json_pointer_token(str(key))}"
            paths.extend(_discover_seed_paths(value, child))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            paths.extend(_discover_seed_paths(value, f"{path}/{index}"))
    return paths


def _replace_json_pointer(root: dict[str, Any], pointer: str, value: object) -> None:
    if not pointer.startswith("/"):
        raise ValueError(f"seed path must be an absolute JSON pointer: {pointer}")
    tokens = [
        _unescape_json_pointer_token(token) for token in pointer.split("/")[1:]
    ]
    if not tokens:
        raise ValueError("seed path must identify a value")
    current: object = root
    for token in tokens[:-1]:
        try:
            current = (
                current[int(token)]
                if isinstance(current, list)
                else current[token]
            )
        except (IndexError, KeyError, TypeError, ValueError) as error:
            raise ValueError(f"runtime config lacks configured seed path '{pointer}'") from error
    final = tokens[-1]
    try:
        if isinstance(current, list):
            current[int(final)] = value
        else:
            current[final] = value
    except (IndexError, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"runtime config lacks configured seed path '{pointer}'") from error


def _normalized_runtime_config(
    config: Mapping[str, object], seed_paths: list[str]
) -> dict[str, Any]:
    """Remove per-run location/identity and normalize derived seed values."""
    normalized = copy.deepcopy(dict(config))
    normalized.pop("run_name", None)
    normalized.pop("output_dir", None)
    for seed_path in seed_paths:
        _replace_json_pointer(normalized, seed_path, "<derived-seed>")
    return normalized


def _hdf5_json_attribute(group: h5py.Group, name: str) -> dict[str, Any]:
    encoded = group.attrs.get(name)
    if isinstance(encoded, bytes):
        encoded = encoded.decode("utf-8")
    if not isinstance(encoded, str):
        raise ValueError(f"HDF5 group lacks JSON attribute '{name}'")
    parsed = json.loads(encoded)
    if not isinstance(parsed, dict):
        raise ValueError(f"HDF5 attribute '{name}' must contain an object")
    return parsed


@dataclass(frozen=True)
class _PackagedRunEvidence:
    """Portable per-run identity, seed, and effective-configuration evidence."""

    run_name: str
    runtime_config: dict[str, Any]
    campaign_run_manifest: dict[str, Any]


def _packaged_run_evidence(bundle_path: Path) -> list[_PackagedRunEvidence]:
    """Load every portable effective run configuration from HDF5 metadata."""
    packaged: list[_PackagedRunEvidence] = []
    with h5py.File(bundle_path, "r") as bundle:
        runs = bundle.get("runs")
        if not isinstance(runs, h5py.Group) or not runs:
            raise ValueError(f"bundle has no packaged runs: {bundle_path}")
        for run_name in sorted(runs.keys()):
            metadata = _hdf5_json_attribute(runs[run_name], "metadata")
            runtime_config = metadata.get("runtime_config")
            if not isinstance(runtime_config, dict):
                raise ValueError(
                    f"bundle run '{run_name}' lacks embedded effective runtime "
                    f"config: {bundle_path}"
                )
            campaign_run_manifest = metadata.get("campaign_run_manifest")
            if not isinstance(campaign_run_manifest, dict):
                raise ValueError(
                    f"bundle run '{run_name}' lacks embedded campaign-run "
                    f"manifest: {bundle_path}"
                )
            packaged.append(
                _PackagedRunEvidence(
                    run_name=run_name,
                    runtime_config=runtime_config,
                    campaign_run_manifest=campaign_run_manifest,
                )
            )
    return packaged


def _json_pointer_value(root: Mapping[str, object], pointer: str) -> object:
    """Read one absolute JSON-pointer value from a runtime configuration."""
    if not pointer.startswith("/"):
        raise ValueError(f"seed path must be an absolute JSON pointer: {pointer}")
    current: object = root
    for token in (
        _unescape_json_pointer_token(value) for value in pointer.split("/")[1:]
    ):
        try:
            current = current[int(token)] if isinstance(current, list) else current[token]
        except (IndexError, KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"runtime config lacks configured seed path '{pointer}'"
            ) from error
    return current


def _campaign_input_provenance(
    campaign: QualificationCampaignSpec,
) -> dict[str, object]:
    campaign_document = _load_json(campaign.config)
    nominal_value = campaign_document.get("nominal_config")
    if not isinstance(nominal_value, str) or not nominal_value:
        raise ValueError(f"campaign lacks nominal_config: {campaign.config}")
    nominal_path = (campaign.config.parent / nominal_value).resolve()
    effective_nominal = load_runtime_config(nominal_path)
    return {
        "source_config_sha256": canonical_json_digest(campaign_document),
        "source_config_raw_sha256": file_digest(campaign.config),
        "nominal_config": str(nominal_path),
        "nominal_config_sha256": canonical_json_digest(effective_nominal),
    }


def _provenance_identity(
    provenance: Mapping[str, object],
    context: str,
) -> dict[str, object]:
    """Extract portable source/build evidence while rejecting incomplete records."""
    git = provenance.get("git")
    build = provenance.get("build")
    if not isinstance(git, Mapping) or not isinstance(build, Mapping):
        raise ValueError(f"{context} lacks source/build provenance")
    manifest = build.get("manifest")
    executable = build.get("application_executable")
    if not isinstance(manifest, Mapping) or not isinstance(executable, Mapping):
        raise ValueError(f"{context} lacks build-artifact provenance")
    identity = {
        "git": {
            "revision": git.get("revision"),
            "dirty": git.get("dirty"),
            "dirty_tree_sha256": git.get("dirty_tree_sha256"),
        },
        "build": {
            "manifest_sha256": manifest.get("sha256"),
            "manifest_canonical_sha256": manifest.get("canonical_sha256"),
            "application_executable_sha256": executable.get("sha256"),
            "application_executable_size_bytes": executable.get("size_bytes"),
        },
    }
    git_identity = identity["git"]
    build_identity = identity["build"]
    assert isinstance(git_identity, dict)
    assert isinstance(build_identity, dict)
    if not isinstance(git_identity["revision"], str) or not git_identity["revision"]:
        raise ValueError(f"{context} lacks a Git revision")
    if not isinstance(git_identity["dirty"], bool):
        raise ValueError(f"{context} has invalid Git dirty-state provenance")
    if git_identity["dirty"] and not isinstance(
        git_identity["dirty_tree_sha256"], str
    ):
        raise ValueError(f"{context} lacks a dirty-tree fingerprint")
    if not git_identity["dirty"] and git_identity["dirty_tree_sha256"] is not None:
        raise ValueError(f"{context} has a dirty-tree fingerprint for a clean tree")
    if not all(
        isinstance(build_identity[name], str) and bool(build_identity[name])
        for name in (
            "manifest_sha256",
            "manifest_canonical_sha256",
            "application_executable_sha256",
        )
    ) or not isinstance(build_identity["application_executable_size_bytes"], int):
        raise ValueError(f"{context} has invalid build-artifact fingerprints")
    return identity


def _validate_current_build_provenance(
    observed_provenance: Mapping[str, object],
    current_provenance: Mapping[str, object],
    context: str,
) -> dict[str, object]:
    """Require evidence from the exact currently selected compiled artifact."""
    observed = _provenance_identity(observed_provenance, context)
    current = _provenance_identity(current_provenance, "current selected build")
    observed_build = observed["build"]
    current_build = current["build"]
    if observed_build != current_build:
        raise ValueError(
            f"{context} was not generated by the current selected build: "
            f"expected={current_build}, observed={observed_build}"
        )
    return {
        "generation": observed,
        "current_build": current_build,
        "build_matches_current": True,
        "source_matches_current": observed["git"] == current["git"],
    }


def _current_execution_provenance(
    root: Path,
    git: Mapping[str, object],
    build_type: str,
    navkit_config: str,
    generator: str,
    build_dir: object = None,
) -> dict[str, object]:
    """Fingerprint the current artifact selected by one execution contract."""
    if build_dir is not None and not isinstance(build_dir, str):
        raise ValueError("execution.build_dir must be a string or null")
    resolved_build_dir = resolve_build_dir(
        root,
        build_type,
        navkit_config,
        Path(build_dir) if isinstance(build_dir, str) else None,
        generator=generator,
    )
    return {
        "git": dict(git),
        "build": build_artifact_provenance(resolved_build_dir, build_type),
    }


def _validate_embedded_campaign_config(
    campaign: QualificationCampaignSpec,
    metadata: Mapping[str, object],
    expected_run_count: int,
    current_provenance: Mapping[str, object],
) -> Mapping[str, object]:
    embedded = metadata.get("campaign_config")
    if not isinstance(embedded, Mapping):
        raise ValueError(f"bundle lacks embedded campaign config: {campaign.name}")
    selected = _load_json(campaign.config)
    selected_nominal = selected.get("nominal_config")
    if not isinstance(selected_nominal, str) or not selected_nominal:
        raise ValueError(f"campaign lacks nominal_config: {campaign.config}")
    embedded_runs = embedded.get("runs")
    selected_runs = selected.get("runs")
    embedded_randomization = embedded.get("randomization")
    selected_randomization = selected.get("randomization")
    embedded_execution = embedded.get("execution")
    selected_execution = selected.get("execution", {})
    if not all(
        isinstance(value, Mapping)
        for value in (
            embedded_runs,
            selected_runs,
            embedded_randomization,
            selected_randomization,
            embedded_execution,
            selected_execution,
        )
    ):
        raise ValueError(f"campaign configuration is structurally invalid: {campaign.name}")
    assert isinstance(embedded_runs, Mapping)
    assert isinstance(selected_runs, Mapping)
    assert isinstance(embedded_randomization, Mapping)
    assert isinstance(selected_randomization, Mapping)
    assert isinstance(embedded_execution, Mapping)
    assert isinstance(selected_execution, Mapping)
    expected_values = {
        "campaign_name": campaign.campaign_name,
        "run_count": expected_run_count,
        "start_index": selected_runs.get("start_index", 0),
        "master_seed": selected_randomization.get("master_seed"),
        "seed_policy": selected_randomization.get("seed_policy", "derive_all"),
        "build_type": selected_execution.get("build_type", "Release"),
        "navkit_config": selected_execution.get(
            "navkit_config", DEFAULT_NAVKIT_CONFIG
        ),
        "generator": selected_execution.get("generator", DEFAULT_GENERATOR),
    }
    observed_values = {
        "campaign_name": embedded.get("campaign_name"),
        "run_count": embedded_runs.get("count"),
        "start_index": embedded_runs.get("start_index", 0),
        "master_seed": embedded_randomization.get("master_seed"),
        "seed_policy": embedded_randomization.get("seed_policy"),
        "build_type": embedded_execution.get("build_type"),
        "navkit_config": embedded_execution.get("navkit_config"),
        "generator": embedded_execution.get("generator"),
    }
    mismatches = {
        name: {"expected": expected_values[name], "observed": observed_values[name]}
        for name in expected_values
        if expected_values[name] != observed_values[name]
    }
    if mismatches:
        raise ValueError(
            f"bundle campaign config does not match selected qualification input "
            f"for {campaign.name}: {mismatches}"
        )
    embedded_provenance = embedded.get("provenance")
    if not isinstance(embedded_provenance, Mapping):
        raise ValueError(f"bundle lacks generation provenance: {campaign.name}")
    current_nominal = load_runtime_config(
        (campaign.config.parent / selected_nominal).resolve()
    )
    expected_nominal_digest = canonical_json_digest(current_nominal)
    if embedded_provenance.get("nominal_config_sha256") != expected_nominal_digest:
        raise ValueError(
            f"bundle nominal-config provenance does not match selected input for "
            f"{campaign.name}"
        )
    _validate_current_build_provenance(
        embedded_provenance,
        current_provenance,
        f"bundle '{campaign.name}'",
    )
    return embedded


def _validate_bundle_runtime_config(
    campaign: QualificationCampaignSpec,
    bundle_path: Path,
    embedded_campaign_config: Mapping[str, object],
) -> dict[str, object]:
    """Validate portable packaged runtime evidence against current nominal input."""
    selected_campaign = _load_json(campaign.config)
    nominal_value = selected_campaign.get("nominal_config")
    if not isinstance(nominal_value, str) or not nominal_value:
        raise ValueError(f"campaign lacks nominal_config: {campaign.config}")
    nominal_path = (campaign.config.parent / nominal_value).resolve()
    current_nominal = load_runtime_config(nominal_path)
    expected_seed_paths = sorted(_discover_seed_paths(current_nominal))

    randomization = embedded_campaign_config.get("randomization")
    if not isinstance(randomization, Mapping):
        raise ValueError(f"bundle lacks randomization metadata: {campaign.name}")
    configured_paths = randomization.get("seed_paths")
    if not isinstance(configured_paths, list) or not all(
        isinstance(path, str) for path in configured_paths
    ):
        raise ValueError(f"bundle lacks configured seed paths: {campaign.name}")
    observed_seed_paths = sorted(configured_paths)
    if observed_seed_paths != expected_seed_paths:
        raise ValueError(
            f"bundle seed paths do not match selected nominal config for {campaign.name}: "
            f"expected={expected_seed_paths}, observed={observed_seed_paths}"
        )

    expected_normalized = _normalized_runtime_config(
        current_nominal, expected_seed_paths
    )
    expected_digest = canonical_json_digest(expected_normalized)
    packaged_runs = _packaged_run_evidence(bundle_path)
    embedded_runs = embedded_campaign_config.get("runs")
    expected_run_count = (
        embedded_runs.get("count") if isinstance(embedded_runs, Mapping) else None
    )
    if len(packaged_runs) != expected_run_count:
        raise ValueError(
            f"bundle packaged run count does not match campaign config for "
            f"{campaign.name}: expected={expected_run_count}, "
            f"observed={len(packaged_runs)}"
        )
    start_index = embedded_runs.get("start_index", 0)
    if not isinstance(start_index, int):
        raise ValueError(f"campaign start index is invalid for {campaign.name}")
    master_seed = randomization.get("master_seed")
    seed_policy = randomization.get("seed_policy")
    if not isinstance(master_seed, int) or seed_policy != "derive_all":
        raise ValueError(f"campaign randomization contract is invalid for {campaign.name}")
    observed_indices: set[int] = set()
    observed_seed_values: dict[str, set[int]] = {
        seed_path: set() for seed_path in expected_seed_paths
    }
    observed_digests: set[str] = set()
    for packaged_run in packaged_runs:
        run_name = packaged_run.run_name
        packaged_runtime = packaged_run.runtime_config
        run_manifest = packaged_run.campaign_run_manifest
        run_index = run_manifest.get("run_index")
        if not isinstance(run_index, int):
            raise ValueError(f"bundle run '{run_name}' has an invalid run index")
        if run_name != f"run_{run_index:06d}":
            raise ValueError(
                f"bundle run name/index mismatch: name={run_name}, index={run_index}"
            )
        observed_indices.add(run_index)
        derived_seeds = run_manifest.get("derived_seeds")
        if not isinstance(derived_seeds, Mapping) or set(derived_seeds) != set(
            expected_seed_paths
        ):
            raise ValueError(
                f"bundle run '{run_name}' derived-seed paths do not match campaign"
            )
        for seed_path in expected_seed_paths:
            expected_seed = derive_seed(master_seed, run_index, seed_path)
            manifest_seed = derived_seeds.get(seed_path)
            runtime_seed = _json_pointer_value(packaged_runtime, seed_path)
            if manifest_seed != expected_seed or runtime_seed != expected_seed:
                raise ValueError(
                    f"bundle run '{run_name}' seed does not match deterministic "
                    f"derivation at '{seed_path}'"
                )
            observed_seed_values[seed_path].add(expected_seed)
        observed_normalized = _normalized_runtime_config(
            packaged_runtime, expected_seed_paths
        )
        observed_digest = canonical_json_digest(observed_normalized)
        observed_digests.add(observed_digest)
        if observed_digest != expected_digest:
            raise ValueError(
                f"bundle run '{run_name}' runtime config does not match selected "
                f"nominal config for {campaign.name}: expected={expected_digest}, "
                f"observed={observed_digest}"
            )
    expected_indices = set(range(start_index, start_index + expected_run_count))
    if observed_indices != expected_indices:
        raise ValueError(
            f"bundle run indices do not match campaign for {campaign.name}: "
            f"missing={sorted(expected_indices - observed_indices)}, "
            f"unexpected={sorted(observed_indices - expected_indices)}"
        )
    duplicate_seed_paths = [
        seed_path
        for seed_path, values in observed_seed_values.items()
        if len(values) != expected_run_count
    ]
    if duplicate_seed_paths:
        raise ValueError(
            f"campaign contains duplicate derived seeds at paths "
            f"{duplicate_seed_paths}"
        )
    return {
        "normalized_nominal_sha256": expected_digest,
        "normalized_packaged_run_sha256": next(iter(observed_digests)),
        "validated_run_count": len(packaged_runs),
        "seed_paths": expected_seed_paths,
    }


def _validate_deterministic_report(
    suite: QualificationSuite,
    report: Mapping[str, object],
    current_provenance: Mapping[str, object],
) -> dict[str, object]:
    """Validate reused deterministic evidence against the selected suite contract."""
    selected = load_deterministic_regression_suite(suite.deterministic_suite)
    mismatches: dict[str, object] = {}
    if report.get("suite_name") != selected.name:
        mismatches["suite_name"] = {
            "expected": selected.name,
            "observed": report.get("suite_name"),
        }
    suite_provenance = report.get("suite")
    expected_suite_digest = _canonical_json_file_digest(suite.deterministic_suite)
    observed_suite_digest = (
        suite_provenance.get("canonical_sha256")
        if isinstance(suite_provenance, Mapping)
        else None
    )
    if observed_suite_digest != expected_suite_digest:
        mismatches["suite.canonical_sha256"] = {
            "expected": expected_suite_digest,
            "observed": observed_suite_digest,
        }

    execution = report.get("execution")
    if not isinstance(execution, Mapping):
        raise ValueError("deterministic report lacks execution provenance")
    build_manifest = execution.get("build_manifest")
    if not isinstance(build_manifest, Mapping):
        raise ValueError("deterministic report lacks build-manifest provenance")
    execution_provenance = execution.get("provenance")
    if not isinstance(execution_provenance, Mapping):
        raise ValueError("deterministic report lacks generation provenance")
    provenance_validation = _validate_current_build_provenance(
        execution_provenance,
        current_provenance,
        "deterministic report",
    )
    for field, expected in (
        ("build_type", selected.build_type),
        ("navkit_config", selected.navkit_config),
        ("generator", DEFAULT_GENERATOR),
    ):
        observed = execution.get(field)
        if observed != expected:
            mismatches[f"execution.{field}"] = {
                "expected": expected,
                "observed": observed,
            }
        manifest_observed = build_manifest.get(field)
        if manifest_observed != expected:
            mismatches[f"execution.build_manifest.{field}"] = {
                "expected": expected,
                "observed": manifest_observed,
            }

    report_cases = report.get("cases")
    if not isinstance(report_cases, list):
        raise ValueError("deterministic report lacks case evidence")
    reported_by_name: dict[str, Mapping[str, object]] = {}
    for case in report_cases:
        if not isinstance(case, Mapping) or not isinstance(case.get("name"), str):
            raise ValueError("deterministic report contains an invalid case")
        name = str(case["name"])
        if name in reported_by_name:
            raise ValueError(f"deterministic report duplicates case '{name}'")
        reported_by_name[name] = case
    expected_names = {case.name for case in selected.cases}
    if set(reported_by_name) != expected_names:
        mismatches["cases"] = {
            "expected": sorted(expected_names),
            "observed": sorted(reported_by_name),
        }

    for case in selected.cases:
        reported = reported_by_name.get(case.name)
        if reported is None:
            continue
        expected_contract = {
            "thresholds": case.thresholds,
            "minimum_duration_s": case.minimum_duration_s,
            "minimum_sample_count": case.minimum_sample_count,
            "sensor_update_counts": {
                name: {"minimum": contract.minimum, "maximum": contract.maximum}
                for name, contract in case.sensor_update_counts.items()
            },
        }
        for field, expected in expected_contract.items():
            if reported.get(field) != expected:
                mismatches[f"cases.{case.name}.{field}"] = {
                    "expected": expected,
                    "observed": reported.get(field),
                }
        scenario = reported.get("scenario")
        expected_effective_digest = canonical_json_digest(
            load_runtime_config(case.scenario)
        )
        observed_effective_digest = (
            scenario.get("effective_sha256")
            if isinstance(scenario, Mapping)
            else None
        )
        if observed_effective_digest != expected_effective_digest:
            mismatches[f"cases.{case.name}.scenario_effective_sha256"] = {
                "expected": expected_effective_digest,
                "observed": observed_effective_digest,
            }

        reported_passed = reported.get("passed")
        reported_checks = reported.get("checks")
        metrics = reported.get("metrics")
        if isinstance(metrics, Mapping):
            try:
                recomputed_passed, recomputed_checks = evaluate_truth_reconstruction(
                    metrics, case
                )
            except (KeyError, TypeError, ValueError) as error:
                mismatches[f"cases.{case.name}.metrics"] = {
                    "expected": "complete metrics matching the selected case contract",
                    "observed": str(error),
                }
            else:
                if reported_checks != recomputed_checks:
                    mismatches[f"cases.{case.name}.checks"] = {
                        "expected": recomputed_checks,
                        "observed": reported_checks,
                    }
                if reported_passed is not recomputed_passed:
                    mismatches[f"cases.{case.name}.passed"] = {
                        "expected": recomputed_passed,
                        "observed": reported_passed,
                    }
                if recomputed_passed and reported.get("return_code") != 0:
                    mismatches[f"cases.{case.name}.return_code"] = {
                        "expected": 0,
                        "observed": reported.get("return_code"),
                    }
        elif not (
            reported_passed is False
            and reported_checks == {}
            and isinstance(reported.get("error"), str)
            and bool(reported.get("error"))
        ):
            mismatches[f"cases.{case.name}.evidence"] = {
                "expected": "metrics/checks or an explicit failed-execution record",
                "observed": {
                    "metrics": metrics,
                    "checks": reported_checks,
                    "passed": reported_passed,
                    "error": reported.get("error"),
                },
            }

    case_passed_values = {
        name: case.get("passed") for name, case in reported_by_name.items()
    }
    invalid_case_results = {
        name: value
        for name, value in case_passed_values.items()
        if not isinstance(value, bool)
    }
    if invalid_case_results:
        mismatches["case_passed_values"] = {
            "expected": "one boolean passed value per deterministic case",
            "observed": invalid_case_results,
        }
    else:
        failed_count = sum(not passed for passed in case_passed_values.values())
        if report.get("failed_count") != failed_count:
            mismatches["failed_count"] = {
                "expected": failed_count,
                "observed": report.get("failed_count"),
            }
        expected_passed = failed_count == 0
        if report.get("passed") is not expected_passed:
            mismatches["passed"] = {
                "expected": expected_passed,
                "observed": report.get("passed"),
            }

    if report.get("case_count") != len(selected.cases):
        mismatches["case_count"] = {
            "expected": len(selected.cases),
            "observed": report.get("case_count"),
        }
    if mismatches:
        raise ValueError(
            "deterministic report does not match selected suite contract: "
            f"{mismatches}"
        )
    return {
        "suite_name": selected.name,
        "suite_sha256": expected_suite_digest,
        "suite_raw_sha256": file_digest(suite.deterministic_suite),
        "build_type": selected.build_type,
        "navkit_config": selected.navkit_config,
        "case_count": len(selected.cases),
        "failed_count": report.get("failed_count"),
        "passed": report.get("passed"),
        "provenance": provenance_validation,
    }


def _validate_baseline_update_request(
    tier: str,
    update_requested: bool,
    existing_baseline_tier: object | None = None,
) -> None:
    """Enforce the qualification-only baseline ownership contract."""
    if not update_requested:
        return
    if tier != "qualification":
        raise ValueError("baseline updates require tier 'qualification'")
    if existing_baseline_tier not in (None, "qualification"):
        raise ValueError(
            "an existing baseline must have tier 'qualification' before it can "
            "be updated"
        )


def _managed_baseline_check_contracts(
    suite: QualificationSuite,
) -> dict[str, dict[str, object]]:
    """Expand the selected suite into its exact qualification check contract."""
    check_kind = {
        "equivalence": "equivalence",
        "minimum": "lower_bound",
        "maximum": "upper_bound",
    }
    contracts: dict[str, dict[str, object]] = {}
    for criterion in suite.criteria:
        for campaign_name in criterion.campaigns:
            name = f"{campaign_name}.{criterion.name}"
            if name in contracts:
                raise ValueError(f"qualification suite duplicates check '{name}'")
            contracts[name] = {
                "kind": check_kind[criterion.test],
                "disposition": criterion.disposition.value,
                "confidence": criterion.confidence,
                "minimum": criterion.minimum,
                "maximum": criterion.maximum,
            }
    return contracts


def _validate_managed_baseline(
    suite: QualificationSuite,
) -> dict[str, object]:
    """Validate the checked baseline contract without executing qualification."""
    if not suite.baseline_path.is_file():
        raise ValueError(f"managed baseline does not exist: {suite.baseline_path}")
    baseline = _load_json(suite.baseline_path)
    validate_schema(
        baseline,
        QUALIFICATION_BASELINE_SCHEMA,
        str(suite.baseline_path),
    )
    metadata = baseline.get("metadata")
    checks = baseline.get("checks")
    if not isinstance(metadata, Mapping):
        raise ValueError("managed baseline metadata must be an object")
    if not isinstance(checks, Mapping):
        raise ValueError("managed baseline checks must be an object")

    expected_run_count = suite.campaign_sizes.get("qualification")
    if not isinstance(expected_run_count, int):
        raise ValueError("qualification suite must declare a qualification tier")
    expected_metadata = {
        "suite_name": suite.name,
        "suite_sha256": _canonical_json_file_digest(suite.source),
        "tier": "qualification",
        "expected_run_count": expected_run_count,
        "deterministic_suite_sha256": _canonical_json_file_digest(
            suite.deterministic_suite
        ),
    }
    mismatches: dict[str, object] = {
        f"metadata.{field}": {
            "expected": expected,
            "observed": metadata.get(field),
        }
        for field, expected in expected_metadata.items()
        if metadata.get(field) != expected
    }
    deterministic_report_sha256 = metadata.get("deterministic_report_sha256")
    if not _is_sha256(deterministic_report_sha256):
        mismatches["metadata.deterministic_report_sha256"] = {
            "expected": "64-character SHA-256 digest",
            "observed": deterministic_report_sha256,
        }

    baseline_campaigns = metadata.get("campaigns")
    reported_campaigns: dict[str, Mapping[str, object]] = {}
    if not isinstance(baseline_campaigns, list):
        mismatches["metadata.campaigns"] = {
            "expected": "one linked-input record per suite campaign",
            "observed": baseline_campaigns,
        }
    else:
        for campaign in baseline_campaigns:
            if not isinstance(campaign, Mapping) or not isinstance(
                campaign.get("name"), str
            ):
                raise ValueError(
                    "managed baseline contains an invalid campaign provenance record"
                )
            name = str(campaign["name"])
            if name in reported_campaigns:
                raise ValueError(
                    f"managed baseline duplicates campaign provenance '{name}'"
                )
            reported_campaigns[name] = campaign

    expected_campaign_names = {campaign.name for campaign in suite.campaigns}
    if set(reported_campaigns) != expected_campaign_names:
        mismatches["metadata.campaign_names"] = {
            "expected": sorted(expected_campaign_names),
            "observed": sorted(reported_campaigns),
        }
    for campaign in suite.campaigns:
        reported = reported_campaigns.get(campaign.name)
        if reported is None:
            continue
        expected_provenance = _campaign_input_provenance(campaign)
        for field in ("source_config_sha256", "nominal_config_sha256"):
            expected = expected_provenance[field]
            if reported.get(field) != expected:
                mismatches[f"metadata.campaigns.{campaign.name}.{field}"] = {
                    "expected": expected,
                    "observed": reported.get(field),
                }
        for field in ("package_fingerprint", "consistency_cache_fingerprint"):
            if not _is_sha256(reported.get(field)):
                mismatches[f"metadata.campaigns.{campaign.name}.{field}"] = {
                    "expected": "64-character SHA-256 digest",
                    "observed": reported.get(field),
                }
        evidence_origin = reported.get("evidence_origin")
        if evidence_origin not in {
            "executed_current_run",
            "reused_existing_bundle",
        }:
            mismatches[
                f"metadata.campaigns.{campaign.name}.evidence_origin"
            ] = {
                "expected": "a declared qualification evidence origin",
                "observed": evidence_origin,
            }
        generation_artifacts = reported.get("generation_artifacts")
        if not isinstance(generation_artifacts, Mapping):
            mismatches[
                f"metadata.campaigns.{campaign.name}.generation_artifacts"
            ] = {
                "expected": "source/build artifact identity",
                "observed": generation_artifacts,
            }
        if reported.get("generation_stable") is not True:
            mismatches[f"metadata.campaigns.{campaign.name}.generation_stable"] = {
                "expected": True,
                "observed": reported.get("generation_stable"),
            }

    expected_contracts = _managed_baseline_check_contracts(suite)
    if set(checks) != set(expected_contracts):
        mismatches["checks"] = {
            "expected": sorted(expected_contracts),
            "observed": sorted(checks),
        }
    active_known_finding = False
    for name, expected_contract in expected_contracts.items():
        check = checks.get(name)
        if not isinstance(check, Mapping):
            continue
        observed_contract = {
            field: check.get(field) for field in expected_contract
        }
        if observed_contract != expected_contract:
            mismatches[f"checks.{name}.contract"] = {
                "expected": expected_contract,
                "observed": observed_contract,
            }
        sample_count = check.get("sample_count")
        if (
            not isinstance(sample_count, int)
            or isinstance(sample_count, bool)
            or sample_count != expected_run_count
        ):
            mismatches[f"checks.{name}.sample_count"] = {
                "expected": expected_run_count,
                "observed": sample_count,
            }
        passed = check.get("passed")
        if not isinstance(passed, bool):
            mismatches[f"checks.{name}.passed"] = {
                "expected": "boolean",
                "observed": passed,
            }
            continue
        disposition = expected_contract["disposition"]
        numeric_values = {
            field: check.get(field)
            for field in ("mean", "ci_lower", "ci_upper")
        }
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and np.isfinite(value)
            for value in numeric_values.values()
        ):
            mismatches[f"checks.{name}.confidence_interval"] = {
                "expected": "finite numeric mean and confidence bounds",
                "observed": numeric_values,
            }
            continue
        expected_passed = qualification_check_claim_passes(
            str(expected_contract["kind"]),
            float(numeric_values["mean"]),
            float(numeric_values["ci_lower"]),
            float(numeric_values["ci_upper"]),
            expected_contract["minimum"],
            expected_contract["maximum"],
        )
        if passed is not expected_passed:
            mismatches[f"checks.{name}.claim"] = {
                "expected_passed": expected_passed,
                "observed_passed": passed,
                "mean": numeric_values["mean"],
                "ci_lower": numeric_values["ci_lower"],
                "ci_upper": numeric_values["ci_upper"],
            }
        if disposition == QualificationDisposition.REQUIRED.value and not expected_passed:
            mismatches[f"checks.{name}.required_pass"] = {
                "expected": True,
                "observed": expected_passed,
            }
        active_known_finding = active_known_finding or (
            disposition == QualificationDisposition.KNOWN_FINDING.value
            and not expected_passed
        )

    expected_status = (
        QualificationStatus.PASS_WITH_KNOWN_FINDINGS.value
        if active_known_finding
        else QualificationStatus.PASS.value
    )
    if baseline.get("qualification_status") != expected_status:
        mismatches["qualification_status"] = {
            "expected": expected_status,
            "observed": baseline.get("qualification_status"),
        }
    if mismatches:
        raise ValueError(
            "managed baseline does not match selected qualification suite: "
            f"{mismatches}"
        )
    return {
        "path": str(suite.baseline_path),
        "suite_name": suite.name,
        "tier": "qualification",
        "expected_run_count": expected_run_count,
        "campaign_count": len(suite.campaigns),
        "check_count": len(expected_contracts),
        "qualification_status": expected_status,
    }


def _apply_baseline_gate(
    report: dict[str, object],
    tier: str,
) -> None:
    """Require compatible baseline evidence only for the qualification tier."""
    baseline = report.get("baseline")
    baseline_status = (
        baseline.get("status") if isinstance(baseline, Mapping) else None
    )
    required = tier == "qualification"
    passed = not required or baseline_status in {"compared", "updated"}
    report["baseline_gate"] = {
        "required": required,
        "passed": passed,
        "baseline_status": baseline_status,
    }
    if not passed:
        report["status"] = QualificationStatus.FAIL.value
        report["passed"] = False


def _series_by_key(bundle_path: Path) -> dict[tuple[str, str], ConsistencySeries]:
    try:
        nees, nis, _ = load_consistency_cache(bundle_path)
    except ValueError:
        nees, nis, _ = refresh_consistency_cache(bundle_path)
    return {(series.kind, series.name): series for series in nees + nis}


def _resolved_campaign_window(
    suite: QualificationSuite,
    campaign: QualificationCampaignSpec,
    series_items: Mapping[tuple[str, str], ConsistencySeries],
) -> QualificationWindow:
    """Resolve one campaign window against its canonical full-INS timebase."""
    canonical = series_items.get(("nees", "full_ins"))
    if canonical is None:
        raise ValueError(
            f"{campaign.name} bundle lacks canonical full-INS NEES time history"
        )
    selection = select_qualification_window(
        canonical.time_s, suite.windows[campaign.window_name]
    )
    return QualificationWindow.absolute_seconds(selection.start_s, selection.end_s)


def _criterion_check(
    campaign: QualificationCampaignSpec,
    criterion: QualificationCriterionSpec,
    window: QualificationWindow,
    series: ConsistencySeries,
) -> tuple[QualificationCheck, dict[str, object]]:
    if criterion.metric == "normalized_window_mean":
        values = series.values / float(series.dof)
    elif criterion.metric == "acceptance_rate":
        if series.accepted is None:
            raise ValueError(
                f"{campaign.name}/{series.name} has no acceptance history"
            )
        values = np.asarray(series.accepted, dtype=float)
        selection = select_qualification_window(series.time_s, window)
        selected_values = values[:, selection.mask]
        finite_values = selected_values[np.isfinite(selected_values)]
        if not np.isin(finite_values, (0.0, 1.0)).all():
            raise ValueError(
                f"{campaign.name}/{series.name} acceptance history must contain "
                "only zero or one in the qualification window"
            )
    else:
        raise ValueError(f"unsupported qualification metric: {criterion.metric}")
    reduction = reduce_window_mean_per_run(
        series.time_s,
        values,
        window,
        require_all_finite=(
            criterion.disposition is QualificationDisposition.REQUIRED
        ),
    )
    interval = student_t_mean_confidence_interval(
        reduction.run_means, criterion.confidence
    )
    check_name = f"{campaign.name}.{criterion.name}"
    if criterion.test == "equivalence":
        if criterion.minimum is None or criterion.maximum is None:
            raise ValueError(f"equivalence criterion lacks bounds: {check_name}")
        check = evaluate_equivalence(
            check_name,
            interval,
            criterion.minimum,
            criterion.maximum,
            criterion.disposition,
        )
    elif criterion.test == "minimum":
        if criterion.minimum is None:
            raise ValueError(f"minimum criterion lacks a bound: {check_name}")
        check = evaluate_lower_bound(
            check_name, interval, criterion.minimum, criterion.disposition
        )
    elif criterion.test == "maximum":
        if criterion.maximum is None:
            raise ValueError(f"maximum criterion lacks a bound: {check_name}")
        check = evaluate_upper_bound(
            check_name, interval, criterion.maximum, criterion.disposition
        )
    else:
        raise ValueError(f"unsupported qualification test: {criterion.test}")
    details = {
        "campaign": campaign.name,
        "criterion": criterion.name,
        "kind": criterion.kind,
        "group": criterion.group,
        "metric": criterion.metric,
        "window": campaign.window_name,
        "window_start_s": reduction.window_start_s,
        "window_end_s": reduction.window_end_s,
        "epoch_count": reduction.epoch_count,
        "run_count": reduction.run_count,
        "selected_value_count": reduction.selected_value_count,
        "finite_value_count": reduction.finite_value_count,
        "finite_coverage": reduction.finite_coverage,
        "minimum_finite_epoch_count_per_run": (
            reduction.minimum_finite_epoch_count_per_run
        ),
        "rationale": criterion.rationale,
    }
    return check, details


def _cross_covariance_diagnosis(
    campaign: QualificationCampaignSpec,
    window: QualificationWindow,
    series_items: Mapping[tuple[str, str], ConsistencySeries],
) -> dict[str, object] | None:
    required = {
        name: series_items.get(("nees", name))
        for name in ("full_ins", "pva", "gyro_bias", "accel_bias")
    }
    if any(series is None for series in required.values()):
        return None
    reductions = {}
    for name, series in required.items():
        assert series is not None
        reductions[name] = reduce_window_mean_per_run(
            series.time_s,
            series.values,
            window,
            require_all_finite=True,
        ).run_means
    marginal_sum = reductions["pva"] + reductions["gyro_bias"] + reductions["accel_bias"]
    valid_denominator = marginal_sum != 0.0
    ratio = np.divide(
        reductions["full_ins"],
        marginal_sum,
        out=np.zeros_like(marginal_sum),
        where=valid_denominator,
    )[valid_denominator]
    return {
        "description": (
            "Full-state NEES divided by the sum of independently inverted PVA, "
            "gyro-bias, and accelerometer-bias marginal NEES. Departure from one "
            "is a cross-covariance diagnostic, not an exact additive decomposition."
        ),
        "mean_full_ins_nees": float(np.mean(reductions["full_ins"])),
        "mean_marginal_sum_nees": float(np.mean(marginal_sum)),
        "mean_joint_to_marginal_sum_ratio": (
            float(np.mean(ratio)) if ratio.size else None
        ),
        "median_joint_to_marginal_sum_ratio": (
            float(np.median(ratio)) if ratio.size else None
        ),
        "ratio_sample_count": int(ratio.size),
        "zero_denominator_count": int(np.count_nonzero(~valid_denominator)),
    }


def _schur_covariance_diagnosis(
    bundle_path: Path,
    campaign: QualificationCampaignSpec,
    window: QualificationWindow,
    maximum_runs: int = 64,
    maximum_epochs_per_run: int = 16,
    maximum_skipped_runs: int = 0,
) -> dict[str, object]:
    """Calculate an exact PVA/IMU-bias Schur diagnosis on sampled bundle epochs."""
    pva_indices = tuple(range(9))
    bias_indices = tuple(range(9, len(ERROR_STATE_LABELS)))
    full_values: list[float] = []
    pva_marginal_values: list[float] = []
    bias_marginal_values: list[float] = []
    bias_conditional_values: list[float] = []
    amplification_values: list[float] = []
    zero_ratio_denominator_count = 0
    skipped_run_details: list[dict[str, str]] = []
    sampled_run_count = 0
    if maximum_skipped_runs < 0:
        raise ValueError("maximum_skipped_runs must not be negative")
    with h5py.File(bundle_path, "r") as bundle:
        runs = bundle.get("runs")
        if not isinstance(runs, h5py.Group):
            raise ValueError(f"bundle has no runs group: {bundle_path}")
        run_names = sorted(runs.keys())
        if len(run_names) > maximum_runs:
            run_positions = np.linspace(
                0, len(run_names) - 1, maximum_runs, dtype=int
            )
            run_names = [run_names[position] for position in run_positions]
        sampled_run_count = len(run_names)
        for run_name in run_names:
            try:
                truth_error = runs[run_name].get("derived/truth_error")
                if not isinstance(truth_error, h5py.Group):
                    raise ValueError("missing derived/truth_error group")
                time_s = np.asarray(truth_error["time_s"][()], dtype=float)
                selection = select_qualification_window(
                    time_s, window
                )
                indices = np.flatnonzero(selection.mask)
                if indices.size > maximum_epochs_per_run:
                    selected_positions = np.linspace(
                        0, indices.size - 1, maximum_epochs_per_run, dtype=int
                    )
                    indices = indices[selected_positions]
                errors = np.column_stack(
                    [
                        np.asarray(
                            truth_error[f"error_{label}"][indices], dtype=float
                        )
                        for label in ERROR_STATE_LABELS
                    ]
                )
                covariances = np.zeros(
                    (
                        indices.size,
                        len(ERROR_STATE_LABELS),
                        len(ERROR_STATE_LABELS),
                    ),
                    dtype=float,
                )
                for row, row_label in enumerate(ERROR_STATE_LABELS):
                    for col, col_label in enumerate(
                        ERROR_STATE_LABELS[row:], start=row
                    ):
                        values = np.asarray(
                            truth_error[f"P_{row_label}__{col_label}"][indices],
                            dtype=float,
                        )
                        covariances[:, row, col] = values
                        covariances[:, col, row] = values
                decomposition = two_block_schur_nees(
                    errors, covariances, pva_indices, bias_indices
                )
            except (KeyError, np.linalg.LinAlgError, ValueError) as error:
                skipped_run_details.append(
                    {"run": run_name, "reason": str(error)}
                )
                continue
            marginal_sum = (
                decomposition.first_marginal_nees
                + decomposition.second_marginal_nees
            )
            full_values.append(float(np.mean(decomposition.full_nees) / 15.0))
            pva_marginal_values.append(
                float(np.mean(decomposition.first_marginal_nees) / 9.0)
            )
            bias_marginal_values.append(
                float(np.mean(decomposition.second_marginal_nees) / 6.0)
            )
            bias_conditional_values.append(
                float(np.mean(decomposition.second_conditional_nees) / 6.0)
            )
            valid_denominator = marginal_sum != 0.0
            zero_ratio_denominator_count += int(
                np.count_nonzero(~valid_denominator)
            )
            ratios = np.divide(
                decomposition.full_nees,
                marginal_sum,
                out=np.zeros_like(marginal_sum),
                where=valid_denominator,
            )[valid_denominator]
            if ratios.size:
                amplification_values.append(float(np.mean(ratios)))
    skipped_runs = len(skipped_run_details)
    if skipped_runs > maximum_skipped_runs:
        raise ValueError(
            "Schur covariance diagnosis rejected skipped evidence: "
            f"{skipped_runs} of {sampled_run_count} sampled runs exceeded the "
            f"allowed maximum of {maximum_skipped_runs}; "
            f"details={skipped_run_details}"
        )
    if not full_values:
        raise ValueError(f"no valid covariance samples for Schur diagnosis: {bundle_path}")
    return {
        "description": (
            "Exact full-state NEES decomposition into PVA marginal and "
            "IMU-bias conditional terms using the covariance Schur complement."
        ),
        "run_count": len(full_values),
        "sampled_run_count": sampled_run_count,
        "skipped_run_count": skipped_runs,
        "maximum_skipped_runs": maximum_skipped_runs,
        "skipped_run_details": skipped_run_details,
        "maximum_runs": maximum_runs,
        "maximum_epochs_per_run": maximum_epochs_per_run,
        "mean_normalized_full_ins_nees": float(np.mean(full_values)),
        "mean_normalized_pva_marginal_nees": float(np.mean(pva_marginal_values)),
        "mean_normalized_bias_marginal_nees": float(np.mean(bias_marginal_values)),
        "mean_normalized_bias_conditional_nees": float(
            np.mean(bias_conditional_values)
        ),
        "mean_joint_to_marginal_sum_ratio": (
            float(np.mean(amplification_values))
            if amplification_values
            else None
        ),
        "ratio_run_count": len(amplification_values),
        "zero_ratio_denominator_count": zero_ratio_denominator_count,
    }


def _check_document(
    check: QualificationCheck,
    details: Mapping[str, object],
) -> dict[str, object]:
    interval = check.confidence_interval
    return {
        **details,
        "name": check.name,
        "test": check.kind.value,
        "disposition": check.disposition.value,
        "passed": bool(check.passed),
        "estimate": interval.mean,
        "confidence": interval.confidence,
        "confidence_interval": [interval.lower, interval.upper],
        "sample_standard_deviation": interval.sample_standard_deviation,
        "standard_error": interval.standard_error,
        "minimum": check.minimum,
        "maximum": check.maximum,
    }


def _markdown_report(report: Mapping[str, object]) -> str:
    deterministic = report["deterministic"]
    assert isinstance(deterministic, Mapping)
    lines = [
        "# NavKit Qualification Report",
        "",
        f"- Suite: `{report['suite_name']}`",
        f"- Tier: `{report['tier']}`",
        f"- Overall status: **{report['status']}**",
        f"- Deterministic regression: **{'pass' if deterministic['passed'] else 'fail'}**",
        f"- Stochastic runs per campaign: {report['expected_run_count']}",
        "",
        "## Stochastic criteria",
        "",
        "| Campaign | Criterion | Disposition | Estimate | Confidence interval | Contract | Result |",
        "| --- | --- | --- | ---: | --- | --- | --- |",
    ]
    checks = report["stochastic_checks"]
    assert isinstance(checks, list)
    for check in checks:
        assert isinstance(check, Mapping)
        interval = check["confidence_interval"]
        assert isinstance(interval, list)
        if check["minimum"] is not None and check["maximum"] is not None:
            contract = f"[{float(check['minimum']):.3g}, {float(check['maximum']):.3g}]"
        elif check["minimum"] is not None:
            contract = f">= {float(check['minimum']):.3g}"
        else:
            contract = f"<= {float(check['maximum']):.3g}"
        if check["passed"]:
            result = "pass"
        elif check["disposition"] == "known_finding":
            result = "known finding"
        elif check["disposition"] == "informational":
            result = "informational finding"
        else:
            result = "fail"
        lines.append(
            f"| {check['campaign']} | {check['criterion']} | {check['disposition']} | "
            f"{float(check['estimate']):.4f} | "
            f"[{float(interval[0]):.4f}, {float(interval[1]):.4f}] | "
            f"{contract} | {result} |"
        )
    lines.extend(["", "## Known findings", ""])
    known = [
        check
        for check in checks
        if check["disposition"] == "known_finding" and not check["passed"]
    ]
    if known:
        for check in known:
            lines.append(f"- `{check['campaign']}`: {check['rationale']}")
    else:
        lines.append("- No configured known finding was active in this evidence set.")
    lines.extend(
        [
            "",
            "## PVA / IMU-bias covariance diagnosis",
            "",
            "| Campaign | Full INS | PVA marginal | Bias marginal | Bias conditional | Joint/marginal ratio |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    campaigns = report["campaigns"]
    assert isinstance(campaigns, list)
    for campaign in campaigns:
        assert isinstance(campaign, Mapping)
        diagnosis = campaign["diagnosis"]
        assert isinstance(diagnosis, Mapping)
        schur = diagnosis["schur_pva_imu_bias"]
        assert isinstance(schur, Mapping)
        ratio_value = schur["mean_joint_to_marginal_sum_ratio"]
        ratio_text = (
            f"{float(ratio_value):.3f}" if ratio_value is not None else "undefined"
        )
        lines.append(
            f"| {campaign['name']} | "
            f"{float(schur['mean_normalized_full_ins_nees']):.3f} | "
            f"{float(schur['mean_normalized_pva_marginal_nees']):.3f} | "
            f"{float(schur['mean_normalized_bias_marginal_nees']):.3f} | "
            f"{float(schur['mean_normalized_bias_conditional_nees']):.3f} | "
            f"{ratio_text} |"
        )
    lines.extend(
        [
            "",
            "The conditional term is computed with the exact Schur complement of the full covariance. It exposes joint cross-covariance inconsistency that marginal PVA or bias NEES cannot show by themselves.",
            "",
            "## Evidence",
            "",
            "| Campaign | Origin | Analysis bundle | Package fingerprint |",
            "| --- | --- | --- | --- |",
        ]
    )
    for campaign in campaigns:
        assert isinstance(campaign, Mapping)
        lines.append(
            f"| {campaign['name']} | {campaign['evidence_origin']} | "
            f"`{campaign['bundle']}` | `{campaign['package_fingerprint']}` |"
        )
    lines.extend(
        [
            "",
            "## Baseline",
            "",
            f"- Status: `{report['baseline']['status']}`",
        ]
    )
    return "\n".join(lines) + "\n"


def _evaluate(
    root: Path,
    suite: QualificationSuite,
    tier: str,
    deterministic_report_path: Path,
    campaign_root: Path,
    evidence_origin: str,
) -> tuple[QualificationReport, dict[str, object]]:
    current_git = git_provenance(root)
    selected_deterministic = load_deterministic_regression_suite(
        suite.deterministic_suite
    )
    deterministic_current_provenance = _current_execution_provenance(
        root,
        current_git,
        selected_deterministic.build_type,
        selected_deterministic.navkit_config,
        DEFAULT_GENERATOR,
    )
    deterministic = _load_json(deterministic_report_path)
    validate_schema(
        deterministic,
        DETERMINISTIC_REGRESSION_REPORT_SCHEMA,
        str(deterministic_report_path),
    )
    deterministic_validation = _validate_deterministic_report(
        suite, deterministic, deterministic_current_provenance
    )
    expected_runs = suite.campaign_sizes[tier]
    checks: list[QualificationCheck] = []
    check_documents: list[dict[str, object]] = []
    campaign_documents: list[dict[str, object]] = []
    for campaign in suite.campaigns:
        selected_campaign = _load_json(campaign.config)
        selected_execution = selected_campaign.get("execution", {})
        if not isinstance(selected_execution, Mapping):
            raise ValueError(f"campaign execution must be an object: {campaign.config}")
        campaign_current_provenance = _current_execution_provenance(
            root,
            current_git,
            str(selected_execution.get("build_type", "Release")),
            str(selected_execution.get("navkit_config", DEFAULT_NAVKIT_CONFIG)),
            str(selected_execution.get("generator", DEFAULT_GENERATOR)),
            selected_execution.get("build_dir"),
        )
        campaign_dir = campaign_root / campaign.campaign_name
        bundle_path = campaign_dir / "analysis_bundle.h5"
        if not bundle_path.is_file():
            raise ValueError(f"missing qualification bundle: {bundle_path}")
        metadata = bundle_metadata(bundle_path)
        series_items = _series_by_key(bundle_path)
        cache_provenance = consistency_cache_provenance(bundle_path)
        resolved_window = _resolved_campaign_window(suite, campaign, series_items)
        available_counts = {series.run_count for series in series_items.values()}
        if available_counts != {expected_runs}:
            raise ValueError(
                f"{campaign.name} must contain exactly {expected_runs} successful runs; "
                f"observed {sorted(available_counts)}"
            )
        campaign_checks = [
            criterion
            for criterion in suite.criteria
            if campaign.name in criterion.campaigns
        ]
        for criterion in campaign_checks:
            key = (criterion.kind, criterion.group)
            series = series_items.get(key)
            if series is None:
                raise ValueError(
                    f"{campaign.name} bundle lacks required series {criterion.kind}/{criterion.group}"
                )
            check, details = _criterion_check(
                campaign, criterion, resolved_window, series
            )
            checks.append(check)
            check_documents.append(_check_document(check, details))
        campaign_config = _validate_embedded_campaign_config(
            campaign, metadata, expected_runs, campaign_current_provenance
        )
        generation_provenance = campaign_config.get("provenance")
        if not isinstance(generation_provenance, Mapping):
            raise ValueError(f"bundle lacks generation provenance: {campaign.name}")
        generation_artifacts = _provenance_identity(
            generation_provenance,
            f"bundle '{campaign.name}'",
        )
        runtime_config_validation = _validate_bundle_runtime_config(
            campaign, bundle_path, campaign_config
        )
        input_provenance = _campaign_input_provenance(campaign)
        campaign_documents.append(
            {
                "name": campaign.name,
                "campaign_name": campaign.campaign_name,
                "source_config": str(campaign.config),
                **input_provenance,
                "bundle": str(bundle_path),
                "bundle_schema": metadata.get("schema"),
                "package_fingerprint": metadata.get("package_fingerprint"),
                "evidence_origin": evidence_origin,
                "consistency_cache": cache_provenance,
                "generation_artifacts": generation_artifacts,
                "generation_stable": generation_provenance.get(
                    "generation_stable"
                ),
                "generation_tooling": generation_provenance.get("tooling"),
                "embedded_campaign_config": campaign_config,
                "runtime_config_validation": runtime_config_validation,
                "diagnosis": {
                    "cached_joint_to_marginal": _cross_covariance_diagnosis(
                        campaign, resolved_window, series_items
                    ),
                    "schur_pva_imu_bias": _schur_covariance_diagnosis(
                        bundle_path, campaign, resolved_window
                    ),
                },
            }
        )

    stochastic_report = aggregate_qualification(checks)
    deterministic_passed = bool(deterministic.get("passed"))
    if not deterministic_passed or stochastic_report.status is QualificationStatus.FAIL:
        status = QualificationStatus.FAIL.value
    else:
        status = stochastic_report.status.value
    document: dict[str, object] = {
        "schema": QUALIFICATION_REPORT_SCHEMA,
        "suite_name": suite.name,
        "tier": tier,
        "expected_run_count": expected_runs,
        "status": status,
        "passed": status != QualificationStatus.FAIL.value,
        "deterministic": {
            "passed": deterministic_passed,
            "report": str(deterministic_report_path),
            "report_sha256": file_digest(deterministic_report_path),
            "schema": deterministic.get("schema"),
            "suite_name": deterministic.get("suite_name"),
            "suite": deterministic.get("suite"),
            "execution": deterministic.get("execution"),
            "case_count": deterministic.get("case_count"),
            "failed_count": deterministic.get("failed_count"),
            "validation": deterministic_validation,
        },
        "stochastic_status": stochastic_report.status.value,
        "stochastic_checks": check_documents,
        "campaigns": campaign_documents,
    }
    return stochastic_report, document


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run or reuse deterministic and stochastic NavKit qualification evidence."
    )
    parser.add_argument("suite", type=Path, help="Qualification-suite JSON file.")
    parser.add_argument(
        "--tier",
        default=None,
        help="Named campaign size declared by the suite (for example smoke or qualification).",
    )
    parser.add_argument(
        "--validate-baseline",
        action="store_true",
        help=(
            "Validate the configured managed baseline against the selected suite "
            "without running deterministic or Monte Carlo evidence."
        ),
    )
    parser.add_argument(
        "--reuse-campaign-root",
        type=Path,
        default=None,
        help="Reuse campaign directories under this root instead of executing Monte Carlo.",
    )
    parser.add_argument(
        "--reuse-deterministic-report",
        type=Path,
        default=None,
        help="Reuse an existing deterministic report instead of running the suite.",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--parallel-jobs", type=int, default=None)
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Atomically write the current accepted evidence as the configured baseline.",
    )
    parser.add_argument(
        "--replace-baseline",
        action="store_true",
        help="Permit --update-baseline to replace an existing baseline explicitly.",
    )
    args = parser.parse_args()
    if args.replace_baseline and not args.update_baseline:
        parser.error("--replace-baseline requires --update-baseline")
    if args.parallel_jobs is not None and args.parallel_jobs <= 0:
        parser.error("--parallel-jobs must be positive")

    root = Path(__file__).resolve().parents[1]
    suite = load_qualification_suite(args.suite)
    if args.validate_baseline:
        conflicting_options = {
            "--reuse-campaign-root": args.reuse_campaign_root is not None,
            "--reuse-deterministic-report": (
                args.reuse_deterministic_report is not None
            ),
            "--output-dir": args.output_dir is not None,
            "--parallel-jobs": args.parallel_jobs is not None,
            "--update-baseline": args.update_baseline,
            "--replace-baseline": args.replace_baseline,
        }
        active_conflicts = [
            name for name, active in conflicting_options.items() if active
        ]
        if active_conflicts:
            parser.error(
                "--validate-baseline cannot be combined with "
                + ", ".join(active_conflicts)
            )
        if args.tier not in (None, "qualification"):
            parser.error("--validate-baseline only accepts --tier qualification")
        try:
            validation = _validate_managed_baseline(suite)
        except ValueError as error:
            print(f"Managed baseline validation failed: {error}", file=sys.stderr)
            return 1
        print(f"Managed baseline valid: {validation['path']}")
        print(
            f"Contract: {validation['campaign_count']} campaigns, "
            f"{validation['check_count']} checks, "
            f"{validation['expected_run_count']} runs"
        )
        return 0
    if args.tier is None:
        parser.error("--tier is required unless --validate-baseline is selected")
    if args.tier not in suite.campaign_sizes:
        parser.error(
            f"unknown tier '{args.tier}'; choices are {sorted(suite.campaign_sizes)}"
        )
    try:
        _validate_baseline_update_request(args.tier, args.update_baseline)
    except ValueError as error:
        parser.error(str(error))
    output_dir = _resolve_inside_repository(
        root,
        args.output_dir
        if args.output_dir is not None
        else suite.output_root / args.tier,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_report in (
        output_dir / "qualification_report.json",
        output_dir / "qualification_report.md",
    ):
        stale_report.unlink(missing_ok=True)
    if args.reuse_campaign_root is None:
        campaign_output = output_dir / "campaigns"
        if campaign_output.is_dir() and any(campaign_output.iterdir()):
            parser.error(
                f"campaign output is not empty: {campaign_output}; choose a new "
                "--output-dir or remove the stale evidence explicitly"
            )

    deterministic_report_path = (
        args.reuse_deterministic_report.resolve()
        if args.reuse_deterministic_report is not None
        else _execute_deterministic_suite(suite, output_dir)
    )
    evidence_origin = (
        "reused_existing_bundle"
        if args.reuse_campaign_root is not None
        else "executed_current_run"
    )
    campaign_root = (
        args.reuse_campaign_root.resolve()
        if args.reuse_campaign_root is not None
        else _execute_campaigns(
            suite, args.tier, output_dir, args.parallel_jobs
        )
    )
    stochastic_report, report = _evaluate(
        root,
        suite,
        args.tier,
        deterministic_report_path,
        campaign_root,
        evidence_origin,
    )
    report["generated_utc"] = datetime.now(timezone.utc).isoformat()
    report["host"] = {"platform": platform.platform(), "python": sys.version}
    report["source"] = {
        "suite": str(suite.source),
        "suite_sha256": _canonical_json_file_digest(suite.source),
        "suite_raw_sha256": file_digest(suite.source),
        "git": git_provenance(root),
    }

    baseline_document: dict[str, object] = {"status": "missing", "deltas": []}
    if suite.baseline_path.is_file():
        baseline = _load_json(suite.baseline_path)
        validate_schema(
            baseline,
            QUALIFICATION_BASELINE_SCHEMA,
            str(suite.baseline_path),
        )
        metadata = baseline.get("metadata")
        baseline_tier = metadata.get("tier") if isinstance(metadata, Mapping) else None
        try:
            _validate_baseline_update_request(
                args.tier, args.update_baseline, baseline_tier
            )
        except ValueError as error:
            parser.error(str(error))
        baseline_suite_digest = (
            metadata.get("suite_sha256") if isinstance(metadata, Mapping) else None
        )
        baseline_deterministic_digest = (
            metadata.get("deterministic_suite_sha256")
            if isinstance(metadata, Mapping)
            else None
        )
        baseline_campaigns = (
            metadata.get("campaigns") if isinstance(metadata, Mapping) else None
        )
        current_suite_digest = _canonical_json_file_digest(suite.source)
        current_deterministic_digest = _canonical_json_file_digest(
            suite.deterministic_suite
        )
        current_campaigns = {
            str(campaign["name"]): {
                "source_config_sha256": campaign["source_config_sha256"],
                "nominal_config_sha256": campaign["nominal_config_sha256"],
            }
            for campaign in report["campaigns"]
            if isinstance(campaign, Mapping)
        }
        baseline_campaign_inputs = {
            str(campaign["name"]): {
                "source_config_sha256": campaign.get("source_config_sha256"),
                "nominal_config_sha256": campaign.get("nominal_config_sha256"),
            }
            for campaign in baseline_campaigns
            if isinstance(campaign, Mapping) and "name" in campaign
        } if isinstance(baseline_campaigns, list) else {}
        if baseline_tier != "qualification":
            baseline_document = {
                "status": "incompatible_tier",
                "path": str(suite.baseline_path),
                "baseline_tier": baseline_tier,
                "required_baseline_tier": "qualification",
                "deltas": [],
            }
        elif args.tier != "qualification":
            baseline_document = {
                "status": "not_applicable_tier",
                "path": str(suite.baseline_path),
                "baseline_tier": baseline_tier,
                "evidence_tier": args.tier,
                "deltas": [],
            }
        elif baseline_suite_digest != current_suite_digest:
            baseline_document = {
                "status": "incompatible_suite",
                "path": str(suite.baseline_path),
                "baseline_suite_sha256": baseline_suite_digest,
                "current_suite_sha256": current_suite_digest,
                "deltas": [],
            }
        elif (
            baseline_deterministic_digest != current_deterministic_digest
            or baseline_campaign_inputs != current_campaigns
        ):
            baseline_document = {
                "status": "incompatible_linked_inputs",
                "path": str(suite.baseline_path),
                "baseline_deterministic_suite_sha256": baseline_deterministic_digest,
                "current_deterministic_suite_sha256": current_deterministic_digest,
                "baseline_campaign_inputs": baseline_campaign_inputs,
                "current_campaign_inputs": current_campaigns,
                "deltas": [],
            }
        else:
            deltas = qualification_baseline_deltas(stochastic_report, baseline)
            baseline_document = {
                "status": "compared",
                "path": str(suite.baseline_path),
                "deltas": [asdict(delta) for delta in deltas],
            }
    report["baseline"] = baseline_document

    if args.update_baseline and bool(report["passed"]):
        campaign_provenance: list[dict[str, object]] = []
        for campaign in report["campaigns"]:
            if not isinstance(campaign, Mapping):
                raise ValueError("qualification report contains invalid campaign evidence")
            cache = campaign.get("consistency_cache")
            if not isinstance(cache, Mapping) or not _is_sha256(
                cache.get("fingerprint")
            ):
                raise ValueError(
                    f"campaign '{campaign.get('name')}' lacks validated consistency-cache provenance"
                )
            if campaign.get("generation_stable") is not True:
                raise ValueError(
                    f"campaign '{campaign.get('name')}' was not generated under a "
                    "verified stable artifact/tooling window"
                )
            campaign_provenance.append(
                {
                    "name": campaign["name"],
                    "source_config_sha256": campaign["source_config_sha256"],
                    "nominal_config_sha256": campaign["nominal_config_sha256"],
                    "package_fingerprint": campaign["package_fingerprint"],
                    "consistency_cache_fingerprint": cache["fingerprint"],
                    "evidence_origin": campaign["evidence_origin"],
                    "generation_artifacts": campaign["generation_artifacts"],
                    "generation_stable": True,
                }
            )
        write_qualification_baseline(
            suite.baseline_path,
            stochastic_report,
            {
                "suite_name": suite.name,
                "suite_sha256": _canonical_json_file_digest(suite.source),
                "suite_raw_sha256": file_digest(suite.source),
                "tier": args.tier,
                "generated_utc": report["generated_utc"],
                "expected_run_count": suite.campaign_sizes[args.tier],
                "deterministic_suite_sha256": _canonical_json_file_digest(
                    suite.deterministic_suite
                ),
                "deterministic_suite_raw_sha256": file_digest(
                    suite.deterministic_suite
                ),
                "deterministic_report_sha256": file_digest(
                    deterministic_report_path
                ),
                "campaigns": campaign_provenance,
                "git": report["source"]["git"],
            },
            replace_existing=args.replace_baseline,
        )
        report["baseline"] = {
            "status": "updated",
            "path": str(suite.baseline_path),
            "deltas": [],
        }
    elif args.update_baseline:
        report["baseline"] = {
            "status": "update_rejected",
            "path": str(suite.baseline_path),
            "reason": "qualification has a deterministic or required stochastic failure",
            "deltas": [],
        }

    _apply_baseline_gate(report, args.tier)

    report_path = output_dir / "qualification_report.json"
    markdown_path = output_dir / "qualification_report.md"
    report["artifacts"] = {
        "json_report": str(report_path),
        "markdown_report": str(markdown_path),
        "deterministic_report": str(deterministic_report_path),
        "campaign_root": str(campaign_root),
    }
    _write_json(report_path, report)
    markdown_path.write_text(_markdown_report(report), encoding="utf-8")
    print(f"Qualification status: {report['status']}")
    print(f"Report: {report_path}")
    return 0 if bool(report["passed"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
