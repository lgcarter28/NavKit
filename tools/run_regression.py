# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Run deterministic NavKit regression suites and emit compact evidence."""

from __future__ import annotations

import argparse
import json
import platform
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from internal.navkit_build_dirs import DEFAULT_GENERATOR, resolve_build_dir
from internal.runtime_config import load_runtime_config
from navkit_analysis.analysis_performance import canonical_json_digest, file_digest
from navkit_analysis.regression import (
    DeterministicRegressionCase,
    evaluate_truth_reconstruction,
    load_deterministic_regression_suite,
    load_truth_reconstruction_metrics,
)
from navkit_analysis.schema import DETERMINISTIC_REGRESSION_REPORT_SCHEMA


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(f"{path.suffix}.tmp")
    temporary_path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(path)


def _resolve_output_root(root: Path, candidate: Path) -> Path:
    output_root = candidate if candidate.is_absolute() else root / candidate
    resolved = output_root.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"regression output must stay inside the repository: {candidate}")
    return resolved


def _unique_artifact_directory(parent: Path, case_name: str) -> Path:
    candidate = parent / case_name
    suffix = 1
    while candidate.exists():
        candidate = parent / f"{case_name}_{suffix:02d}"
        suffix += 1
    return candidate


def _selected_cases(
    cases: tuple[DeterministicRegressionCase, ...], selected_names: list[str]
) -> tuple[DeterministicRegressionCase, ...]:
    if not selected_names:
        return cases
    requested = set(selected_names)
    available = {case.name for case in cases}
    unknown = sorted(requested - available)
    if unknown:
        raise ValueError(f"unknown regression case names: {unknown}")
    return tuple(case for case in cases if case.name in requested)


def _scenario_provenance(case: DeterministicRegressionCase) -> dict[str, object]:
    effective_config = load_runtime_config(case.scenario)
    return {
        "path": str(case.scenario),
        "source_sha256": file_digest(case.scenario),
        "effective_sha256": canonical_json_digest(effective_config),
    }


def _run_case(
    *,
    case: DeterministicRegressionCase,
    work_parent: Path,
    artifacts_parent: Path,
    build_type: str,
    navkit_config: str,
    generator: str,
    build_dir: Path | None,
    retain_artifacts: bool,
) -> dict[str, object]:
    tools_dir = Path(__file__).resolve().parent
    case_started_s = time.perf_counter()
    result: dict[str, Any] = {
        "name": case.name,
        "scenario": _scenario_provenance(case),
        "thresholds": case.thresholds,
        "minimum_duration_s": case.minimum_duration_s,
        "minimum_sample_count": case.minimum_sample_count,
        "sensor_update_counts": {
            sensor_name: {
                "minimum": contract.minimum,
                "maximum": contract.maximum,
            }
            for sensor_name, contract in case.sensor_update_counts.items()
        },
    }

    with tempfile.TemporaryDirectory(prefix=f"{case.name}_", dir=work_parent) as temporary:
        run_dir = Path(temporary) / "run"
        command = [
            sys.executable,
            str(tools_dir / "run_sim.py"),
            "--build-type",
            build_type,
            "--generator",
            generator,
            "--navkit-config",
            navkit_config,
            "--config",
            str(case.scenario),
            "--output-dir",
            str(run_dir),
            "--run-name",
            f"regression_{case.name}",
            "--no-timing-report",
            "--no-profile-report",
            "--no-profile-trace",
        ]
        if build_dir is not None:
            command.extend(["--build-dir", str(build_dir)])

        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "regression_stdout.txt").write_text(
            completed.stdout, encoding="utf-8"
        )
        (run_dir / "regression_stderr.txt").write_text(
            completed.stderr, encoding="utf-8"
        )
        result["command"] = command
        result["return_code"] = completed.returncode

        passed = False
        if completed.returncode != 0:
            result["error"] = "simulation returned a nonzero exit status"
            result["checks"] = {}
        else:
            try:
                metrics = load_truth_reconstruction_metrics(run_dir)
                passed, checks = evaluate_truth_reconstruction(metrics, case)
                result["metrics"] = metrics
                result["checks"] = checks
            except (KeyError, OSError, ValueError) as error:
                result["error"] = str(error)
                result["checks"] = {}

        result["passed"] = passed
        preserve = retain_artifacts or not passed
        if preserve:
            artifact_dir = _unique_artifact_directory(artifacts_parent, case.name)
            artifact_dir.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(run_dir, artifact_dir)
            result["artifact_directory"] = str(artifact_dir)
        else:
            result["artifact_directory"] = None

    result["elapsed_s"] = time.perf_counter() - case_started_s
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a versioned deterministic NavKit regression suite."
    )
    parser.add_argument("suite", type=Path, help="Regression-suite JSON file.")
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Run only this named case. May be supplied more than once.",
    )
    parser.add_argument("--build-type", choices=["Debug", "Release"], default=None)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--generator", default=DEFAULT_GENERATOR)
    parser.add_argument(
        "--navkit-config",
        default=None,
        help="Override the suite's compile-time product selection.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Override the suite's regression-output root.",
    )
    parser.add_argument(
        "--retain-artifacts",
        action="store_true",
        help="Retain complete run logs for passing cases as well as failures.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    suite = load_deterministic_regression_suite(args.suite)
    build_type = args.build_type or suite.build_type
    navkit_config = args.navkit_config or suite.navkit_config
    output_root = _resolve_output_root(
        root, args.output_dir if args.output_dir is not None else suite.output_root
    )
    suite_output = output_root
    suite_output.mkdir(parents=True, exist_ok=True)
    report_path = suite_output / "report.json"
    report_path.unlink(missing_ok=True)
    cases = _selected_cases(suite.cases, args.case)
    work_parent = suite_output / ".work"
    artifacts_parent = suite_output / "artifacts"
    work_parent.mkdir(parents=True, exist_ok=True)

    resolved_build_dir = resolve_build_dir(
        root,
        build_type,
        navkit_config,
        args.build_dir,
        generator=args.generator,
    )
    build_manifest_path = resolved_build_dir / "navkit_build_manifest.json"
    if not build_manifest_path.is_file():
        raise FileNotFoundError(
            f"missing build manifest in {resolved_build_dir}; build the selected product first"
        )
    loaded_manifest = json.loads(build_manifest_path.read_text(encoding="utf-8"))
    if not isinstance(loaded_manifest, dict):
        raise ValueError(f"build manifest root must be an object: {build_manifest_path}")
    build_manifest: dict[str, object] = loaded_manifest
    selected_manifest_config = build_manifest.get("navkit_config")
    if selected_manifest_config != navkit_config:
        raise ValueError(
            f"build directory selects '{selected_manifest_config}', not '{navkit_config}'"
        )
    manifest_build_type = build_manifest.get("build_type")
    if manifest_build_type != build_type:
        raise ValueError(
            f"build directory contains '{manifest_build_type}', not '{build_type}'"
        )

    started = datetime.now(timezone.utc)
    print(f"Regression suite: {suite.name}")
    print(f"Build: {build_type} ({navkit_config})")
    results: list[dict[str, object]] = []
    for case in cases:
        case_result = _run_case(
            case=case,
            work_parent=work_parent,
            artifacts_parent=artifacts_parent,
            build_type=build_type,
            navkit_config=navkit_config,
            generator=args.generator,
            build_dir=args.build_dir,
            retain_artifacts=args.retain_artifacts,
        )
        results.append(case_result)
        status = "PASS" if bool(case_result["passed"]) else "FAIL"
        print(f"  {status} {case.name} ({float(case_result['elapsed_s']):.3f} s)")

    try:
        work_parent.rmdir()
    except OSError:
        pass

    passed = all(bool(result["passed"]) for result in results)
    report = {
        "schema": DETERMINISTIC_REGRESSION_REPORT_SCHEMA,
        "suite_name": suite.name,
        "suite": {
            "path": str(suite.source),
            "sha256": file_digest(suite.source),
        },
        "execution": {
            "started_utc": started.isoformat(),
            "finished_utc": datetime.now(timezone.utc).isoformat(),
            "host_platform": platform.platform(),
            "python": sys.version,
            "build_type": build_type,
            "navkit_config": navkit_config,
            "generator": args.generator,
            "build_directory": str(resolved_build_dir),
            "build_manifest": build_manifest,
        },
        "case_count": len(results),
        "passed_count": sum(bool(result["passed"]) for result in results),
        "failed_count": sum(not bool(result["passed"]) for result in results),
        "passed": passed,
        "cases": results,
    }
    _write_json(report_path, report)
    print(f"Report: {report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
