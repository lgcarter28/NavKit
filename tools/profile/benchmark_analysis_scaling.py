# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Measure repeatable HDF5-backed Monte Carlo analysis scaling."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


BENCHMARK_SCHEMA = "navkit.analysis_scaling_benchmark.v1"


@dataclass(frozen=True)
class StageResult:
    """One measured public-tool invocation."""

    elapsed_s: float
    command: list[str]


@dataclass(frozen=True)
class PreparedBundle:
    """A benchmark-owned, cache-warmed copy of one immutable source bundle."""

    path: Path
    copy_elapsed_s: float
    consistency_cache_warmup_elapsed_s: float


def parse_bundle(value: str) -> tuple[str, Path]:
    """Parse a required `<label>=<analysis_bundle.h5>` benchmark input."""
    label, separator, path_string = value.partition("=")
    if not separator or not label or not path_string:
        raise argparse.ArgumentTypeError("--bundle must use <label>=<analysis_bundle.h5>")
    path = Path(path_string)
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"analysis bundle does not exist: {path}")
    return label, path.resolve()


def run_stage(command: list[str]) -> StageResult:
    """Run one public plotting command and return its wall-clock duration."""
    print("+", subprocess.list2cmdline(command), flush=True)
    start = time.perf_counter()
    subprocess.run(command, check=True)
    return StageResult(elapsed_s=time.perf_counter() - start, command=command)


def prepare_bundle(
    label: str,
    source_bundle: Path,
    output_root: Path,
    consistency_tool: Path,
    max_plot_points: int,
) -> PreparedBundle:
    """Copy and prewarm one bundle without mutating the archived source input."""
    workspace = output_root / "prepared_bundles" / label
    workspace.mkdir(parents=True, exist_ok=True)
    copied_bundle = workspace / "analysis_bundle.h5"
    copy_start = time.perf_counter()
    shutil.copy2(source_bundle, copied_bundle)
    copy_elapsed_s = time.perf_counter() - copy_start
    warmup_command = [
        sys.executable,
        str(consistency_tool),
        str(copied_bundle),
        "--refresh-cache",
        "--max-plot-points",
        str(max_plot_points),
        "--output-dir",
        str(workspace / "cache_warmup"),
    ]
    warmup = run_stage(warmup_command)
    return PreparedBundle(
        path=copied_bundle,
        copy_elapsed_s=copy_elapsed_s,
        consistency_cache_warmup_elapsed_s=warmup.elapsed_s,
    )


def main() -> int:
    """Benchmark aggregate and consistency rendering at selected worker counts."""
    parser = argparse.ArgumentParser(
        description=(
            "Measure HDF5-backed Monte Carlo aggregate and consistency rendering with "
            "identical public-tool workloads at multiple worker counts."
        )
    )
    parser.add_argument(
        "--bundle",
        type=parse_bundle,
        action="append",
        required=True,
        help="Named analysis bundle as <label>=<analysis_bundle.h5>; may be repeated.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        action="append",
        default=[1, 2, 4],
        help="Plotly worker count to benchmark; may be repeated (default: 1, 2, 4).",
    )
    parser.add_argument(
        "--stage",
        choices=("aggregate", "consistency"),
        action="append",
        default=None,
        help="Benchmark only this stage; may be repeated (default: both stages).",
    )
    parser.add_argument(
        "--reuse-prepared-bundles",
        action="store_true",
        help="Reuse cache-warmed copies already present under --output-root.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("output/analysis_benchmarks/phase_6_9_scaling"),
        help="Root for isolated generated benchmark artifacts and the summary JSON.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=1000,
        help="Common aggregate and consistency plotting time-grid limit.",
    )
    args = parser.parse_args()
    if any(worker <= 0 for worker in args.workers):
        parser.error("--workers must be positive")
    if args.max_plot_points <= 1:
        parser.error("--max-plot-points must be greater than one")

    worker_counts = list(dict.fromkeys(args.workers))
    selected_stages = tuple(args.stage or ("aggregate", "consistency"))
    tool_root = Path(__file__).resolve().parents[1]
    aggregate_tool = tool_root / "plot_monte_carlo.py"
    consistency_tool = tool_root / "plot_consistency.py"
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    measurements: list[dict[str, object]] = []
    preparation: dict[str, dict[str, object]] = {}

    for label, source_bundle in args.bundle:
        existing_prepared_path = output_root / "prepared_bundles" / label / "analysis_bundle.h5"
        if args.reuse_prepared_bundles:
            if not existing_prepared_path.is_file():
                parser.error(f"prepared bundle does not exist: {existing_prepared_path}")
            prepared_bundle = PreparedBundle(
                path=existing_prepared_path,
                copy_elapsed_s=0.0,
                consistency_cache_warmup_elapsed_s=0.0,
            )
        else:
            prepared_bundle = prepare_bundle(
                label,
                source_bundle,
                output_root,
                consistency_tool,
                args.max_plot_points,
            )
        preparation[label] = {
            "source_bundle": str(source_bundle),
            "prepared_bundle": str(prepared_bundle.path),
            "copy_elapsed_s": prepared_bundle.copy_elapsed_s,
            "consistency_cache_warmup_elapsed_s": prepared_bundle.consistency_cache_warmup_elapsed_s,
        }
        for worker_count in worker_counts:
            destination = output_root / label / f"workers_{worker_count}"
            aggregate_command = [
                sys.executable,
                str(aggregate_tool),
                str(prepared_bundle.path),
                "--renderer",
                "plotly",
                "--parallel-jobs",
                str(worker_count),
                "--max-plot-points",
                str(args.max_plot_points),
                "--output-dir",
                str(destination / "aggregate"),
                "--force",
            ]
            consistency_command = [
                sys.executable,
                str(consistency_tool),
                str(prepared_bundle.path),
                "--parallel-jobs",
                str(worker_count),
                "--output-dir",
                str(destination / "consistency"),
                "--force",
            ]
            aggregate = run_stage(aggregate_command) if "aggregate" in selected_stages else None
            consistency = (
                run_stage(consistency_command)
                if "consistency" in selected_stages
                else None
            )
            measurements.append(
                {
                    "label": label,
                    "bundle": str(prepared_bundle.path),
                    "workers": worker_count,
                    "aggregate": None if aggregate is None else asdict(aggregate),
                    "consistency": None if consistency is None else asdict(consistency),
                    "total_elapsed_s": sum(
                        result.elapsed_s
                        for result in (aggregate, consistency)
                        if result is not None
                    ),
                }
            )

    report = {
        "schema": BENCHMARK_SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "max_plot_points": args.max_plot_points,
        "stages": selected_stages,
        "preparation": preparation,
        "measurements": measurements,
    }
    suffix = "" if len(selected_stages) == 2 else f"_{selected_stages[0]}"
    report_path = output_root / f"analysis_scaling_benchmark{suffix}.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote analysis scaling benchmark: {report_path}")
    for measurement in measurements:
        stages = [
            f"{name}={measurement[name]['elapsed_s']:.3f} s"
            for name in selected_stages
            if measurement[name] is not None
        ]
        print(
            f"{measurement['label']}, workers={measurement['workers']}: "
            f"{', '.join(stages)}, total={measurement['total_elapsed_s']:.3f} s"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
