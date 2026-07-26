# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Summarize and convert NavKit profiling CSV exports."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Iterable


PROFILE_SCHEMA = "navkit.profile.v1"
PROFILE_RUN_MANIFEST_SCHEMA = "navkit.profile_run_manifest.v1"
BUILD_MANIFEST_SCHEMA = "navkit.build_manifest.v1"


@dataclass(frozen=True)
class ProfileRecord:
    point_id: int
    point: str
    start_tick: int
    elapsed_ticks: int
    sequence: int
    parent_sequence: int
    depth: int
    flags: int


def default_profile_run_manifest_path(csv_path: Path) -> Path:
    return csv_path.with_name("profile_run_manifest.json")


def load_json_manifest(path: Path, expected_schema: str, label: str) -> dict[str, object]:
    document = json.loads(path.read_text(encoding="utf-8"))
    schema = document.get("schema", "")
    if schema != expected_schema:
        raise ValueError(f"Unsupported {label} schema '{schema}' in {path}")
    return document


def load_profile_run_manifest(path: Path) -> dict[str, object]:
    return load_json_manifest(path, PROFILE_RUN_MANIFEST_SCHEMA, "profile run manifest")


def load_build_manifest(path: Path) -> dict[str, object]:
    return load_json_manifest(path, BUILD_MANIFEST_SCHEMA, "build manifest")


def load_profile_csv(path: Path) -> list[ProfileRecord]:
    records: list[ProfileRecord] = []

    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            schema = row.get("schema", "")
            if schema != PROFILE_SCHEMA:
                raise ValueError(f"Unsupported profile schema '{schema}' in {path}")

            records.append(
                ProfileRecord(
                    point_id=int(row["point_id"]),
                    point=row["point"],
                    start_tick=int(row["start_tick"]),
                    elapsed_ticks=int(row["elapsed_ticks"]),
                    sequence=int(row["sequence"]),
                    parent_sequence=int(row["parent_sequence"]),
                    depth=int(row["depth"]),
                    flags=int(row["flags"]),
                )
            )

    return records


def profiling_metadata(build_manifest: dict[str, object]) -> dict[str, object]:
    metadata = build_manifest.get("compiletime_config_metadata", {})
    if not isinstance(metadata, dict):
        return {}
    profiling = metadata.get("profiling", {})
    if not isinstance(profiling, dict):
        return {}
    return profiling


def resolve_tick_period_us(build_manifest: dict[str, object], override: float | None) -> float:
    if override is not None:
        return override

    tick_period = profiling_metadata(build_manifest).get("tick_period_us")
    if isinstance(tick_period, (int, float)):
        return float(tick_period)

    raise ValueError(
        "Profile trace export requires profiling.tick_period_us in navkit_build_manifest.json "
        "or an explicit --tick-period-us diagnostic override."
    )


def percentile(values: list[int], percent: float) -> float:
    if not values:
        return 0.0

    ordered = sorted(values)
    rank = (len(ordered) - 1) * percent
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def summarize(records: Iterable[ProfileRecord]) -> list[dict[str, float | int | str]]:
    grouped: dict[str, list[int]] = defaultdict(list)
    for record in records:
        grouped[record.point].append(record.elapsed_ticks)

    total_ticks = sum(sum(values) for values in grouped.values())
    rows: list[dict[str, float | int | str]] = []

    for point, values in sorted(grouped.items()):
        point_total = sum(values)
        rows.append(
            {
                "point": point,
                "count": len(values),
                "total_ticks": point_total,
                "min_ticks": min(values),
                "max_ticks": max(values),
                "mean_ticks": mean(values),
                "p95_ticks": percentile(values, 0.95),
                "p99_ticks": percentile(values, 0.99),
                "percent_profiled_time": (100.0 * point_total / total_ticks) if total_ticks else 0.0,
            }
        )

    return rows


def print_summary(rows: Iterable[dict[str, float | int | str]]) -> None:
    print("Profile summary")
    print("point,count,total_ticks,min_ticks,max_ticks,mean_ticks,p95_ticks,p99_ticks,percent")
    for row in rows:
        print(
            f"{row['point']},{row['count']},{row['total_ticks']},{row['min_ticks']},"
            f"{row['max_ticks']},{row['mean_ticks']:.3f},{row['p95_ticks']:.3f},"
            f"{row['p99_ticks']:.3f},{row['percent_profiled_time']:.2f}"
        )


def write_chrome_trace(
    records: Iterable[ProfileRecord],
    path: Path,
    tick_period_us: float,
    build_manifest: dict[str, object],
    run_manifest: dict[str, object],
) -> None:
    profiling = profiling_metadata(build_manifest)
    events = []
    for record in records:
        events.append(
            {
                "name": record.point,
                "cat": "navkit.profile",
                "ph": "X",
                "ts": record.start_tick * tick_period_us,
                "dur": record.elapsed_ticks * tick_period_us,
                "pid": 0,
                "tid": 0,
                "args": {
                    "schema": PROFILE_SCHEMA,
                    "point_id": record.point_id,
                    "sequence": record.sequence,
                    "parent_sequence": record.parent_sequence,
                    "depth": record.depth,
                    "flags": record.flags,
                },
            }
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "traceEvents": events,
                "displayTimeUnit": "us",
                "metadata": {
                    "schema": PROFILE_SCHEMA,
                    "build_manifest_schema": build_manifest.get("schema"),
                    "profile_run_manifest_schema": run_manifest.get("schema"),
                    "selected_config": build_manifest.get("navkit_config"),
                    "clock_source": profiling.get("clock_source"),
                    "tick_period_us": tick_period_us,
                    "record_count": run_manifest.get("record_count"),
                    "dropped_record_count": run_manifest.get("dropped_record_count"),
                    "sink_capacity": profiling.get("sink_capacity"),
                    "sink_overflow_policy": profiling.get("sink_overflow_policy"),
                },
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="NavKit profile CSV export")
    parser.add_argument(
        "--profile-run-manifest",
        type=Path,
        default=None,
        help="Profile run manifest JSON. Defaults to profile_run_manifest.json beside the CSV.",
    )
    parser.add_argument(
        "--build-manifest",
        type=Path,
        required=False,
        help="Build manifest JSON containing compile-time profiling metadata.",
    )
    parser.add_argument("--chrome-trace", type=Path, help="Write Chrome Trace / Perfetto JSON")
    parser.add_argument(
        "--tick-period-us",
        type=float,
        default=None,
        help="Override manifest tick-to-microsecond conversion for trace output.",
    )
    parser.add_argument("--no-summary", action="store_true", help="Do not print the text summary")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = load_profile_csv(args.csv)

    if not args.no_summary:
        print_summary(summarize(records))

    if args.chrome_trace is not None:
        profile_run_manifest_path = args.profile_run_manifest or default_profile_run_manifest_path(args.csv)
        if args.build_manifest is None:
            raise ValueError("--build-manifest is required when writing Chrome Trace JSON.")

        build_manifest = load_build_manifest(args.build_manifest)
        run_manifest = load_profile_run_manifest(profile_run_manifest_path)
        tick_period_us = resolve_tick_period_us(build_manifest, args.tick_period_us)
        write_chrome_trace(records, args.chrome_trace, tick_period_us, build_manifest, run_manifest)
        print(f"Wrote Chrome Trace JSON: {args.chrome_trace}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
