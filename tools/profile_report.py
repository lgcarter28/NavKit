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


def write_chrome_trace(records: Iterable[ProfileRecord], path: Path, tick_period_us: float) -> None:
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
                    "tick_period_us": tick_period_us,
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
    parser.add_argument("--chrome-trace", type=Path, help="Write Chrome Trace / Perfetto JSON")
    parser.add_argument(
        "--tick-period-us",
        type=float,
        default=1.0,
        help="Tick-to-microsecond conversion for trace output; defaults to 1 tick = 1 us",
    )
    parser.add_argument("--no-summary", action="store_true", help="Do not print the text summary")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = load_profile_csv(args.csv)

    if not args.no_summary:
        print_summary(summarize(records))

    if args.chrome_trace is not None:
        write_chrome_trace(records, args.chrome_trace, args.tick_period_us)
        print(f"Wrote Chrome Trace JSON: {args.chrome_trace}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
