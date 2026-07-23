# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from navkit_analysis.schema import MONTE_CARLO_REPORT_SCHEMA, validate_schema


def load_json(path: Path) -> dict[str, Any]:
    parsed = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError(f"JSON artifact '{path}' must contain an object")
    return parsed


def campaign_dir_from_input(path: Path) -> Path:
    if path.is_file():
        if path.name == "monte_carlo_summary.json" and path.parent.name == "reports":
            return path.parent.parent.parent
        return path.parent
    return path


def report_path_from_campaign_dir(campaign_dir: Path) -> Path:
    return campaign_dir / "summary" / "reports" / "monte_carlo_summary.json"


def optional_number(mapping: dict[str, Any], key: str) -> int | float | None:
    value = mapping.get(key)
    if isinstance(value, (int, float)):
        return value
    return None


def nested_optional_number(mapping: dict[str, Any], *keys: str) -> int | float | None:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    if isinstance(current, (int, float)):
        return current
    return None


def write_csv(path: Path, rows: list[dict[str, Any]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return path


def markdown_value(value: object, precision: int = 6) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{precision}g}"
    return str(value)


def campaign_name(campaign_dir: Path, report: dict[str, Any]) -> str:
    name = report.get("campaign_name")
    if isinstance(name, str) and name:
        return name
    manifest_path = campaign_dir / "campaign_manifest.json"
    if manifest_path.exists():
        manifest = load_json(manifest_path)
        name = manifest.get("campaign_name")
        if isinstance(name, str) and name:
            return name
    return campaign_dir.name


def collect_campaign_rows(inputs: list[Path]) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
]:
    summary_rows: list[dict[str, Any]] = []
    axis_rows: list[dict[str, Any]] = []
    group_rows: list[dict[str, Any]] = []
    nis_rows: list[dict[str, Any]] = []

    for input_path in inputs:
        campaign_dir = campaign_dir_from_input(input_path)
        report_path = (
            input_path
            if input_path.is_file() and input_path.name == "monte_carlo_summary.json"
            else report_path_from_campaign_dir(campaign_dir)
        )
        report = load_json(report_path)
        validate_schema(report, MONTE_CARLO_REPORT_SCHEMA, str(report_path))
        name = campaign_name(campaign_dir, report)
        summary_rows.append(
            {
                "campaign": name,
                "campaign_dir": str(campaign_dir),
                "run_count": report.get("run_count"),
                "passed_count": report.get("passed_count"),
                "failed_count": report.get("failed_count"),
                "successful_aggregate_count": report.get("successful_aggregate_count"),
                "runner_elapsed_mean_s": nested_optional_number(
                    report, "timing_summary", "runner_elapsed_s", "mean"
                ),
                "simulation_elapsed_mean_s": nested_optional_number(
                    report, "timing_summary", "simulation_elapsed_s", "mean"
                ),
                "output_bytes_total": nested_optional_number(
                    report, "output_size_summary", "output_bytes", "sum"
                ),
                "output_bytes_mean": nested_optional_number(
                    report, "output_size_summary", "output_bytes", "mean"
                ),
                "data_bytes_total": nested_optional_number(
                    report, "output_size_summary", "data_bytes", "sum"
                ),
                "data_bytes_mean": nested_optional_number(
                    report, "output_size_summary", "data_bytes", "mean"
                ),
            }
        )
        for row in report.get("state_axis_metrics", []):
            if isinstance(row, dict):
                axis_rows.append({"campaign": name, **row})
        for row in report.get("state_group_metrics", []):
            if isinstance(row, dict):
                group_rows.append({"campaign": name, **row})
        for row in report.get("nis_metrics", []):
            if isinstance(row, dict):
                nis_rows.append({"campaign": name, **row})

    return summary_rows, axis_rows, group_rows, nis_rows


def write_markdown_report(output_dir: Path, summary_rows: list[dict[str, Any]]) -> Path:
    path = output_dir / "monte_carlo_comparison.md"
    lines = [
        "# Monte Carlo Campaign Comparison",
        "",
        "| Campaign | Runs | Passed | Failed | Mean run time [s] | Mean output [bytes] |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        lines.append(
            "| "
            f"{row['campaign']} | "
            f"{markdown_value(row['run_count'])} | "
            f"{markdown_value(row['passed_count'])} | "
            f"{markdown_value(row['failed_count'])} | "
            f"{markdown_value(row['runner_elapsed_mean_s'], 4)} | "
            f"{markdown_value(row['output_bytes_mean'], 4)} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare existing NavKit Monte Carlo aggregate reports."
    )
    parser.add_argument(
        "campaign",
        type=Path,
        nargs="+",
        help="Campaign output directory or summary/reports/monte_carlo_summary.json.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/monte_carlo_comparison"),
        help="Directory for comparison CSV and Markdown outputs.",
    )
    args = parser.parse_args()

    summary_rows, axis_rows, group_rows, nis_rows = collect_campaign_rows(args.campaign)
    output_dir = args.output_dir
    write_csv(output_dir / "campaign_summary.csv", summary_rows)
    write_csv(output_dir / "state_axis_metrics.csv", axis_rows)
    write_csv(output_dir / "state_group_metrics.csv", group_rows)
    write_csv(output_dir / "nis_metrics.csv", nis_rows)
    report_path = write_markdown_report(output_dir, summary_rows)
    print(f"Wrote Monte Carlo comparison report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
