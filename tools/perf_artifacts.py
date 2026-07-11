# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import json
import platform
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

TIMING_SCHEMA = "navkit.timing.v1"
RESOURCE_SCHEMA = "navkit.resources.v1"
DEFAULT_TIMING_PATH = Path("output/logs/stationary_gnss_demo/timing.json")
DEFAULT_RESOURCE_DIR = Path("output/logs/stationary_gnss_demo")
DEFAULT_RUN_NAME = "stationary_gnss_demo"
DEFAULT_NAVKIT_CONFIG = "apps/navkit_sim/StationaryGnss.hpp"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def measure_call(func: Any, *args: Any, **kwargs: Any) -> tuple[int, dict[str, Any]]:
    start_utc = utc_now()
    start = time.perf_counter()
    return_code = int(func(*args, **kwargs))
    return return_code, timing_result(start_utc, start, return_code)


def timing_result(start_utc: str, start: float, return_code: int) -> dict[str, Any]:
    return {
        "start_utc": start_utc,
        "end_utc": utc_now(),
        "elapsed_s": time.perf_counter() - start,
        "return_code": return_code,
    }


def update_timing_artifact(
    timing_path: Path,
    *,
    run_name: str,
    command_name: str,
    command: list[str],
    result: dict[str, Any],
    build_type: str | None = None,
    navkit_config: str | None = None,
    tool_version: str | None = None,
) -> dict[str, Any]:
    timing_path.parent.mkdir(parents=True, exist_ok=True)

    if timing_path.exists():
        document = json.loads(timing_path.read_text(encoding="utf-8"))
    else:
        document = {
            "schema": TIMING_SCHEMA,
            "run_name": run_name,
            "created_utc": utc_now(),
            "environment": {
                "platform": platform.platform(),
                "python": platform.python_version(),
            },
            "build": {},
            "commands": {},
        }

    document["schema"] = TIMING_SCHEMA
    document["run_name"] = run_name
    document["updated_utc"] = utc_now()

    build = document.setdefault("build", {})
    if build_type is not None:
        build["build_type"] = build_type
    if navkit_config is not None:
        build["navkit_config"] = navkit_config

    record = {
        "command": command,
        "tool_version": tool_version,
        **result,
    }
    if build_type is not None:
        record["build_type"] = build_type
    if navkit_config is not None:
        record["navkit_config"] = navkit_config
    document.setdefault("commands", {})[command_name] = record

    timing_path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    return record


def load_timing_artifact(timing_path: Path) -> dict[str, Any]:
    document = json.loads(timing_path.read_text(encoding="utf-8"))
    schema = document.get("schema")
    if schema != TIMING_SCHEMA:
        raise ValueError(f"Expected schema {TIMING_SCHEMA!r}, found {schema!r}")
    return document


def format_seconds(value: Any) -> str:
    if not isinstance(value, int | float):
        return "n/a"
    return f"{value:.3f} s"


def print_command_timing(command_name: str, record: dict[str, Any]) -> None:
    print(f"{command_name}: {format_seconds(record.get('elapsed_s'))}")
    print(f"Return: {record.get('return_code', 'n/a')}")
    if record.get("build_type") is not None:
        print(f"Build: {record['build_type']}")
    if record.get("navkit_config") is not None:
        print(f"Config: {record['navkit_config']}")


def print_timing_report(document: dict[str, Any]) -> None:
    run_name = document.get("run_name", "<unknown>")
    build = document.get("build", {})
    environment = document.get("environment", {})
    commands = document.get("commands", {})

    print(f"NavKit timing: {run_name}")
    print(f"Schema: {document.get('schema')}")
    print(f"Config: {build.get('navkit_config', 'n/a')}")
    print(f"Latest build metadata: {build.get('build_type', 'n/a')}")
    print(f"Platform: {environment.get('platform', 'n/a')}")
    print(f"Python: {environment.get('python', 'n/a')}")
    print()

    if not isinstance(commands, dict) or not commands:
        print("No command timing records found.")
        return

    name_width = max(len(name) for name in commands)
    print(f"{'Command'.ljust(name_width)}  Elapsed   Return  Build    Tool")
    print(f"{'-' * name_width}  --------  ------  -------  ----")

    total_elapsed_s = 0.0
    for name, record in commands.items():
        elapsed = record.get("elapsed_s")
        if isinstance(elapsed, int | float):
            total_elapsed_s += elapsed
        return_code = record.get("return_code", "n/a")
        build_type = record.get("build_type", "n/a")
        tool_version = record.get("tool_version", "n/a")
        print(
            f"{name.ljust(name_width)}  {format_seconds(elapsed).rjust(8)}  "
            f"{str(return_code).rjust(6)}  {str(build_type).ljust(7)}  {tool_version}"
        )

    print(f"{'-' * name_width}  --------  ------  -------  ----")
    print(f"{'total_recorded'.ljust(name_width)}  {format_seconds(total_elapsed_s).rjust(8)}")


def default_resource_report_path(build_type: str) -> Path:
    return DEFAULT_RESOURCE_DIR / f"resources-{build_type.lower()}-local.json"


def load_resource_report(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    schema = document.get("schema")
    if schema != RESOURCE_SCHEMA:
        raise ValueError(f"Expected schema {RESOURCE_SCHEMA!r}, found {schema!r}")
    return document


def print_resource_report(document: dict[str, Any], *, max_artifacts: int | None = None) -> None:
    build = document.get("build", {})
    artifacts = document.get("artifacts", [])

    print("NavKit resource report")
    print(f"Schema: {document.get('schema')}")
    print(f"Build: {build.get('build_type', 'n/a')}")
    print(f"Build dir: {build.get('build_dir', 'n/a')}")
    print(f"Config: {document.get('navkit_config', 'n/a')}")
    print()

    if not isinstance(artifacts, list) or not artifacts:
        print("No executable/library artifacts found.")
        return

    sorted_artifacts = sorted(artifacts, key=lambda item: item.get("bytes", 0), reverse=True)
    if max_artifacts is not None:
        sorted_artifacts = sorted_artifacts[:max_artifacts]

    path_width = max(len(str(item.get("path", ""))) for item in sorted_artifacts)
    print(f"{'Artifact'.ljust(path_width)}  Size")
    print(f"{'-' * path_width}  ----------")
    for item in sorted_artifacts:
        size_bytes = item.get("bytes", 0)
        print(f"{str(item.get('path', '')).ljust(path_width)}  {format_bytes(size_bytes)}")


def format_bytes(value: Any) -> str:
    if not isinstance(value, int | float):
        return "n/a"
    units = ["B", "KiB", "MiB", "GiB"]
    size = float(value)
    for unit in units:
        if abs(size) < 1024.0 or unit == units[-1]:
            return f"{size:.1f} {unit}"
        size /= 1024.0


def write_resource_report(
    output_path: Path,
    *,
    build_dir: Path,
    build_type: str,
    navkit_config: str | None = None,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    artifacts = []
    artifact_names = {
        "navkit_sim",
        "navkit_sim.exe",
        "navkit_replay",
        "navkit_replay.exe",
        "navkit_tests",
        "navkit_tests.exe",
        "libnavkit_sim.a",
        "navkit_sim.lib",
    }

    if build_dir.exists():
        for path in sorted(build_dir.rglob("*")):
            if path.is_file() and path.name in artifact_names:
                artifacts.append(
                    {
                        "path": path.relative_to(build_dir).as_posix(),
                        "bytes": path.stat().st_size,
                    }
                )

    report = {
        "schema": RESOURCE_SCHEMA,
        "generated_utc": utc_now(),
        "build": {
            "build_type": build_type,
            "build_dir": build_dir.as_posix(),
        },
        "navkit_config": navkit_config,
        "artifacts": artifacts,
    }

    output_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
