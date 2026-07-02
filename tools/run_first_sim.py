# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import platform
import subprocess
from pathlib import Path

from perf_artifacts import (
    measure_call,
    print_command_timing,
    update_timing_artifact,
)
from profile_report import load_profile_csv, print_summary, summarize, write_chrome_trace


def default_build_dir(build_type: str) -> Path:
    return Path("build") / build_type


def default_exe(build_dir: Path, build_type: str) -> Path:
    base = build_dir / "apps" / "navkit_sim"
    if platform.system() == "Windows":
        candidate = base / build_type / "navkit_sim.exe"
        if candidate.exists():
            return candidate
        return base / "navkit_sim.exe"
    return base / "navkit_sim"


def load_build_manifest(build_dir: Path) -> dict[str, object]:
    manifest_path = build_dir / "navkit_build_manifest.json"
    if not manifest_path.exists():
        return {}
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Build directory. Defaults to build/<build-type>.",
    )
    parser.add_argument(
        "--config", type=Path, default=Path("config/runtime/navkit_sim/stationary_gnss.json")
    )
    parser.add_argument(
        "--no-timing-report",
        action="store_true",
        help="Update timing.json without printing the simulation timing summary.",
    )
    parser.add_argument(
        "--no-profile-report",
        action="store_true",
        help="Do not print embedded profile summaries when profile.csv is produced.",
    )
    parser.add_argument(
        "--no-profile-trace",
        action="store_true",
        help="Do not write profile.trace.json when profile.csv is produced.",
    )
    args = parser.parse_args()

    build_dir = args.build_dir or default_build_dir(args.build_type)
    build_manifest = load_build_manifest(build_dir)
    navkit_config = str(build_manifest.get("navkit_config", "unknown"))

    runtime_config = json.loads(args.config.read_text(encoding="utf-8"))
    run_name = runtime_config.get("run_name", "stationary_gnss_demo")
    output_dir = Path(runtime_config.get("output_dir", f"data/logs/{run_name}"))
    timing_path = output_dir / "timing.json"

    for generated_profile_artifact in (output_dir / "profile.csv", output_dir / "profile.trace.json"):
        if generated_profile_artifact.exists():
            generated_profile_artifact.unlink()

    exe = default_exe(build_dir, args.build_type)
    command = [str(exe), str(args.config)]
    print(f"Build config: {navkit_config}")
    print(f"Running {' '.join(command)}")

    return_code, result = measure_call(subprocess.call, command)
    command_name = "stationary_simulation"
    record = update_timing_artifact(
        timing_path,
        run_name=run_name,
        command_name=command_name,
        command=command,
        result=result,
        build_type=args.build_type,
        navkit_config=navkit_config,
        tool_version="run_first_sim.py",
    )
    if not args.no_timing_report:
        print()
        print_command_timing(command_name, record)

    profile_path = output_dir / "profile.csv"
    if return_code == 0 and profile_path.exists():
        records = load_profile_csv(profile_path)

        if not args.no_profile_report:
            print()
            print_summary(summarize(records))

        if not args.no_profile_trace:
            trace_path = output_dir / "profile.trace.json"
            write_chrome_trace(records, trace_path, tick_period_us=1.0)
            print(f"Wrote Chrome Trace JSON: {trace_path}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
