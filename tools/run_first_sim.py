# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import platform
import subprocess
from pathlib import Path

from perf_artifacts import (
    DEFAULT_NAVKIT_CONFIG,
    measure_call,
    print_command_timing,
    update_timing_artifact,
)


def default_exe(build_type: str) -> Path:
    base = Path("build") / build_type / "apps" / "navkit_sim"
    if platform.system() == "Windows":
        candidate = base / build_type / "navkit_sim.exe"
        if candidate.exists():
            return candidate
        return base / "navkit_sim.exe"
    return base / "navkit_sim"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument(
        "--config", type=Path, default=Path("config/runtime/navkit_sim/stationary_gnss.json")
    )
    parser.add_argument("--navkit-config", default=DEFAULT_NAVKIT_CONFIG)
    parser.add_argument(
        "--no-timing-report",
        action="store_true",
        help="Update timing.json without printing the simulation timing summary.",
    )
    args = parser.parse_args()

    runtime_config = json.loads(args.config.read_text(encoding="utf-8"))
    run_name = runtime_config.get("run_name", "stationary_gnss_demo")
    output_dir = Path(runtime_config.get("output_dir", f"data/logs/{run_name}"))
    timing_path = output_dir / "timing.json"

    exe = default_exe(args.build_type)
    command = [str(exe), str(args.config)]
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
        navkit_config=args.navkit_config,
        tool_version="run_first_sim.py",
    )
    if not args.no_timing_report:
        print()
        print_command_timing(command_name, record)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
