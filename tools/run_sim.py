# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import platform
import subprocess
from pathlib import Path

from navkit_build_dirs import DEFAULT_GENERATOR, resolve_build_dir
from perf_artifacts import (
    DEFAULT_NAVKIT_CONFIG,
    measure_call,
    print_command_timing,
    update_timing_artifact,
)
from profile_report import (
    default_profile_run_manifest_path,
    load_build_manifest as load_profile_build_manifest,
    load_profile_csv,
    load_profile_run_manifest,
    print_summary,
    resolve_tick_period_us,
    summarize,
    write_chrome_trace,
)
from runtime_config import (
    apply_runtime_overrides,
    load_runtime_config,
    runtime_output_dir,
    runtime_run_name,
    write_effective_runtime_config,
)


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


def build_manifest_path(build_dir: Path) -> Path:
    return build_dir / "navkit_build_manifest.json"


def remove_stale_profile_artifacts(output_dir: Path) -> bool:
    ok = True
    for generated_profile_artifact in (
        output_dir / "profile.csv",
        output_dir / "profile.trace.json",
        output_dir / "profile_run_manifest.json",
    ):
        if not generated_profile_artifact.exists():
            continue
        try:
            generated_profile_artifact.unlink()
        except PermissionError:
            print(
                "Profile artifact is locked; close viewers/editors before rerunning: "
                f"{generated_profile_artifact}"
            )
            ok = False
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the configured NavKit simulation app.")
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help=(
            "Build directory. Defaults to "
            "build/<build-type-lower>/<navkit-config-without-.hpp>."
        ),
    )
    parser.add_argument("--generator", default=DEFAULT_GENERATOR, help="CMake generator used.")
    parser.add_argument(
        "--navkit-config",
        default=DEFAULT_NAVKIT_CONFIG,
        help=(
            "Compile-time config header relative to config/compiletime. This locates the "
            "matching build tree/executable; it does not switch a compiled executable at runtime."
        ),
    )
    parser.add_argument(
        "--config", type=Path, default=Path("config/runtime/navkit_sim/ecef_ins_gnss.json")
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Override the runtime config output_dir. Writes an effective_runtime_config.json "
            "into the selected output directory and runs the sim with that file."
        ),
    )
    parser.add_argument(
        "--run-name",
        default=None,
        help=(
            "Override the runtime config run_name. Defaults to the output directory name when "
            "--output-dir is provided."
        ),
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

    root = Path(__file__).resolve().parents[1]
    build_dir = resolve_build_dir(
        root, args.build_type, args.navkit_config, args.build_dir, generator=args.generator
    )
    build_manifest = load_build_manifest(build_dir)
    navkit_config = str(build_manifest.get("navkit_config", "unknown"))

    runtime_config = apply_runtime_overrides(
        load_runtime_config(args.config), output_dir=args.output_dir, run_name=args.run_name
    )
    run_name = runtime_run_name(runtime_config)
    output_dir = runtime_output_dir(runtime_config)
    data_dir = output_dir / "data"
    timing_path = data_dir / "timing.json"
    data_dir.mkdir(parents=True, exist_ok=True)

    runtime_config_path = args.config
    if args.output_dir is not None or args.run_name is not None:
        runtime_config_path = write_effective_runtime_config(runtime_config, output_dir)

    if not remove_stale_profile_artifacts(data_dir):
        return 1

    exe = default_exe(build_dir, args.build_type)
    command = [str(exe), str(runtime_config_path)]
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
        tool_version="run_sim.py",
    )
    if not args.no_timing_report:
        print()
        print_command_timing(command_name, record)

    profile_path = data_dir / "profile.csv"
    if return_code == 0 and profile_path.exists():
        records = load_profile_csv(profile_path)
        run_manifest = load_profile_run_manifest(default_profile_run_manifest_path(profile_path))
        profile_build_manifest = load_profile_build_manifest(build_manifest_path(build_dir))

        if not args.no_profile_report:
            print()
            print_summary(summarize(records))

        if not args.no_profile_trace:
            trace_path = output_dir / "profile.trace.json"
            tick_period_us = resolve_tick_period_us(profile_build_manifest, override=None)
            write_chrome_trace(
                records, trace_path, tick_period_us, profile_build_manifest, run_manifest
            )
            print(f"Wrote Chrome Trace JSON: {trace_path}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
