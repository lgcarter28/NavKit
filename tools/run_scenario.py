# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from navkit_build_dirs import DEFAULT_GENERATOR
from perf_artifacts import DEFAULT_NAVKIT_CONFIG
from runtime_config import (
    apply_runtime_overrides,
    load_runtime_config,
    runtime_output_dir,
    runtime_run_name,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a NavKit runtime scenario and generate standard plots."
    )
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Build directory override forwarded to run_sim.py.",
    )
    parser.add_argument("--generator", default=DEFAULT_GENERATOR, help="CMake generator used.")
    parser.add_argument(
        "--navkit-config",
        default=DEFAULT_NAVKIT_CONFIG,
        help=(
            "Compile-time config header used to locate the matching build tree/executable. "
            "This does not switch runtime behavior of an already-built executable."
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
            "Override runtime output_dir for this run. The effective runtime JSON is written "
            "to the selected output directory."
        ),
    )
    parser.add_argument(
        "--run-name",
        default=None,
        help="Override runtime run_name. Defaults to the output directory name when overridden.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Only run the sim; do not invoke plot_run.py.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Forward --show to plot_run.py when plots are enabled.",
    )
    parser.add_argument(
        "--no-profile-report",
        action="store_true",
        help="Forward --no-profile-report to run_sim.py.",
    )
    parser.add_argument(
        "--no-profile-trace",
        action="store_true",
        help="Forward --no-profile-trace to run_sim.py.",
    )
    args = parser.parse_args()

    tools_dir = Path(__file__).resolve().parent
    runtime_config = apply_runtime_overrides(
        load_runtime_config(args.config), output_dir=args.output_dir, run_name=args.run_name
    )
    output_dir = runtime_output_dir(runtime_config)
    run_name = runtime_run_name(runtime_config)

    run_sim_command = [
        sys.executable,
        str(tools_dir / "run_sim.py"),
        "--build-type",
        args.build_type,
        "--generator",
        args.generator,
        "--navkit-config",
        args.navkit_config,
        "--config",
        str(args.config),
    ]
    if args.build_dir is not None:
        run_sim_command.extend(["--build-dir", str(args.build_dir)])
    if args.output_dir is not None:
        run_sim_command.extend(["--output-dir", str(output_dir)])
    if args.run_name is not None or args.output_dir is not None:
        run_sim_command.extend(["--run-name", run_name])
    if args.no_profile_report:
        run_sim_command.append("--no-profile-report")
    if args.no_profile_trace:
        run_sim_command.append("--no-profile-trace")

    sim_result = subprocess.run(run_sim_command, check=False)
    if sim_result.returncode != 0 or args.no_plot:
        return sim_result.returncode

    plot_command = [sys.executable, str(tools_dir / "plot_run.py"), str(output_dir)]
    if args.show:
        plot_command.append("--show")
    return subprocess.run(plot_command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
