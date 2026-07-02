# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from perf_artifacts import (
    DEFAULT_RUN_NAME,
    DEFAULT_TIMING_PATH,
    print_command_timing,
    timing_result,
    update_timing_artifact,
    utc_now,
)


def run(cmd: list[str], cwd: Path) -> int:
    print("+", " ".join(cmd))
    return subprocess.call(cmd, cwd=cwd)


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
        "--timing-output",
        type=Path,
        default=DEFAULT_TIMING_PATH,
        help="timing.json path to update with this test duration.",
    )
    parser.add_argument("--timing-run-name", default=DEFAULT_RUN_NAME)
    parser.add_argument(
        "--no-timing",
        action="store_true",
        help="Do not update the timing artifact for this test run.",
    )
    parser.add_argument(
        "--no-timing-report",
        action="store_true",
        help="Update timing.json without printing the test timing summary.",
    )
    args = parser.parse_args()
    start_utc = utc_now()
    start = time.perf_counter()

    root = Path(__file__).resolve().parents[1]
    build_dir = args.build_dir if args.build_dir is not None else root / "build" / args.build_type
    if not build_dir.is_absolute():
        build_dir = root / build_dir

    if not build_dir.exists():
        print(f"Build directory does not exist: {build_dir}")
        print("Run: python tools/build.py --build-type", args.build_type)
        return 1

    build_manifest = load_build_manifest(build_dir)
    navkit_config = str(build_manifest.get("navkit_config", "unknown"))

    # Avoid Conan-generated CMake preset names such as conan-release/conan-debug.
    # Those can collide with repository presets. Drive CTest directly from the
    # generated build tree instead.
    cmd = [
        "ctest",
        "--test-dir",
        str(build_dir),
        "--build-config",
        args.build_type,
        "--output-on-failure",
    ]

    return_code = run(cmd, cwd=root)
    command_name = f"tests_{args.build_type.lower()}"
    if not args.no_timing:
        record = update_timing_artifact(
            args.timing_output,
            run_name=args.timing_run_name,
            command_name=command_name,
            command=[Path(sys.executable).name, *sys.argv],
            result=timing_result(start_utc, start, return_code),
            build_type=args.build_type,
            navkit_config=navkit_config,
            tool_version="run_tests.py",
        )
        if not args.no_timing_report:
            print()
            print_command_timing(command_name, record)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
