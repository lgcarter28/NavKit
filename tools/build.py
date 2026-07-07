# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import json
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

from navkit_build_dirs import repo_root_from_tools_file
from navkit_build_dirs import resolve_build_dir as resolve_config_build_dir
from perf_artifacts import (
    DEFAULT_NAVKIT_CONFIG,
    DEFAULT_RUN_NAME,
    DEFAULT_TIMING_PATH,
    default_resource_report_path,
    load_resource_report,
    print_command_timing,
    print_resource_report,
    timing_result,
    update_timing_artifact,
    utc_now,
    write_resource_report,
)


def repo_root() -> Path:
    """Return the repository root when this script is run from tools/build.py."""
    return repo_root_from_tools_file(__file__)


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(str(c) for c in cmd))
    subprocess.check_call(cmd, cwd=cwd)


def find_executable_near_python(name: str) -> str:
    venv_dir = Path(__file__).resolve().parents[1] / ".venv"
    candidates = [
        venv_dir / "Scripts" / f"{name}.exe",
        venv_dir / "bin" / name,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    found = shutil.which(name)
    if found:
        return found

    raise FileNotFoundError(f"Could not find executable: {name}")


def find_conan_toolchain(build_dir: Path) -> Path:
    candidates = [
        build_dir / "build" / "generators" / "conan_toolchain.cmake",
        build_dir / "generators" / "conan_toolchain.cmake",
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    searched = "\n".join(f"  - {p}" for p in candidates)
    raise FileNotFoundError(
        "Could not find Conan-generated conan_toolchain.cmake. Searched:\n"
        f"{searched}"
    )


def read_cmake_cache_value(build_dir: Path, key: str) -> str | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.exists():
        return None

    for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
        prefix = f"{key}:"
        if line.startswith(prefix):
            _, value = line.split("=", maxsplit=1)
            return value

    return None


def selected_navkit_config(build_dir: Path, navkit_config_arg: str | None) -> str:
    if navkit_config_arg:
        return navkit_config_arg

    cached = read_cmake_cache_value(build_dir, "NAVKIT_CONFIG")
    if cached:
        return cached

    manifest_path = build_dir / "navkit_build_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest_config = manifest.get("navkit_config")
        if isinstance(manifest_config, str) and manifest_config:
            return manifest_config

    return DEFAULT_NAVKIT_CONFIG


def navkit_sim_executable(build_dir: Path, build_type: str) -> Path:
    base = build_dir / "apps" / "navkit_sim"
    if platform.system() == "Windows":
        candidate = base / build_type / "navkit_sim.exe"
        if candidate.exists():
            return candidate
        return base / "navkit_sim.exe"
    return base / "navkit_sim"


def query_compiletime_config_metadata(build_dir: Path, build_type: str) -> dict[str, object]:
    executable = navkit_sim_executable(build_dir, build_type)
    if not executable.exists():
        return {}

    result = subprocess.run(
        [str(executable), "--describe-config"],
        check=True,
        capture_output=True,
        text=True,
    )
    document = json.loads(result.stdout)
    schema = document.get("schema", "")
    if schema != "navkit.compiletime_config_metadata.v1":
        raise ValueError(
            f"Unsupported compile-time config metadata schema '{schema}' from {executable}"
        )
    return document


def write_build_manifest(
    build_dir: Path,
    *,
    build_type: str,
    navkit_config: str,
    compiletime_config_metadata: dict[str, object],
    tests_enabled: bool,
    warnings_as_errors: bool,
    coverage_enabled: bool,
) -> Path:
    manifest_path = build_dir / "navkit_build_manifest.json"
    manifest = {
        "schema": "navkit.build_manifest.v1",
        "build_type": build_type,
        "navkit_config": navkit_config,
        "compiletime_config_metadata": compiletime_config_metadata,
        "tests_enabled": tests_enabled,
        "warnings_as_errors": warnings_as_errors,
        "coverage_enabled": coverage_enabled,
        "updated_utc": utc_now(),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure and build NavKit.")
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help=(
            "Build directory. Defaults to build/<build-type>/<navkit-config-without-.hpp>."
        ),
    )
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--skip-conan", action="store_true")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--without-tests", action="store_true", help="Configure without test targets.")
    parser.add_argument(
        "--warnings-as-errors",
        action="store_true",
        help="Treat warnings in NavKit-owned C++ targets as errors.",
    )
    parser.add_argument(
        "--coverage",
        action="store_true",
        help="Enable coverage instrumentation for supported Debug builds.",
    )
    parser.add_argument("--jobs", "-j", type=int, default=None)
    parser.add_argument(
        "--navkit-config",
        default=None,
        help="Compile-time config header relative to config/compiletime.",
    )
    parser.add_argument(
        "--timing-output",
        type=Path,
        default=DEFAULT_TIMING_PATH,
        help="timing.json path to update with this build duration.",
    )
    parser.add_argument("--timing-run-name", default=DEFAULT_RUN_NAME)
    parser.add_argument(
        "--no-timing",
        action="store_true",
        help="Do not update the timing artifact for this build.",
    )
    parser.add_argument(
        "--no-timing-report",
        action="store_true",
        help="Update timing.json without printing the build timing summary.",
    )
    parser.add_argument(
        "--resource-output",
        type=Path,
        default=None,
        help="Resource report output path. Defaults to data/logs/stationary_gnss_demo/resources-<build>-local.json.",
    )
    parser.add_argument(
        "--no-resource-report",
        action="store_true",
        help="Do not write or print the build artifact size report.",
    )
    args = parser.parse_args()
    start_utc = utc_now()
    start = time.perf_counter()

    root = repo_root()
    requested_navkit_config = args.navkit_config or DEFAULT_NAVKIT_CONFIG
    build_dir = resolve_config_build_dir(
        root, args.build_type, requested_navkit_config, args.build_dir
    )

    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)

    build_dir.mkdir(parents=True, exist_ok=True)

    if args.build_only and args.clean:
        raise ValueError("--clean cannot be used with --build-only")

    if args.coverage and args.build_type != "Debug":
        raise ValueError("--coverage is only supported with --build-type Debug")

    if not args.build_only and not args.skip_conan:
        conan = find_executable_near_python("conan")
        run(
            [
                conan,
                "install",
                ".",
                "--output-folder",
                str(build_dir),
                "--build=missing",
                "-s",
                f"build_type={args.build_type}",
            ],
            cwd=root,
        )

    if not args.build_only:
        toolchain_file = find_conan_toolchain(build_dir)
        run(
            [
                "cmake",
                "-S",
                str(root),
                "-B",
                str(build_dir),
                f"-DCMAKE_TOOLCHAIN_FILE={toolchain_file}",
                f"-DCMAKE_BUILD_TYPE={args.build_type}",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                f"-DNAVKIT_BUILD_TESTS={'OFF' if args.without_tests else 'ON'}",
                f"-DNAVKIT_WARNINGS_AS_ERRORS={'ON' if args.warnings_as_errors else 'OFF'}",
                f"-DNAVKIT_ENABLE_COVERAGE={'ON' if args.coverage else 'OFF'}",
                f"-DNAVKIT_CONFIG={requested_navkit_config}",
            ],
            cwd=root,
        )

    build_cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        args.build_type,
    ]

    if args.jobs is not None:
        build_cmd += ["--parallel", str(args.jobs)]

    run(build_cmd, cwd=root)
    navkit_config = selected_navkit_config(build_dir, requested_navkit_config)
    compiletime_config_metadata = query_compiletime_config_metadata(build_dir, args.build_type)
    write_build_manifest(
        build_dir,
        build_type=args.build_type,
        navkit_config=navkit_config,
        compiletime_config_metadata=compiletime_config_metadata,
        tests_enabled=not args.without_tests,
        warnings_as_errors=args.warnings_as_errors,
        coverage_enabled=args.coverage,
    )

    command_name = f"build_{args.build_type.lower()}"
    if not args.no_timing:
        record = update_timing_artifact(
            args.timing_output,
            run_name=args.timing_run_name,
            command_name=command_name,
            command=[Path(sys.executable).name, *sys.argv],
            result=timing_result(start_utc, start, 0),
            build_type=args.build_type,
            navkit_config=navkit_config,
            tool_version="build.py",
        )
        if not args.no_timing_report:
            print()
            print_command_timing(command_name, record)

    if not args.no_resource_report:
        resource_output = args.resource_output or default_resource_report_path(args.build_type)
        write_resource_report(
            resource_output,
            build_dir=build_dir,
            build_type=args.build_type,
            navkit_config=navkit_config,
        )
        print()
        print_resource_report(load_resource_report(resource_output), max_artifacts=8)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
