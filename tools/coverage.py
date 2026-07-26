# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from navkit_build_dirs import default_build_dir


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(str(c) for c in cmd))
    subprocess.check_call(cmd, cwd=cwd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Build, test, and report NavKit coverage.")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Coverage build directory. Defaults to build/coverage/<navkit-config-without-.hpp>.",
    )
    parser.add_argument(
        "--navkit-config",
        default="apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp",
        help="Compile-time config header relative to config/compiletime.",
    )
    parser.add_argument(
        "--html",
        action="store_true",
        help="Also generate an HTML coverage report.",
    )
    args = parser.parse_args()

    root = repo_root()
    if args.build_dir is None:
        build_dir = default_build_dir(root, "coverage", args.navkit_config)
    else:
        build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir

    gcovr = shutil.which("gcovr")
    if gcovr is None:
        print("ERROR: gcovr not found. Install gcovr to generate coverage reports.")
        return 1

    run(
        [
            sys.executable,
            "tools/build.py",
            "--build-type",
            "Debug",
            "--build-dir",
            str(build_dir),
            "--clean",
            "--coverage",
            "--warnings-as-errors",
            "--navkit-config",
            args.navkit_config,
        ],
        cwd=root,
    )
    run(
        [
            sys.executable,
            "tools/run_tests.py",
            "--build-type",
            "Debug",
            "--build-dir",
            str(build_dir),
            "--navkit-config",
            args.navkit_config,
        ],
        cwd=root,
    )

    report_dir = build_dir / "coverage"
    report_dir.mkdir(parents=True, exist_ok=True)
    xml_report = report_dir / "coverage.xml"
    text_report = report_dir / "coverage.txt"

    gcovr_cmd = [
        gcovr,
        "--root",
        str(root),
        "--filter",
        str(root / "include" / "navkit"),
        "--filter",
        str(root / "src"),
        "--exclude",
        str(root / "tests"),
        "--exclude",
        str(build_dir),
        "--print-summary",
        "--txt",
        str(text_report),
        "--xml-pretty",
        "--output",
        str(xml_report),
    ]

    if args.html:
        gcovr_cmd.extend(
            [
                "--html-details",
                str(report_dir / "index.html"),
            ]
        )

    run(gcovr_cmd, cwd=root)
    print(f"Wrote coverage reports to: {report_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
