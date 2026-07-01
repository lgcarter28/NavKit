# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import platform
import subprocess
from pathlib import Path


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
    args = parser.parse_args()
    exe = default_exe(args.build_type)
    print(f"Running {exe} {args.config}")
    return subprocess.call([str(exe), str(args.config)])


if __name__ == "__main__":
    raise SystemExit(main())
