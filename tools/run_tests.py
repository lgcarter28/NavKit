from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def run(cmd: list[str], cwd: Path) -> int:
    print("+", " ".join(cmd))
    return subprocess.call(cmd, cwd=cwd)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    build_dir = root / "build" / args.build_type

    if not build_dir.exists():
        print(f"Build directory does not exist: {build_dir}")
        print("Run: python tools/build.py --build-type", args.build_type)
        return 1

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

    return run(cmd, cwd=root)


if __name__ == "__main__":
    raise SystemExit(main())
