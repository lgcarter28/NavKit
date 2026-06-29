# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


def repo_root() -> Path:
    """Return the repository root when this script is run from tools/build.py."""
    script_path = Path(__file__).resolve()
    if script_path.parent.name == "tools":
        return script_path.parent.parent
    return Path.cwd().resolve()


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(str(c) for c in cmd))
    subprocess.check_call(cmd, cwd=cwd)


def find_executable_near_python(name: str) -> str:
    scripts_dir = Path(__file__).resolve().parents[1] / ".venv" / "Scripts"
    exe = scripts_dir / f"{name}.exe"
    if exe.exists():
        return str(exe)

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


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure and build NavKit.")
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Release")
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--skip-conan", action="store_true")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--jobs", "-j", type=int, default=None)
    args = parser.parse_args()

    root = repo_root()
    build_dir = root / "build" / args.build_type

    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)

    build_dir.mkdir(parents=True, exist_ok=True)

    if args.build_only and args.clean:
        raise ValueError("--clean cannot be used with --build-only")

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
