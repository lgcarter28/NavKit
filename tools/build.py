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
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd)


def find_conan_toolchain(build_dir: Path) -> Path:
    """Locate Conan's generated CMake toolchain file.

    Conan 2 commonly writes it to:
        <output-folder>/build/generators/conan_toolchain.cmake

    Some generator configurations may write it to:
        <output-folder>/generators/conan_toolchain.cmake
    """
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
    parser.add_argument(
        "--build-type",
        choices=["Release", "Debug"],
        default="Release",
        help="CMake/Conan build type.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove the selected build directory before configuring.",
    )
    parser.add_argument(
        "--jobs",
        "-j",
        type=int,
        default=None,
        help="Parallel build jobs passed to cmake --build.",
    )
    args = parser.parse_args()

    root = repo_root()
    build_dir = root / "build" / args.build_type

    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)

    build_dir.mkdir(parents=True, exist_ok=True)

    # Do not use `cmake --preset conan-release/debug` here. Conan also generates
    # presets, and if the repository has presets with the same names CMake fails
    # with "Duplicate presets". Driving CMake directly through the generated
    # toolchain file avoids that conflict and works consistently with Conan 2.
    run(
        [
            "conan",
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
