from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

# Directories to ignore
EXCLUDE_DIRS = {
    ".git",
    ".vs",
    ".vscode",
    ".venv",
    "build",
    "out",
    "data",
    "__pycache__",
}

# C/C++ source files to process
SOURCE_EXTENSIONS = {
    ".h",
    ".hpp",
    ".hh",
    ".ipp",
    ".cpp",
    ".cc",
    ".cxx",
    ".tpp",
}


def repo_root() -> Path:
    script = Path(__file__).resolve()
    if script.parent.name == "tools":
        return script.parent.parent
    return Path.cwd().resolve()


def find_cpp_files(root: Path) -> list[Path]:
    files: list[Path] = []

    for path in root.rglob("*"):
        if any(part in EXCLUDE_DIRS for part in path.parts):
            continue

        if path.suffix.lower() in SOURCE_EXTENSIONS:
            files.append(path)

    return sorted(files)


def run(cmd: list[str]) -> int:
    print("+", " ".join(str(c) for c in cmd))
    return subprocess.call(cmd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Format and lint NavKit.")

    parser.add_argument(
        "--check",
        action="store_true",
        help="Check formatting without modifying files.",
    )

    parser.add_argument(
        "--tidy",
        action="store_true",
        help="Run clang-tidy.",
    )

    parser.add_argument(
        "--fix",
        action="store_true",
        help="Apply clang-tidy fixes (requires --tidy).",
    )

    args = parser.parse_args()

    root = repo_root()
    files = find_cpp_files(root)

    if not files:
        print("No source files found.")
        return 0

    clang_format = shutil.which("clang-format")
    if clang_format is None:
        print("ERROR: clang-format not found on PATH.")
        return 1

    if args.check:
        ret = run(
            [
                clang_format,
                "--dry-run",
                "--Werror",
                *map(str, files),
            ]
        )
    else:
        ret = run(
            [
                clang_format,
                "-i",
                *map(str, files),
            ]
        )

    if ret != 0:
        return ret

    if args.tidy:
        clang_tidy = shutil.which("clang-tidy")

        if clang_tidy is None:
            print("ERROR: clang-tidy not found on PATH.")
            return 1

        build_dir = root / "build" / "Debug"

        for file in files:
            cmd = [
                clang_tidy,
                str(file),
                "-p",
                str(build_dir),
            ]

            if args.fix:
                cmd.append("--fix")

            ret = run(cmd)

            if ret != 0:
                return ret

    return 0


if __name__ == "__main__":
    sys.exit(main())
