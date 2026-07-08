# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import platform
import subprocess
from pathlib import Path


def find_conan_build_env(build_dir: Path, build_type: str) -> Path | None:
    candidates = [
        build_dir / "build" / build_type / "generators" / "conanbuild.bat",
        build_dir / "build" / "generators" / "conanbuild.bat",
        build_dir / "generators" / "conanbuild.bat",
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return None


def run_with_optional_conan_env(
    cmd: list[str],
    cwd: Path,
    *,
    build_dir: Path,
    build_type: str,
) -> int:
    if platform.system() != "Windows":
        print("+", " ".join(str(c) for c in cmd), flush=True)
        return subprocess.call(cmd, cwd=cwd)

    conan_build_env = find_conan_build_env(build_dir, build_type)
    if conan_build_env is None:
        print("+", " ".join(str(c) for c in cmd), flush=True)
        return subprocess.call(cmd, cwd=cwd)

    batch_path = build_dir / ".navkit_run_with_conan_env.bat"
    batch_path.write_text(
        "\n".join(
            [
                "@echo off",
                f'call "{conan_build_env}"',
                "if errorlevel 1 exit /b %errorlevel%",
                subprocess.list2cmdline(cmd),
                "exit /b %errorlevel%",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print("+", str(batch_path), flush=True)
    return subprocess.call([str(batch_path)], cwd=cwd)
