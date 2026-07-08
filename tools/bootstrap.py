# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def run(command: list[str], cwd: Path) -> None:
    print("+", " ".join(command))
    subprocess.check_call(command, cwd=cwd)


def venv_executable(root: Path, name: str) -> Path:
    if os.name == "nt":
        return root / ".venv" / "Scripts" / f"{name}.exe"
    return root / ".venv" / "bin" / name


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    venv_dir = root / ".venv"

    if not venv_dir.exists():
        run([sys.executable, "-m", "venv", str(venv_dir)], root)

    python = venv_executable(root, "python")
    conan = venv_executable(root, "conan")

    run([str(python), "-m", "pip", "install", "--upgrade", "pip"], root)
    run([str(python), "-m", "pip", "install", "conan", "ninja", "-e", str(root / "python")], root)

    profile = subprocess.run(
        [str(conan), "profile", "path", "default"],
        cwd=root,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if profile.returncode != 0:
        run([str(conan), "profile", "detect"], root)

    print(f"NavKit environment is ready: {venv_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
