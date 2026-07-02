# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import json
import sys
from pathlib import Path

from perf_artifacts import (
    measure_call,
    print_command_timing,
    update_timing_artifact,
)


def repo_root() -> Path:
    script_path = Path(__file__).resolve()

    if script_path.parent.name == "tools":
        return script_path.parent.parent

    return Path.cwd().resolve()


def run_dir_from_argv(argv: list[str]) -> Path:
    for item in argv:
        if not item.startswith("-"):
            return Path(item)

    return Path("data/logs/stationary_gnss_demo")


def run_name_from_dir(run_dir: Path) -> str:
    manifest = run_dir / "run_manifest.json"
    if manifest.exists():
        data = json.loads(manifest.read_text(encoding="utf-8"))
        if isinstance(data.get("run_name"), str):
            return data["run_name"]

    return run_dir.name


def main(argv: list[str] | None = None) -> int:
    forwarded_argv = list(sys.argv[1:] if argv is None else argv)
    show_timing_report = "--no-timing-report" not in forwarded_argv
    forwarded_argv = [item for item in forwarded_argv if item != "--no-timing-report"]
    root = repo_root()
    python_root = root / "python"

    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))

    from navkit_analysis.plots import main as plots_main

    run_dir = run_dir_from_argv(forwarded_argv)
    command = [str(Path(__file__).name), *forwarded_argv]

    return_code, result = measure_call(plots_main, forwarded_argv)
    timing_path = run_dir / "timing.json"
    command_name = "analysis"
    record = update_timing_artifact(
        timing_path,
        run_name=run_name_from_dir(run_dir),
        command_name=command_name,
        command=command,
        result=result,
        tool_version="run_analysis.py",
    )
    if show_timing_report:
        print()
        print_command_timing(command_name, record)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
