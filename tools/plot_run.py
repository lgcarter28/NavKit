# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from internal.perf_artifacts import (
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

    return Path("output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal")


def run_name_from_dir(run_dir: Path) -> str:
    if run_dir.is_file():
        return run_dir.stem
    manifest = run_dir / "data" / "run_manifest.json"
    if not manifest.exists():
        manifest = run_dir / "run_manifest.json"
    if manifest.exists():
        data = json.loads(manifest.read_text(encoding="utf-8"))
        if isinstance(data.get("run_name"), str):
            return data["run_name"]

    return run_dir.name


def timing_path_from_source(source: Path) -> Path:
    """Choose a writable timing artifact path for a CSV directory or HDF5 source."""
    if source.is_file():
        return source.parent / f"{source.stem}.timing.json"
    return source / "data" / "timing.json"


def main(argv: list[str] | None = None) -> int:
    forwarded_argv = list(sys.argv[1:] if argv is None else argv)
    show_timing_report = "--no-timing-report" not in forwarded_argv
    forwarded_argv = [item for item in forwarded_argv if item != "--no-timing-report"]
    root = repo_root()
    python_root = root / "python"
    if "--show" not in forwarded_argv:
        os.environ.setdefault("MPLBACKEND", "Agg")

    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))

    from navkit_analysis.plots import main as plots_main

    run_dir = run_dir_from_argv(forwarded_argv)
    command = [str(Path(__file__).name), *forwarded_argv]

    return_code, result = measure_call(plots_main, forwarded_argv)
    timing_path = timing_path_from_source(run_dir)
    timing_path.parent.mkdir(parents=True, exist_ok=True)
    command_name = "plot_run"
    record = update_timing_artifact(
        timing_path,
        run_name=run_name_from_dir(run_dir),
        command_name=command_name,
        command=command,
        result=result,
        tool_version="plot_run.py",
    )
    if show_timing_report:
        print()
        print_command_timing(command_name, record)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
