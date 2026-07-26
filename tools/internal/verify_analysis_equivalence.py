# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Verify serial and parallel Plotly aggregate rendering from one HDF5 bundle."""

from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import sys
import tempfile
from pathlib import Path


PLOTLY_DIV_ID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")


def normalized_digest(path: Path) -> str:
    """Hash Plotly HTML after removing its intentionally random div identifier."""
    document = path.read_text(encoding="utf-8")
    normalized = PLOTLY_DIV_ID.sub("<plotly-div-id>", document)
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def render(bundle: Path, output_dir: Path, plots: list[str], parallel_jobs: int) -> None:
    """Render one selected aggregate set through the public plotting tool."""
    tool = Path(__file__).resolve().parents[1] / "plot_monte_carlo.py"
    command = [
        sys.executable,
        str(tool),
        str(bundle),
        "--renderer",
        "plotly",
        "--parallel-jobs",
        str(parallel_jobs),
        "--output-dir",
        str(output_dir),
        "--force",
    ]
    for plot in plots:
        command.extend(("--plot", plot))
    subprocess.run(command, check=True)


def main() -> int:
    """Render the same aggregate products serially and in parallel, then compare data."""
    parser = argparse.ArgumentParser(
        description="Verify serial/parallel Plotly aggregate-rendering equivalence."
    )
    parser.add_argument("bundle", type=Path, help="Packaged Monte Carlo HDF5 analysis bundle.")
    parser.add_argument(
        "--plot",
        action="append",
        default=None,
        help="Aggregate plot family to compare; may be supplied multiple times.",
    )
    parser.add_argument(
        "--parallel-jobs",
        type=int,
        default=2,
        help="Parallel rendering worker count to compare against serial output.",
    )
    args = parser.parse_args()
    if args.parallel_jobs <= 1:
        parser.error("--parallel-jobs must be greater than one")
    if not args.bundle.is_file():
        parser.error(f"bundle does not exist: {args.bundle}")
    plots = args.plot or ["position_ned", "attitude_ned"]

    with tempfile.TemporaryDirectory(prefix="navkit_analysis_equivalence_") as temporary:
        root = Path(temporary)
        serial_dir = root / "serial"
        parallel_dir = root / "parallel"
        render(args.bundle, serial_dir, plots, 1)
        render(args.bundle, parallel_dir, plots, args.parallel_jobs)
        serial_paths = sorted(serial_dir.glob("*.html"))
        parallel_paths = sorted(parallel_dir.glob("*.html"))
        if [path.name for path in serial_paths] != [path.name for path in parallel_paths]:
            raise RuntimeError("serial and parallel renderer output names differ")
        for serial_path, parallel_path in zip(serial_paths, parallel_paths, strict=True):
            if normalized_digest(serial_path) != normalized_digest(parallel_path):
                raise RuntimeError(f"serial and parallel Plotly output differs: {serial_path.name}")
    print("Serial and parallel Plotly aggregate output is equivalent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
