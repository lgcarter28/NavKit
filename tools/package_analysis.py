# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Package existing NavKit CSV run/campaign artifacts into HDF5 analysis bundles."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def add_python_package_to_path() -> None:
    root = Path(__file__).resolve().parents[1]
    python_root = root / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))


def default_output_path(source: Path) -> Path:
    if (source / "campaign_manifest.json").exists():
        return source / "analysis_bundle.h5"
    return source / "data" / "analysis_bundle.h5"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Package a NavKit CSV run or Monte Carlo campaign into an HDF5 analysis bundle."
    )
    parser.add_argument("source", type=Path, help="Run directory or campaign directory to package.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output .h5 path. Defaults inside the selected run/campaign directory.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=1000,
        help="Maximum cached points for Monte Carlo aggregate plot products.",
    )
    parser.add_argument(
        "--bundle-mode",
        choices=["full", "derived_only"],
        default="full",
        help="Store all selected raw tables or only reusable derived/aggregate campaign products.",
    )
    parser.add_argument(
        "--compression",
        choices=["lzf", "gzip", "none"],
        default="lzf",
        help="Numeric HDF5 dataset compression; lzf favors repeated interactive analysis.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Rebuild even when the input-manifest and packaging settings match the current bundle.",
    )
    args = parser.parse_args()
    if args.max_plot_points <= 1:
        raise ValueError("--max-plot-points must be greater than one")

    add_python_package_to_path()
    from navkit_analysis.bundle import package_analysis

    source = args.source.resolve()
    output = (args.output or default_output_path(source)).resolve()
    started_s = time.perf_counter()
    package_analysis(
        source,
        output,
        max_plot_points=args.max_plot_points,
        bundle_mode=args.bundle_mode,
        compression=args.compression,
        force=args.force,
    )
    print(f"Analysis bundle packaging: {time.perf_counter() - started_s:.3f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
