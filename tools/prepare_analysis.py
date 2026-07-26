# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Prepare a reusable HDF5 analysis bundle and selected consistency-cache families."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def add_python_package_to_path() -> None:
    """Expose the repository analysis package to this standalone tool."""
    python_root = Path(__file__).resolve().parents[1] / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))


def default_output_path(source: Path) -> Path:
    """Return the conventional bundle path for one CSV source directory."""
    if (source / "campaign_manifest.json").exists():
        return source / "analysis_bundle.h5"
    return source / "data" / "analysis_bundle.h5"


def main() -> int:
    """Package raw artifacts when needed, then build selected reusable cache families."""
    parser = argparse.ArgumentParser(
        description=(
            "Prepare a NavKit CSV run/campaign or existing HDF5 bundle for repeated analysis "
            "without rendering figures."
        )
    )
    parser.add_argument(
        "source",
        type=Path,
        help="CSV run/campaign directory or an existing .h5 analysis bundle.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Bundle output for a CSV source; defaults inside the selected source directory.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=1000,
        help="Maximum cached selectable time points when creating or refreshing a bundle.",
    )
    parser.add_argument(
        "--bundle-mode",
        choices=["full", "derived_only"],
        default="full",
        help="Raw-table retention mode used only when packaging a CSV source.",
    )
    parser.add_argument(
        "--compression",
        choices=["lzf", "gzip", "none"],
        default="lzf",
        help="Numeric HDF5 compression used only when packaging a CSV source.",
    )
    parser.add_argument(
        "--series-kind",
        action="append",
        choices=["nees", "nis", "marginal"],
        default=None,
        help="Consistency cache family to prepare; repeat to select several. Defaults to all.",
    )
    parser.add_argument(
        "--force-package",
        action="store_true",
        help="Rebuild the HDF5 bundle from CSV even when its source fingerprint matches.",
    )
    args = parser.parse_args()
    if args.max_plot_points <= 1:
        parser.error("--max-plot-points must be greater than one")
    if args.source.suffix.lower() in {".h5", ".hdf5"} and args.output is not None:
        parser.error("--output is valid only when packaging a CSV source directory")

    add_python_package_to_path()
    from navkit_analysis.bundle import package_analysis
    from navkit_analysis.consistency import (
        consistency_cache_performance,
        refresh_consistency_cache,
    )

    started_s = time.perf_counter()
    source = args.source.resolve()
    if source.suffix.lower() in {".h5", ".hdf5"}:
        if not source.is_file():
            parser.error(f"bundle does not exist: {source}")
        bundle_path = source
        package_elapsed_s = 0.0
    else:
        if not source.is_dir():
            parser.error(f"CSV source directory does not exist: {source}")
        package_started_s = time.perf_counter()
        bundle_path = package_analysis(
            source,
            (args.output or default_output_path(source)).resolve(),
            max_plot_points=args.max_plot_points,
            bundle_mode=args.bundle_mode,
            compression=args.compression,
            force=args.force_package,
        )
        package_elapsed_s = time.perf_counter() - package_started_s

    cache_started_s = time.perf_counter()
    selected_kinds = tuple(args.series_kind or ("nees", "nis", "marginal"))
    nees_series, nis_series, marginal_series = refresh_consistency_cache(
        bundle_path,
        args.max_plot_points,
        selected_kinds=selected_kinds,
    )
    cache_elapsed_s = time.perf_counter() - cache_started_s
    print("Analysis preparation timing:")
    print(f"  package: {package_elapsed_s:.3f} s")
    print(f"  cache:   {cache_elapsed_s:.3f} s")
    print(f"  total:   {time.perf_counter() - started_s:.3f} s")
    print(
        f"  series:  {len(nees_series)} NEES, {len(nis_series)} NIS, "
        f"{len(marginal_series)} marginal"
    )
    performance = consistency_cache_performance(bundle_path)
    stages = performance.get("stages", {}) if isinstance(performance, dict) else {}
    if isinstance(stages, dict):
        print("  cache stages:")
        for name, value in stages.items():
            elapsed_s = value.get("elapsed_s") if isinstance(value, dict) else None
            if isinstance(elapsed_s, int | float):
                print(f"    {name}: {elapsed_s:.3f} s")
    print(f"  bundle:  {bundle_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
