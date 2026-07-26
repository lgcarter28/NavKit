# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Generate interactive Monte Carlo NEES/NIS consistency dashboards from HDF5."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def add_python_package_to_path() -> None:
    """Expose the repository analysis package to this standalone tool."""
    python_root = Path(__file__).resolve().parents[1] / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))


def main() -> int:
    """Parse arguments and generate/refresh bundle-backed consistency artifacts."""
    parser = argparse.ArgumentParser(
        description="Generate interactive Monte Carlo NEES/NIS consistency dashboards from HDF5."
    )
    parser.add_argument("bundle", type=Path, help="Packaged Monte Carlo HDF5 analysis bundle.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Override the output directory. Defaults to <bundle-parent>/summary.",
    )
    parser.add_argument(
        "--refresh-cache",
        action="store_true",
        help="Recompute the time-indexed NEES/NIS cache from packaged per-run data.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=None,
        help="Limit the cached selectable time grid when refreshing the cache.",
    )
    parser.add_argument(
        "--heatmap-mode",
        action="append",
        default=None,
        help=(
            "Regenerate only one heatmap mode; may be supplied multiple times. "
            "The dashboard index is preserved for a selective refresh."
        ),
    )
    parser.add_argument(
        "--parallel-jobs",
        type=int,
        default=1,
        help="Render independent Plotly dashboards with this many threads after loading the bundle.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Regenerate selected dashboard/report artifacts even when their cache matches.",
    )
    args = parser.parse_args()
    if args.max_plot_points is not None and args.max_plot_points <= 1:
        parser.error("--max-plot-points must be greater than one")
    if args.parallel_jobs <= 0:
        parser.error("--parallel-jobs must be positive")
    if not args.bundle.is_file():
        parser.error(f"bundle does not exist: {args.bundle}")

    add_python_package_to_path()
    from navkit_analysis.consistency import generate_consistency_outputs
    from navkit_analysis.consistency_plots import HEATMAP_MODES

    if args.heatmap_mode is not None:
        unsupported_modes = set(args.heatmap_mode).difference(HEATMAP_MODES)
        if unsupported_modes:
            parser.error(
                f"unsupported --heatmap-mode values: {sorted(unsupported_modes)}"
            )

    output_dir = args.output_dir or args.bundle.parent / "summary"
    summary = generate_consistency_outputs(
        args.bundle,
        output_dir,
        refresh_cache=args.refresh_cache,
        max_plot_points=args.max_plot_points,
        heatmap_modes=args.heatmap_mode,
        parallel_jobs=args.parallel_jobs,
        force=args.force,
    )
    print("Consistency analysis timing:")
    print(f"  cache:   {float(summary['cache_elapsed_s']):.3f} s")
    print(f"  plots:   {float(summary['plot_elapsed_s']):.3f} s")
    print(f"  reports: {float(summary['report_elapsed_s']):.3f} s")
    print(f"  total:   {float(summary['total_elapsed_s']):.3f} s")
    print(
        f"  groups:  {summary['nees_group_count']} NEES, "
        f"{summary['nis_group_count']} NIS, "
        f"{summary['marginal_group_count']} marginal"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
