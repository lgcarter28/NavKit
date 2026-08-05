# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Quick CSV/HDF5 field plotting through NavKit's shared plot-spec renderers."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def add_python_package_to_path() -> None:
    root = Path(__file__).resolve().parents[1]
    python_root = root / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))


def main() -> int:
    parser = argparse.ArgumentParser(description="Quickly plot two named NavKit analysis fields.")
    parser.add_argument("source", type=Path, help="CSV run directory or HDF5 analysis bundle.")
    parser.add_argument("--table", default="nav", help="Run table name, for example nav or truth.")
    parser.add_argument("--x", default="time_s", help="X-axis field name.")
    parser.add_argument("--y", required=True, help="Y-axis field name.")
    parser.add_argument("--run-id", default=None, help="Run ID for a multi-run HDF5 campaign bundle.")
    parser.add_argument("--title", default=None, help="Optional plot title.")
    parser.add_argument("--renderer", choices=["matplotlib", "plotly"], default="plotly")
    parser.add_argument("--output", type=Path, default=None, help="Output .png or .html file.")
    parser.add_argument("--show", action="store_true", help="Open the rendered figure interactively.")
    args = parser.parse_args()

    add_python_package_to_path()
    from navkit_analysis.plot_spec import quick_xy_plot_spec
    from navkit_analysis.renderers import (
        PLOTLY_INTERACTION_CONFIG,
        render_matplotlib,
        render_plotly,
    )
    from navkit_analysis.sources import load_analysis_run

    run = load_analysis_run(args.source, run_id=args.run_id)
    frame = getattr(run, args.table, None)
    if frame is None:
        raise ValueError(f"source does not contain table '{args.table}'")
    if args.x not in frame or args.y not in frame:
        raise ValueError(f"table '{args.table}' must contain both '{args.x}' and '{args.y}'")
    extension = ".html" if args.renderer == "plotly" else ".png"
    output = args.output or Path(f"{args.table}_{args.y}{extension}")
    spec = quick_xy_plot_spec(
        frame[args.x].to_numpy(),
        frame[args.y].to_numpy(),
        title=args.title or f"{args.table}: {args.y} versus {args.x}",
        x_label=args.x,
        y_label=args.y,
        series_label=args.y,
        output_name=output.name,
    )
    if args.renderer == "plotly":
        figure = render_plotly(spec, output)
        if args.show:
            figure.show(config=PLOTLY_INTERACTION_CONFIG)
    else:
        import matplotlib.pyplot as plt

        render_matplotlib(spec, output)
        if args.show:
            plt.show()
        else:
            plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
