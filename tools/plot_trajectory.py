# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Render frame-explicit trajectory truth and command/response dashboards."""

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
    parser = argparse.ArgumentParser(
        description="Generate available trajectory inspection plots for one NavKit run."
    )
    parser.add_argument("source", type=Path, help="Simulation run or data directory.")
    parser.add_argument("--renderer", choices=["matplotlib", "plotly"], default="plotly")
    parser.add_argument(
        "--plot",
        action="append",
        choices=(
            "kinematics_ecef",
            "kinematics_eci",
            "kinematics_ned",
            "kinematics_body",
            "position_lla_3d",
            "position_relative_ned_3d",
            "guidance",
            "autopilot_response",
            "guidance_control",
            "tracking_error",
        ),
        help="Render only the selected plot family; may be repeated.",
    )
    parser.add_argument("--start-time", type=float, default=None)
    parser.add_argument("--end-time", type=float, default=None)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    add_python_package_to_path()
    from navkit_analysis.figures.trajectory import trajectory_plot_specs
    from navkit_analysis.plot_spec import Plot3DSpec
    from navkit_analysis.renderers import (
        PLOTLY_INTERACTION_CONFIG,
        render_matplotlib,
        render_matplotlib_3d,
        render_plotly,
        render_plotly_3d,
    )
    from navkit_analysis.trajectory_data import load_trajectory_run

    run = load_trajectory_run(
        args.source,
        start_time_s=args.start_time,
        end_time_s=args.end_time,
    )
    specs = trajectory_plot_specs(run)
    if args.plot:
        selected = set(args.plot)
        specs = {name: spec for name, spec in specs.items() if name in selected}
    if not specs:
        print(f"No trajectory inspection logs found under {run.data_dir}")
        return 0

    for spec in specs.values():
        extension = ".html" if args.renderer == "plotly" else ".png"
        output_path = run.figures_dir / Path(spec.output_name).with_suffix(extension)
        if args.renderer == "plotly":
            figure = (
                render_plotly_3d(spec, output_path)
                if isinstance(spec, Plot3DSpec)
                else render_plotly(spec, output_path)
            )
            if args.show:
                figure.show(config=PLOTLY_INTERACTION_CONFIG)
        else:
            if isinstance(spec, Plot3DSpec):
                render_matplotlib_3d(spec, output_path)
            else:
                render_matplotlib(spec, output_path)

    if args.renderer == "matplotlib":
        import matplotlib.pyplot as plt

        if args.show:
            plt.show()
        else:
            plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
