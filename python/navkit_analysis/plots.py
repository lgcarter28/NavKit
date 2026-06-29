from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

try:
    from navkit_analysis.style import (
        BOUND_COLOR,
        ERROR_COLOR,
        apply_nav_axes_style,
        apply_style,
        axis_position_error_label,
        ecef_position_error_label,
    )
except ModuleNotFoundError:
    # Allows direct execution as:
    #   python python/navkit_analysis/plots.py <run_dir>
    from style import (  # type: ignore[no-redef]
        BOUND_COLOR,
        ERROR_COLOR,
        apply_nav_axes_style,
        apply_style,
        axis_position_error_label,
        ecef_position_error_label,
    )


AXES = ("x", "y", "z")


def _required_columns(axis_name: str) -> tuple[str, str]:
    err_col = f"err_p_e_{axis_name}_m"
    sigma_col = f"sigma_p_e_{axis_name}_m"
    return err_col, sigma_col


def plot_position_error_covariance(run_dir: Path, show: bool = False) -> Path:
    """Plot ECEF position error with 1-sigma and 3-sigma covariance bounds."""
    apply_style()

    nav = pd.read_csv(run_dir / "nav.csv")
    time_s = nav["time_s"]

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )

    fig.suptitle(r"ECEF Position Error with $1\sigma$ and $3\sigma$ Bounds")

    for ax, axis_name in zip(axes, AXES):
        err_col, sigma_col = _required_columns(axis_name)
        err = nav[err_col]
        sigma = nav[sigma_col]

        ax.plot(
            time_s,
            err,
            color=ERROR_COLOR,
            label=ecef_position_error_label(axis_name),
        )

        ax.plot(
            time_s,
            sigma,
            color=BOUND_COLOR,
            linestyle="--",
            label=r"$1\sigma$",
        )
        ax.plot(time_s, -sigma, color=BOUND_COLOR, linestyle="--")

        ax.plot(
            time_s,
            3.0 * sigma,
            color=BOUND_COLOR,
            linestyle="-",
            label=r"$3\sigma$",
        )
        ax.plot(time_s, -3.0 * sigma, color=BOUND_COLOR, linestyle="-")

        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(axis_position_error_label(axis_name))
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    out = run_dir / "position_error_covariance.png"
    fig.savefig(out)
    print(f"Wrote {out}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return out


def plot_run(run_dir: Path, show: bool = False) -> None:
    plot_position_error_covariance(run_dir, show=show)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--show", action="store_true", help="Open an interactive matplotlib window.")
    args = parser.parse_args()

    plot_run(args.run_dir, show=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
