from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

try:
    from navkit_analysis.style import (
        BOUND_COLOR,
        ERROR_COLOR,
        RESIDUAL_COLOR,
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
        RESIDUAL_COLOR,
        apply_nav_axes_style,
        apply_style,
        axis_position_error_label,
        ecef_position_error_label,
    )


AXES = ("x", "y", "z")
NIS_95_DOF3 = 7.814727903251179
NIS_99_DOF3 = 11.344866730144373


def _position_columns(axis_name: str) -> tuple[str, str]:
    err_col = f"err_p_e_{axis_name}_m"
    sigma_col = f"sigma_p_e_{axis_name}_m"
    return err_col, sigma_col


def _innovation_columns(axis_name: str) -> tuple[str, str]:
    nu_col = f"nu_p_e_{axis_name}_m"
    sigma_col = f"sigma_nu_p_e_{axis_name}_m"
    return nu_col, sigma_col


def _save_or_show(fig: plt.Figure, out: Path, show: bool) -> Path:
    fig.savefig(out)
    print(f"Wrote {out}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return out


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
        err_col, sigma_col = _position_columns(axis_name)
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

    return _save_or_show(fig, run_dir / "position_error_covariance.png", show)


def plot_gnss_position_innovation(run_dir: Path, show: bool = False) -> Path | None:
    """Plot GNSS position innovations with 1-sigma and 3-sigma innovation bounds."""
    update_path = run_dir / "gnss_pos_update.csv"
    if not update_path.exists():
        print(f"Skipping GNSS innovation plot; missing {update_path}")
        return None

    apply_style()

    updates = pd.read_csv(update_path)
    time_s = updates["time_s"]

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )

    fig.suptitle(r"GNSS Position Innovation with $1\sigma$ and $3\sigma$ Bounds")

    for ax, axis_name in zip(axes, AXES):
        nu_col, sigma_col = _innovation_columns(axis_name)
        nu = updates[nu_col]
        sigma = updates[sigma_col]

        ax.plot(
            time_s,
            nu,
            color=RESIDUAL_COLOR,
            label=rf"$\nu_{{p_{axis_name}}}$",
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
        ax.set_ylabel(f"{axis_name.upper()} Innovation [m]")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    return _save_or_show(fig, run_dir / "gnss_position_innovation.png", show)


def plot_gnss_position_nis(run_dir: Path, show: bool = False) -> Path | None:
    """Plot GNSS position NIS with common chi-square thresholds for 3 DOF."""
    update_path = run_dir / "gnss_pos_update.csv"
    if not update_path.exists():
        print(f"Skipping GNSS NIS plot; missing {update_path}")
        return None

    apply_style()

    updates = pd.read_csv(update_path)
    time_s = updates["time_s"]

    fig, ax = plt.subplots(figsize=(14.0, 5.0), constrained_layout=True)
    fig.suptitle("GNSS Position Normalized Innovation Squared")

    ax.plot(time_s, updates["nis"], color=RESIDUAL_COLOR, label="NIS")
    ax.axhline(NIS_95_DOF3, color=BOUND_COLOR, linestyle="--", label=r"$\chi^2_{3,0.95}$")
    ax.axhline(NIS_99_DOF3, color=BOUND_COLOR, linestyle="-", label=r"$\chi^2_{3,0.99}$")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NIS [-]")
    ax.legend(loc="upper right")
    apply_nav_axes_style(ax)

    return _save_or_show(fig, run_dir / "gnss_position_nis.png", show)


def plot_gnss_position_residual_histograms(run_dir: Path, show: bool = False) -> Path | None:
    """Plot GNSS position residual/innovation histograms by ECEF axis."""
    update_path = run_dir / "gnss_pos_update.csv"
    if not update_path.exists():
        print(f"Skipping GNSS residual histogram; missing {update_path}")
        return None

    apply_style()

    updates = pd.read_csv(update_path)

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=False,
        figsize=(10.0, 9.0),
        constrained_layout=True,
    )

    fig.suptitle("GNSS Position Innovation Histograms")

    for ax, axis_name in zip(axes, AXES):
        nu_col, _ = _innovation_columns(axis_name)
        ax.hist(updates[nu_col], bins=30, color=RESIDUAL_COLOR, alpha=0.75)
        ax.axvline(0.0, color=BOUND_COLOR, linewidth=1.0)
        ax.set_ylabel(f"{axis_name.upper()} Count")
        ax.set_xlabel(f"{axis_name.upper()} Innovation [m]")
        apply_nav_axes_style(ax)

    return _save_or_show(fig, run_dir / "gnss_position_innovation_histograms.png", show)


def plot_run(run_dir: Path, show: bool = False) -> None:
    plot_position_error_covariance(run_dir, show=show)
    plot_gnss_position_innovation(run_dir, show=show)
    plot_gnss_position_nis(run_dir, show=show)
    plot_gnss_position_residual_histograms(run_dir, show=show)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--show", action="store_true", help="Open interactive matplotlib windows.")
    args = parser.parse_args()

    plot_run(args.run_dir, show=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
