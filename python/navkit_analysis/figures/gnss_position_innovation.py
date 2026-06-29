from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.style import (
    BOUND_COLOR,
    RESIDUAL_COLOR,
    apply_nav_axes_style,
    axis_innovation_label,
    gnss_position_innovation_label,
)


def _innovation_columns(axis_name: str) -> tuple[str, str]:
    nu_col = f"nu_p_e_{axis_name}_m"
    sigma_col = f"sigma_nu_p_e_{axis_name}_m"
    return nu_col, sigma_col


def plot_gnss_position_innovation(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot GNSS position innovations with 1-sigma and 3-sigma innovation bounds."""
    updates = run.gnss_pos_update

    if updates is None:
        print("Skipping GNSS innovation plot; missing gnss_pos_update.csv")
        return None

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
            label=gnss_position_innovation_label(axis_name),
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
        ax.set_ylabel(axis_innovation_label(axis_name))
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.run_dir / "gnss_position_innovation.png")

    return fig
