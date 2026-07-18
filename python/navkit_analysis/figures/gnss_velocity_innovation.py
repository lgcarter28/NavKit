# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.style import BOUND_COLOR, RESIDUAL_COLOR, apply_nav_axes_style


def plot_gnss_velocity_innovation(run: RunData, save: bool = True) -> plt.Figure | None:
    updates = run.gnss_vel_update

    if updates is None:
        print("Skipping GNSS velocity innovation plot; missing gnss_vel_update.csv")
        return None

    time_s = updates["time_s"]
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(14.0, 9.0),
        constrained_layout=True,
    )
    fig.suptitle(r"GNSS Velocity Innovation with $1\sigma$ and $3\sigma$ Bounds")

    for ax, axis_name in zip(axes, AXES):
        nu = updates[f"nu_v_e_{axis_name}_mps"]
        sigma = updates[f"sigma_nu_v_e_{axis_name}_mps"]
        ax.plot(time_s, nu, color=RESIDUAL_COLOR, label=rf"$\nu_{{v_{axis_name}}}$")
        ax.plot(time_s, sigma, color=BOUND_COLOR, linestyle="--", label=r"$1\sigma$")
        ax.plot(time_s, -sigma, color=BOUND_COLOR, linestyle="--")
        ax.plot(time_s, 3.0 * sigma, color=BOUND_COLOR, linestyle="-", label=r"$3\sigma$")
        ax.plot(time_s, -3.0 * sigma, color=BOUND_COLOR, linestyle="-")
        ax.axhline(0.0, color="0.25", linewidth=0.8)
        ax.set_ylabel(f"{axis_name.upper()} Innovation [m/s]")
        ax.legend(loc="upper right")
        apply_nav_axes_style(ax)

    axes[-1].set_xlabel("Time [s]")

    if save:
        save_figure(fig, run.figures_dir / "gnss_velocity_innovation_ecef.png")

    return fig
