# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import save_figure
from navkit_analysis.statistics import chi_square_threshold, measurement_dof_from_innovations
from navkit_analysis.style import BOUND_COLOR, RESIDUAL_COLOR, apply_nav_axes_style


def plot_gnss_velocity_nis(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot GNSS velocity NIS against chi-square consistency thresholds."""

    updates = run.gnss_vel_update

    if updates is None:
        print("Skipping GNSS velocity NIS plot; missing gnss_vel_update.csv")
        return None

    dof = measurement_dof_from_innovations(updates)
    nis_95 = chi_square_threshold(0.95, dof)
    nis_99 = chi_square_threshold(0.99, dof)

    fig, ax = plt.subplots(figsize=(14.0, 5.0), constrained_layout=True)
    fig.suptitle("GNSS Velocity NIS")
    ax.scatter(updates["time_s"], updates["nis"], s=18, color=RESIDUAL_COLOR, label="NIS")
    ax.axhline(
        nis_95,
        color=BOUND_COLOR,
        linestyle="--",
        label=rf"$\chi^2_{{{dof},0.95}}$",
    )
    ax.axhline(
        nis_99,
        color=BOUND_COLOR,
        linestyle="-",
        label=rf"$\chi^2_{{{dof},0.99}}$",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NIS [-]")
    ax.legend(loc="upper right")
    apply_nav_axes_style(ax)

    if save:
        save_figure(fig, run.figures_dir / "gnss_velocity_nis_ecef.png")

    return fig
