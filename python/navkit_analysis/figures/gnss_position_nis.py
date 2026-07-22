# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import save_figure
from navkit_analysis.statistics import (
    chi_square_threshold,
    chi_square_upper_tail_probability,
    measurement_dof_from_innovations,
)
from navkit_analysis.style import BOUND_COLOR, RESIDUAL_COLOR, apply_nav_axes_style


def plot_gnss_position_nis(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot GNSS position NIS and chi-square upper-tail p-values."""
    updates = run.gnss_pos_update

    if updates is None:
        print("Skipping GNSS NIS plot; missing gnss_pos_update.csv")
        return None

    time_s = updates["time_s"]
    nis = updates["nis"]

    dof = measurement_dof_from_innovations(updates)
    nis_95 = chi_square_threshold(0.95, dof)
    nis_99 = chi_square_threshold(0.99, dof)

    # Upper-tail p-value:
    #   p = P(ChiSq_dof >= observed NIS)
    # For a statistically consistent filter, p-values should be Uniform(0, 1),
    # with expected value 0.5.
    p_value = chi_square_upper_tail_probability(nis, dof)
    mean_p_value = p_value.mean()

    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=(14.0, 8.0),
        constrained_layout=True,
    )

    fig.suptitle("GNSS Position NIS Consistency")

    ax_nis = axes[0]
    ax_nis.scatter(time_s, nis, color=RESIDUAL_COLOR, label="NIS", s=28)
    ax_nis.axhline(
        nis_95,
        color=BOUND_COLOR,
        linestyle="--",
        label=rf"$\chi^2_{{{dof},0.95}}$",
    )
    ax_nis.axhline(
        nis_99,
        color=BOUND_COLOR,
        linestyle="-",
        label=rf"$\chi^2_{{{dof},0.99}}$",
    )
    ax_nis.set_ylabel("NIS [-]")
    ax_nis.legend(loc="upper right")
    apply_nav_axes_style(ax_nis)

    ax_p = axes[1]
    ax_p.scatter(time_s, p_value, color=RESIDUAL_COLOR, label="p-value", s=28)
    ax_p.axhline(
        mean_p_value,
        color=BOUND_COLOR,
        linestyle="--",
        label=rf"$\bar{{p}}={mean_p_value:.3f}$",
    )
    ax_p.axhline(
        0.5,
        color=BOUND_COLOR,
        linestyle=":",
        label=r"$E[p]=0.5$",
    )
    ax_p.axhline(0.05, color=BOUND_COLOR, linestyle="-.", label=r"$p=0.05$")
    ax_p.axhline(0.01, color=BOUND_COLOR, linestyle=(0, (3, 1, 1, 1)), label=r"$p=0.01$")
    ax_p.set_xlabel("Time [s]")
    ax_p.set_ylabel("p-value [-]")
    ax_p.set_ylim(-0.02, 1.02)
    ax_p.legend(loc="upper right")
    apply_nav_axes_style(ax_p)

    if save:
        save_figure(fig, run.figures_dir / "gnss_position_nis_ecef.png")

    return fig
