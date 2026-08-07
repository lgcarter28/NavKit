# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from navkit_analysis.figures.common import (
    plot_runtime_innovation_gate,
    plot_runtime_p_value_gate,
    save_figure,
)
from navkit_analysis.statistics import (
    chi_square_threshold,
    chi_square_upper_tail_probability,
    measurement_dof_from_innovations,
)
from navkit_analysis.style import BOUND_COLOR, RESIDUAL_COLOR, apply_nav_axes_style


def plot_gnss_nis_consistency(
    updates: pd.DataFrame,
    title: str,
    output_path: Path,
    save: bool,
) -> plt.Figure:
    """Plot GNSS NIS and its equivalent upper-tail chi-square p-value."""

    time_s = updates["time_s"].to_numpy(dtype=float)
    nis = updates["nis"].to_numpy(dtype=float)
    dof = measurement_dof_from_innovations(updates)
    p_value = chi_square_upper_tail_probability(nis, dof)
    finite_p_values = p_value[np.isfinite(p_value)]
    mean_p_value = finite_p_values.mean() if finite_p_values.size > 0 else None

    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=(14.0, 8.0),
        constrained_layout=True,
    )
    fig.suptitle(title)

    ax_nis = axes[0]
    ax_nis.scatter(time_s, nis, color=RESIDUAL_COLOR, label="NIS", s=28)
    ax_nis.axhline(
        chi_square_threshold(0.95, dof),
        color=BOUND_COLOR,
        linestyle="--",
        label=rf"$\chi^2_{{{dof},0.95}}$",
    )
    ax_nis.axhline(
        chi_square_threshold(0.99, dof),
        color=BOUND_COLOR,
        linestyle="-",
        label=rf"$\chi^2_{{{dof},0.99}}$",
    )
    plot_runtime_innovation_gate(ax_nis, updates)
    ax_nis.set_ylabel("NIS [-]")
    ax_nis.legend(loc="upper right")
    apply_nav_axes_style(ax_nis)

    ax_p = axes[1]
    ax_p.scatter(time_s, p_value, color=RESIDUAL_COLOR, label="p-value", s=28)
    if mean_p_value is not None:
        ax_p.axhline(
            mean_p_value,
            color=BOUND_COLOR,
            linestyle="--",
            label=rf"$\bar{{p}}={mean_p_value:.3f}$",
        )
    ax_p.axhline(0.5, color=BOUND_COLOR, linestyle=":", label=r"$E[p]=0.5$")
    ax_p.axhline(0.05, color=BOUND_COLOR, linestyle="-.", label=r"$p=0.05$")
    ax_p.axhline(
        0.01,
        color=BOUND_COLOR,
        linestyle=(0, (3, 1, 1, 1)),
        label=r"$p=0.01$",
    )
    plot_runtime_p_value_gate(ax_p, updates, p_value)
    ax_p.set_xlabel("Time [s]")
    ax_p.set_ylabel("upper-tail p-value [-]")
    ax_p.set_ylim(-0.02, 1.02)
    ax_p.legend(loc="upper right")
    apply_nav_axes_style(ax_p)

    if save:
        save_figure(fig, output_path)

    return fig
