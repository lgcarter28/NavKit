# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import save_figure
from navkit_analysis.style import apply_nav_axes_style


def plot_gnss_velocity_nis(run: RunData, save: bool = True) -> plt.Figure | None:
    updates = run.gnss_vel_update

    if updates is None:
        print("Skipping GNSS velocity NIS plot; missing gnss_vel_update.csv")
        return None

    fig, ax = plt.subplots(figsize=(14.0, 5.0), constrained_layout=True)
    fig.suptitle("GNSS Velocity NIS")
    ax.scatter(updates["time_s"], updates["nis"], s=18, color="tab:purple", label="NIS")
    ax.axhline(7.814727903251179, color="black", linestyle="--", label=r"$\chi^2_{3,0.95}$")
    ax.axhline(11.344866730144373, color="black", linestyle="-", label=r"$\chi^2_{3,0.99}$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NIS [-]")
    ax.legend(loc="upper right")
    apply_nav_axes_style(ax)

    if save:
        save_figure(fig, run.figures_dir / "gnss_velocity_nis_ecef.png")

    return fig
