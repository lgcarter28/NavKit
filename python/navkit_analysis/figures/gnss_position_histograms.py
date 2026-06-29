from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.common import AXES, save_figure
from navkit_analysis.figures.gnss_position_innovation import _innovation_columns
from navkit_analysis.style import BOUND_COLOR, RESIDUAL_COLOR, apply_nav_axes_style


def plot_gnss_position_histograms(
    run: RunData,
    save: bool = True,
) -> plt.Figure | None:
    """Plot GNSS position residual/innovation histograms by ECEF axis."""
    updates = run.gnss_pos_update

    if updates is None:
        print("Skipping GNSS residual histogram; missing gnss_pos_update.csv")
        return None

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

    if save:
        save_figure(fig, run.run_dir / "gnss_position_innovation_histograms.png")

    return fig
