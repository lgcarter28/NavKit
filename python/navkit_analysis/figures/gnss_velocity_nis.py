# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import matplotlib.pyplot as plt

from navkit_analysis.data import RunData
from navkit_analysis.figures.gnss_nis import plot_gnss_nis_consistency


def plot_gnss_velocity_nis(run: RunData, save: bool = True) -> plt.Figure | None:
    """Plot GNSS velocity NIS and upper-tail chi-square p-values."""

    if run.gnss_vel_update is None:
        print("Skipping GNSS velocity NIS plot; missing gnss_vel_update.csv")
        return None

    return plot_gnss_nis_consistency(
        run.gnss_vel_update,
        "GNSS Velocity NIS Consistency",
        run.figures_dir / "gnss_velocity_nis_ecef.png",
        save,
    )
