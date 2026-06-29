from __future__ import annotations

import matplotlib.pyplot as plt


# Canonical NavKit plotting colors. Keep this small and consistent.
ERROR_COLOR = "red"
TRUTH_COLOR = "black"
ESTIMATE_COLOR = "tab:blue"
MEASUREMENT_COLOR = "0.55"
RESIDUAL_COLOR = "tab:purple"
BOUND_COLOR = "black"


def apply_style() -> None:
    """Apply the standard NavKit matplotlib style.

    The goal is publication/report-quality plots without requiring a full LaTeX
    installation. Matplotlib's built-in mathtext renderer is used for symbols.
    """
    plt.rcParams.update(
        {
            "font.family": "serif",
            "mathtext.fontset": "cm",
            "font.size": 12,
            "axes.titlesize": 16,
            "axes.labelsize": 13,
            "legend.fontsize": 12,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "figure.titlesize": 16,
            "lines.linewidth": 1.8,
            "grid.alpha": 0.35,
            "savefig.dpi": 150,
            "savefig.bbox": "tight",
        }
    )


def apply_nav_axes_style(ax: plt.Axes) -> None:
    """Apply standard grid/axis styling to one axes object."""
    ax.grid(True, which="major", alpha=0.35)
    ax.minorticks_on()
    ax.grid(True, which="minor", alpha=0.15)


def axis_position_error_label(axis_name: str) -> str:
    """Return a capitalized position-error axis label, e.g. 'X Error [m]'."""
    return f"{axis_name.upper()} Error [m]"


def axis_innovation_label(axis_name: str) -> str:
    """Return a capitalized innovation axis label, e.g. 'X Innovation [m]'."""
    return f"{axis_name.upper()} Innovation [m]"


def ecef_position_error_label(axis_name: str) -> str:
    """Return a LaTeX-style ECEF position error legend label."""
    return rf"$\delta p_{{{axis_name}}}^e$"


def gnss_position_innovation_label(axis_name: str) -> str:
    """Return a LaTeX-style GNSS position innovation legend label."""
    return rf"$\nu_{{p_{axis_name}}}$"
