from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


AXES = ("x", "y", "z")


def save_figure(fig: plt.Figure, out: Path) -> Path:
    fig.savefig(out)
    print(f"Wrote {out}")
    return out


def maybe_close_figures(figures: list[plt.Figure], show: bool) -> None:
    """Show all figures at once or close all figures after saving.

    Calling plt.show() only once prevents the script from blocking after each
    individual figure.
    """
    if show:
        plt.show()
        return

    for fig in figures:
        plt.close(fig)
