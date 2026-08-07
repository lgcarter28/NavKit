# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


AXES = ("x", "y", "z")


def save_figure(fig: plt.Figure, out: Path) -> Path:
    out.parent.mkdir(parents=True, exist_ok=True)
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


def plot_runtime_innovation_gate(ax: plt.Axes, updates: pd.DataFrame) -> None:
    """Overlay the configured runtime NIS gate and rejected observations when logged."""

    required_columns = {
        "time_s",
        "nis",
        "accepted",
        "gate_enabled",
        "gate_probability",
        "gate_threshold",
    }
    if not required_columns.issubset(updates.columns):
        return

    time_s = updates["time_s"].to_numpy(dtype=float)
    nis = updates["nis"].to_numpy(dtype=float)
    accepted = updates["accepted"].to_numpy(dtype=float) > 0.5
    gate_enabled = updates["gate_enabled"].to_numpy(dtype=float) > 0.5
    gate_probability = updates["gate_probability"].to_numpy(dtype=float)
    gate_threshold = updates["gate_threshold"].to_numpy(dtype=float)
    valid_gate = gate_enabled & np.isfinite(gate_threshold)

    if np.any(valid_gate):
        unique_probabilities = np.unique(gate_probability[valid_gate])
        label = "runtime acceptance gate"
        if unique_probabilities.size == 1:
            label += f" (P={unique_probabilities[0]:.6g})"
        threshold_trace = np.where(valid_gate, gate_threshold, np.nan)
        ax.plot(
            time_s,
            threshold_trace,
            color="tab:blue",
            linestyle="--",
            marker=".",
            markersize=2.0,
            label=label,
        )

    rejected = (~accepted) & np.isfinite(nis)
    if np.any(rejected):
        ax.scatter(
            time_s[rejected],
            nis[rejected],
            color="red",
            marker="x",
            s=48,
            label="rejected observation",
            zorder=5,
        )


def plot_runtime_p_value_gate(
    ax: plt.Axes,
    updates: pd.DataFrame,
    p_value: np.ndarray,
) -> None:
    """Overlay the upper-tail p-value rejection threshold and rejected observations."""

    required_columns = {"time_s", "accepted", "gate_enabled", "gate_probability"}
    if not required_columns.issubset(updates.columns):
        return

    time_s = updates["time_s"].to_numpy(dtype=float)
    accepted = updates["accepted"].to_numpy(dtype=float) > 0.5
    gate_enabled = updates["gate_enabled"].to_numpy(dtype=float) > 0.5
    gate_probability = updates["gate_probability"].to_numpy(dtype=float)
    valid_gate = gate_enabled & np.isfinite(gate_probability)

    if np.any(valid_gate):
        rejection_probability = 1.0 - gate_probability
        unique_thresholds = np.unique(rejection_probability[valid_gate])
        label = "runtime rejection threshold"
        if unique_thresholds.size == 1:
            label += f" (p={unique_thresholds[0]:.6g})"
        threshold_trace = np.where(valid_gate, rejection_probability, np.nan)
        ax.plot(
            time_s,
            threshold_trace,
            color="tab:blue",
            linestyle="--",
            marker=".",
            markersize=2.0,
            label=label,
        )

    rejected = (~accepted) & np.isfinite(p_value)
    if np.any(rejected):
        ax.scatter(
            time_s[rejected],
            p_value[rejected],
            color="red",
            marker="x",
            s=48,
            label="rejected observation",
            zorder=5,
        )
