# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Renderer-neutral prepared plotting data for static and interactive analysis."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from navkit_analysis.schema import PLOT_SPEC_SCHEMA


@dataclass(frozen=True)
class PlotTrace:
    """One prepared line or a collection of same-style line histories."""

    x: np.ndarray
    y: np.ndarray
    label: str
    color: str
    line_style: str = "solid"
    line_width: float = 1.5
    opacity: float = 1.0
    show_legend: bool = True


@dataclass(frozen=True)
class PlotAxis:
    """Prepared traces and labels for one vertically stacked axis."""

    title: str
    y_label: str
    traces: tuple[PlotTrace, ...]
    zero_line: bool = True


@dataclass(frozen=True)
class PlotSpec:
    """Complete renderer-independent description of one analysis figure."""

    title: str
    x_label: str
    axes: tuple[PlotAxis, ...]
    output_name: str
    schema: str = PLOT_SPEC_SCHEMA
    metadata: Mapping[str, object] = field(default_factory=dict)


def quick_xy_plot_spec(
    x: np.ndarray,
    y: np.ndarray,
    *,
    title: str,
    x_label: str,
    y_label: str,
    series_label: str,
    output_name: str,
) -> PlotSpec:
    """Build a minimal reusable plot specification for ad hoc field inspection."""
    trace = PlotTrace(x=x, y=y, label=series_label, color="tab:blue")
    axis = PlotAxis(title="", y_label=y_label, traces=(trace,), zero_line=False)
    return PlotSpec(title=title, x_label=x_label, axes=(axis,), output_name=output_name)
