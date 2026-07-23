# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Static and interactive renderers for shared NavKit plot specifications."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
from matplotlib.collections import LineCollection
from matplotlib.colors import to_hex
from plotly.subplots import make_subplots

from navkit_analysis.plot_spec import PlotSpec, PlotTrace
from navkit_analysis.style import apply_nav_axes_style, apply_style


def _matplotlib_line_style(line_style: str) -> str:
    return {"solid": "-", "dash": "--", "dot": ":"}.get(line_style, "-")


def _plot_matplotlib_trace(ax: plt.Axes, trace: PlotTrace) -> None:
    y = np.asarray(trace.y)
    if y.ndim == 1:
        ax.plot(
            trace.x,
            y,
            color=trace.color,
            linestyle=_matplotlib_line_style(trace.line_style),
            linewidth=trace.line_width,
            alpha=trace.opacity,
            label=trace.label if trace.show_legend else None,
        )
        return
    if y.ndim != 2:
        raise ValueError(f"plot trace '{trace.label}' must contain one or two dimensions")
    segments = [np.column_stack((trace.x, row)) for row in y]
    collection = LineCollection(
        segments,
        colors=trace.color,
        linestyles=_matplotlib_line_style(trace.line_style),
        linewidths=trace.line_width,
        alpha=trace.opacity,
        label=trace.label if trace.show_legend else None,
    )
    ax.add_collection(collection)
    ax.update_datalim(np.column_stack((np.tile(trace.x, y.shape[0]), y.ravel())))
    ax.autoscale_view()


def render_matplotlib(spec: PlotSpec, output_path: Path | None = None) -> plt.Figure:
    """Render a plot specification as a publication-quality Matplotlib figure."""
    apply_style()
    figure, axes = plt.subplots(
        nrows=len(spec.axes),
        ncols=1,
        sharex=True,
        figsize=(14.0, max(3.0 * len(spec.axes), 4.0)),
        constrained_layout=True,
    )
    axis_list = [axes] if len(spec.axes) == 1 else list(axes)
    figure.suptitle(spec.title)

    for axis, axis_spec in zip(axis_list, spec.axes):
        for trace in axis_spec.traces:
            _plot_matplotlib_trace(axis, trace)
        if axis_spec.zero_line:
            axis.axhline(0.0, color="0.25", linewidth=0.8)
        if axis_spec.title:
            axis.set_title(axis_spec.title)
        axis.set_ylabel(axis_spec.y_label)
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            axis.legend(handles, labels, loc="upper right")
        apply_nav_axes_style(axis)

    axis_list[-1].set_xlabel(spec.x_label)
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=120, bbox_inches=None)
        print(f"Wrote {output_path}")
    return figure


def _plotly_dash(line_style: str) -> str:
    return {"solid": "solid", "dash": "dash", "dot": "dot"}.get(line_style, "solid")


def _plotly_color(color: str) -> str:
    """Translate Matplotlib-compatible colors into CSS-safe Plotly color values."""
    return to_hex(color)


def _plot_plotly_trace(
    figure: go.Figure,
    trace: PlotTrace,
    row: int,
    show_legend: bool,
) -> None:
    y = np.asarray(trace.y)
    histories = y[None, :] if y.ndim == 1 else y
    if histories.ndim != 2:
        raise ValueError(f"plot trace '{trace.label}' must contain one or two dimensions")
    for index, history in enumerate(histories):
        figure.add_trace(
            go.Scattergl(
                x=trace.x,
                y=history,
                mode="lines",
                name=trace.label,
                legendgroup=trace.label,
                showlegend=show_legend and trace.show_legend and index == 0,
                opacity=trace.opacity,
                line={
                    "color": _plotly_color(trace.color),
                    "dash": _plotly_dash(trace.line_style),
                    "width": trace.line_width,
                },
            ),
            row=row,
            col=1,
        )


def render_plotly(spec: PlotSpec, output_path: Path | None = None) -> go.Figure:
    """Render a plot specification as a responsive Plotly HTML-ready figure."""
    subplot_titles = [axis.title for axis in spec.axes]
    figure = make_subplots(
        rows=len(spec.axes),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.04,
        subplot_titles=subplot_titles,
    )
    for row, axis_spec in enumerate(spec.axes, start=1):
        for trace in axis_spec.traces:
            _plot_plotly_trace(
                figure,
                trace,
                row,
                show_legend=row == 1,
            )
        if axis_spec.zero_line:
            figure.add_hline(y=0.0, line_color="rgba(64,64,64,0.8)", line_width=1, row=row, col=1)
        figure.update_yaxes(title_text=axis_spec.y_label, row=row, col=1)
    figure.update_xaxes(title_text=spec.x_label, row=len(spec.axes), col=1)
    layout: dict[str, object] = {
        "title": spec.title,
        "height": max(300 * len(spec.axes), 450),
        "template": "plotly_white",
        "hovermode": False,
        "legend": {"traceorder": "normal", "groupclick": "togglegroup", "x": 1.02, "y": 0.90},
        "margin": {"r": 230},
    }
    layout["updatemenus"] = [
        {
            "type": "buttons",
            "showactive": True,
            "x": 1.02,
            "xanchor": "left",
            "y": 1.08,
            "yanchor": "top",
            "buttons": [
                {
                    "label": "Toggle hover details",
                    "method": "relayout",
                    "args": [{"hovermode": "x unified"}],
                    "args2": [{"hovermode": False}],
                }
            ],
        }
    ]
    figure.update_layout(**layout)
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.write_html(output_path, include_plotlyjs="cdn", full_html=True)
        print(f"Wrote {output_path}")
    return figure
