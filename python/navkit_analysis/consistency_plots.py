# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Interactive Plotly dashboards for Monte Carlo NEES/NIS consistency evidence."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Sequence

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.stats import chi2

from navkit_analysis.consistency import ConsistencySeries, chi_square_mean_bounds


DASHBOARD_DEFINITIONS = (
    (
        "full_ins_nees",
        "Full INS NEES Consistency",
        ("full_ins",),
    ),
    (
        "navigation_nees",
        "Navigation NEES Consistency",
        ("pva", "position", "velocity", "attitude"),
    ),
    (
        "imu_bias_nees",
        "IMU-Bias NEES Consistency",
        ("imu_bias", "gyro_bias", "accel_bias"),
    ),
    (
        "gnss_nis",
        "GNSS NIS Consistency",
        ("gnss_position", "gnss_velocity"),
    ),
)

MARGINAL_DASHBOARD_DEFINITIONS = (
    (
        "position_axes",
        "Position Axis Consistency",
        tuple(f"position_axes_{axis}" for axis in range(3)),
    ),
    (
        "velocity_axes",
        "Velocity Axis Consistency",
        tuple(f"velocity_axes_{axis}" for axis in range(3)),
    ),
    (
        "attitude_axes",
        "Attitude Axis Consistency",
        tuple(f"attitude_axes_{axis}" for axis in range(3)),
    ),
    (
        "gyro_bias_axes",
        "Gyro-Bias Axis Consistency",
        tuple(f"gyro_bias_axes_{axis}" for axis in range(3)),
    ),
    (
        "accel_bias_axes",
        "Accelerometer-Bias Axis Consistency",
        tuple(f"accel_bias_axes_{axis}" for axis in range(3)),
    ),
)

HEATMAP_DENSITY = "density"
HEATMAP_EMPIRICAL_CDF = "empirical_cdf"
HEATMAP_CDF_PROBABILITY_RESIDUAL = "cdf_probability_residual"
HEATMAP_MODES = (
    HEATMAP_DENSITY,
    HEATMAP_EMPIRICAL_CDF,
    HEATMAP_CDF_PROBABILITY_RESIDUAL,
)
REFERENCE_PROBABILITIES = (0.6827, 0.95, 0.99)


def _kind_label(kind: str) -> str:
    """Return a concise display label for one consistency statistic kind."""
    return {
        "nees": "NEES",
        "nis": "NIS",
        "marginal_nse": "Marginal NSE",
    }.get(kind, kind)


def _axis_ref(index: int, axis: str) -> str:
    suffix = "" if index == 1 else str(index)
    return f"{axis}{suffix}"


def _safe_nanmean(values: np.ndarray, axis: int) -> np.ndarray:
    with np.errstate(invalid="ignore"):
        return np.nanmean(values, axis=axis)


def _histogram_density_grid(series: ConsistencySeries, bin_count: int = 72) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    finite = series.values[np.isfinite(series.values)]
    if finite.size == 0:
        raise ValueError(f"consistency series '{series.name}' has no finite values")
    expected_upper = float(chi2.ppf(0.999, series.dof))
    observed_upper = float(np.quantile(finite, 0.995))
    upper = max(expected_upper, observed_upper, 1.0)
    edges = np.linspace(0.0, upper, bin_count + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    density = np.zeros((bin_count, series.values.shape[1]), dtype=float)
    for index in range(series.values.shape[1]):
        values = series.values[:, index]
        values = values[np.isfinite(values)]
        if values.size == 0:
            continue
        counts, _ = np.histogram(np.clip(values, edges[0], edges[-1]), bins=edges)
        density[:, index] = np.log1p(counts)
    return edges, centers, density


def _statistic_grid(series: ConsistencySeries, point_count: int = 128) -> np.ndarray:
    """Return a common raw-statistic grid spanning expected and observed tails."""
    finite = series.values[np.isfinite(series.values)]
    if finite.size == 0:
        raise ValueError(f"consistency series '{series.name}' has no finite values")
    expected_upper = float(chi2.ppf(0.999, series.dof))
    observed_upper = float(np.quantile(finite, 0.995))
    return np.linspace(0.0, max(expected_upper, observed_upper, 1.0), point_count)


def _empirical_cdf_grid(
    series: ConsistencySeries,
    point_count: int = 128,
) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate each epoch's exact empirical CDF on a common statistic grid."""
    statistic_grid = _statistic_grid(series, point_count)
    empirical_cdf = np.full(
        (point_count, series.values.shape[1]),
        np.nan,
        dtype=float,
    )
    for index in range(series.values.shape[1]):
        values = np.sort(series.values[:, index][np.isfinite(series.values[:, index])])
        if values.size == 0:
            continue
        empirical_cdf[:, index] = np.searchsorted(
            values,
            statistic_grid,
            side="right",
        ) / values.size
    return statistic_grid, empirical_cdf


def _cdf_probability_residual_grid(
    series: ConsistencySeries,
    point_count: int = 128,
) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate empirical-minus-uniform CDF residuals in chi-square PIT space."""
    probability_grid = np.linspace(0.0, 1.0, point_count)
    residual = np.full(
        (point_count, series.values.shape[1]),
        np.nan,
        dtype=float,
    )
    for index in range(series.values.shape[1]):
        values = series.values[:, index]
        values = values[np.isfinite(values)]
        if values.size == 0:
            continue
        transformed = np.sort(chi2.cdf(values, series.dof))
        empirical_cdf = np.searchsorted(
            transformed,
            probability_grid,
            side="right",
        ) / transformed.size
        residual[:, index] = empirical_cdf - probability_grid
    return probability_grid, residual


def _dashboard_payload(series_items: Sequence[ConsistencySeries]) -> str:
    """Serialize the browser-side data needed by interactive distributions."""
    groups: list[dict[str, object]] = []
    for series in series_items:
        run_count = series.values.shape[0]
        probabilities = (np.arange(1, run_count + 1) - 0.5) / run_count
        groups.append(
            {
                "name": series.name,
                "time_s": series.time_s.tolist(),
                "values": series.values.tolist(),
                "kind_label": _kind_label(series.kind),
                "reference_quantiles": chi2.ppf(probabilities, series.dof).tolist(),
            }
        )
    payload = {
        "groups": groups,
    }
    return json.dumps(payload, separators=(",", ":"))


def _dashboard_script(
    figure_id: str,
    payload_id: str,
    freeze_id: str,
    epoch_input_id: str,
    freeze_button_id: str,
    distribution_button_ids: dict[str, str],
    export_id: str,
    distribution_trace_indices: list[dict[str, list[int]]],
    row_axis_indices: list[int],
    right_axis_indices: list[int],
    output_name: str,
) -> str:
    distribution_indices = json.dumps(distribution_trace_indices)
    button_ids = json.dumps(distribution_button_ids)
    left_axis_indices = json.dumps(row_axis_indices)
    distribution_axis_indices = json.dumps(right_axis_indices)
    return f"""
<script>
(() => {{
  const figure = document.getElementById("{figure_id}");
  const payload = JSON.parse(document.getElementById("{payload_id}").textContent);
  const distributionTraceIndices = {distribution_indices};
  const rowAxisIndices = {left_axis_indices};
  const rightAxisIndices = {distribution_axis_indices};
  const freezeLabel = document.getElementById("{freeze_id}");
  const epochInput = document.getElementById("{epoch_input_id}");
  const freezeButton = document.getElementById("{freeze_button_id}");
  const distributionButtons = Object.fromEntries(
    Object.entries({button_ids}).map(([mode, id]) => [mode, document.getElementById(id)])
  );
  let selectedTime = payload.groups[0].time_s[payload.groups[0].time_s.length - 1];
  let frozen = false;
  let distributionMode = "pdf";
  epochInput.min = String(payload.groups[0].time_s[0]);
  epochInput.max = String(payload.groups[0].time_s[payload.groups[0].time_s.length - 1]);
  epochInput.step = "any";

  function nearestIndex(timeValues, target) {{
    let bestIndex = 0;
    let bestDistance = Math.abs(timeValues[0] - target);
    for (let index = 1; index < timeValues.length; ++index) {{
      const distance = Math.abs(timeValues[index] - target);
      if (distance < bestDistance) {{
        bestIndex = index;
        bestDistance = distance;
      }}
    }}
    return bestIndex;
  }}

  function sortedValues(group, index) {{
    return group.values
      .map((history) => history[index])
      .filter(Number.isFinite)
      .sort((left, right) => left - right);
  }}

  function updateDistribution(group, groupIndex, index, restylePromises) {{
    const values = sortedValues(group, index);
    const indices = distributionTraceIndices[groupIndex];
    if (distributionMode === "pdf") {{
      restylePromises.push(Plotly.restyle(figure, {{ x: [values] }}, [indices.pdf[0]]));
      return;
    }}
    const probabilities = values.map((_, valueIndex) => (valueIndex + 0.5) / values.length);
    if (distributionMode === "cdf") {{
      restylePromises.push(
        Plotly.restyle(figure, {{ x: [values], y: [probabilities] }}, [indices.cdf[0]])
      );
      return;
    }}
    restylePromises.push(
      Plotly.restyle(
        figure,
        {{ x: [group.reference_quantiles.slice(0, values.length)], y: [values] }},
        [indices.qq[0]],
      )
    );
  }}

  function axisName(axis, index) {{
    return axis + "axis" + (index === 1 ? "" : String(index));
  }}

  function setDistributionMode(nextMode) {{
    distributionMode = nextMode;
    const restylePromises = [];
    const axisUpdates = {{}};
    payload.groups.forEach((group, groupIndex) => {{
      const indices = distributionTraceIndices[groupIndex];
      const allIndices = [...indices.pdf, ...indices.cdf, ...indices.qq];
      const visible = allIndices.map((traceIndex) => indices[distributionMode].includes(traceIndex));
      restylePromises.push(Plotly.restyle(figure, {{ visible: visible }}, allIndices));
      const rightAxisIndex = rightAxisIndices[groupIndex];
      const xAxis = axisName("x", rightAxisIndex);
      const yAxis = axisName("y", rightAxisIndex);
      const isBottomRow = groupIndex === payload.groups.length - 1;
      if (distributionMode === "pdf") {{
        axisUpdates[xAxis + ".title.text"] = isBottomRow ? group.kind_label : "";
        axisUpdates[yAxis + ".title.text"] = "Density";
      }} else if (distributionMode === "cdf") {{
        axisUpdates[xAxis + ".title.text"] = isBottomRow ? group.kind_label : "";
        axisUpdates[yAxis + ".title.text"] = "Cumulative probability";
      }} else {{
        axisUpdates[xAxis + ".title.text"] =
          isBottomRow ? "Expected chi-square quantile" : "";
        axisUpdates[yAxis + ".title.text"] = "Observed statistic";
      }}
    }});
    Object.entries(distributionButtons).forEach(([mode, button]) => {{
      button.disabled = mode === distributionMode;
    }});
    restylePromises.push(Plotly.relayout(figure, axisUpdates));
    Promise.all(restylePromises).then(() => freezeAt(selectedTime));
  }}

  function freezeAt(time) {{
    selectedTime = time;
    const restylePromises = [];
    const shapes = [];
    payload.groups.forEach((group, groupIndex) => {{
      const index = nearestIndex(group.time_s, selectedTime);
      updateDistribution(group, groupIndex, index, restylePromises);
      const axisIndex = rowAxisIndices[groupIndex];
      const suffix = axisIndex === 1 ? "" : String(axisIndex);
      shapes.push({{
        type: "line",
        x0: group.time_s[index],
        x1: group.time_s[index],
        y0: 0,
        y1: 1,
        xref: "x" + suffix,
        yref: "y" + suffix + " domain",
        line: {{ color: "rgba(20,20,20,0.85)", width: 1.5, dash: "dot" }}
      }});
    }});
    epochInput.value = selectedTime.toFixed(6);
    restylePromises.push(Plotly.relayout(figure, {{ shapes: shapes }}));
    Promise.all(restylePromises);
  }}

  function setFrozen(nextFrozen) {{
    frozen = nextFrozen;
    freezeButton.textContent = frozen ? "Follow heatmap" : "Freeze epoch";
    freezeLabel.textContent = (frozen ? "Frozen epoch: " : "Following heatmap: ") + selectedTime.toFixed(3) + " s";
  }}

  function heatmapTime(event) {{
    if (!event.points) {{
      return null;
    }}
    const point = event.points.find((candidate) =>
      candidate.data && candidate.data.type === "heatmap" && Number.isFinite(candidate.x)
    );
    return point ? point.x : null;
  }}

  figure.on("plotly_click", (event) => {{
    const time = heatmapTime(event);
    if (time !== null) {{
      freezeAt(time);
      setFrozen(true);
    }}
  }});
  figure.on("plotly_hover", (event) => {{
    const time = heatmapTime(event);
    if (!frozen && time !== null) {{
      freezeAt(time);
      setFrozen(false);
    }}
  }});
  epochInput.addEventListener("change", () => {{
    const time = Number(epochInput.value);
    if (Number.isFinite(time)) {{
      freezeAt(time);
      setFrozen(true);
    }}
  }});
  freezeButton.addEventListener("click", () => {{
    setFrozen(!frozen);
  }});
  Object.entries(distributionButtons).forEach(([mode, button]) => {{
    button.addEventListener("click", () => setDistributionMode(mode));
  }});
  document.getElementById("{export_id}").addEventListener("click", () => {{
    Plotly.downloadImage(figure, {{ format: "png", filename: "{output_name}", scale: 2 }});
  }});
  setDistributionMode("pdf");
}})();
</script>
"""


def _heatmap_data(
    series: ConsistencySeries,
    mode: str,
) -> tuple[np.ndarray, np.ndarray, str, str, str, float | None, float | None, float | None]:
    """Prepare one left-panel heatmap without changing selected-epoch data."""
    if mode == HEATMAP_DENSITY:
        _, y_values, z_values = _histogram_density_grid(series)
        return (
            y_values,
            z_values,
            "Viridis",
            "log(1 + count)",
            "Statistic bin: %{y:.3f}<br>log(1 + count): %{z:.3f}",
            None,
            None,
            None,
        )
    if mode == HEATMAP_EMPIRICAL_CDF:
        y_values, z_values = _empirical_cdf_grid(series)
        return (
            y_values,
            z_values,
            "Viridis",
            "Empirical CDF",
            "Statistic threshold: %{y:.3f}<br>Empirical CDF: %{z:.4f}",
            0.0,
            1.0,
            None,
        )
    if mode == HEATMAP_CDF_PROBABILITY_RESIDUAL:
        y_values, z_values = _cdf_probability_residual_grid(series)
        finite = np.abs(z_values[np.isfinite(z_values)])
        limit = max(float(finite.max(initial=0.0)), 0.02)
        return (
            y_values,
            z_values,
            "RdBu_r",
            "Empirical CDF - expected",
            "CDF probability: %{y:.4f}<br>CDF residual: %{z:+.4f}",
            -limit,
            limit,
            0.0,
        )
    raise ValueError(f"unsupported consistency heatmap mode '{mode}'")


def _dashboard_title(title: str, mode: str) -> str:
    """Return a title that identifies the left-panel consistency diagnostic."""
    if mode == HEATMAP_DENSITY:
        return f"{title} - Occurrence Density"
    if mode == HEATMAP_EMPIRICAL_CDF:
        return f"{title} - Empirical CDF"
    if mode == HEATMAP_CDF_PROBABILITY_RESIDUAL:
        return f"{title} - CDF Probability-Space Residual"
    raise ValueError(f"unsupported consistency heatmap mode '{mode}'")


def _add_left_reference_traces(
    figure: go.Figure,
    series: ConsistencySeries,
    row: int,
    mode: str,
) -> None:
    """Overlay the reference appropriate to the selected heatmap diagnostic."""
    if mode == HEATMAP_DENSITY:
        mean = _safe_nanmean(series.values, axis=0)
        counts = np.sum(np.isfinite(series.values), axis=0)
        lower, upper = chi_square_mean_bounds(series.dof, counts)
        figure.add_trace(
            go.Scatter(
                x=series.time_s,
                y=mean,
                mode="lines",
                name="ensemble mean",
                line={"color": "white", "width": 2.0},
                hoverinfo="skip",
                showlegend=row == 1,
            ),
            row=row,
            col=1,
        )
        figure.add_trace(
            go.Scatter(
                x=series.time_s,
                y=upper,
                mode="lines",
                name="95% ensemble-mean consistency bounds",
                line={"color": "black", "dash": "dash", "width": 1.5},
                hoverinfo="skip",
                showlegend=row == 1,
            ),
            row=row,
            col=1,
        )
        figure.add_trace(
            go.Scatter(
                x=series.time_s,
                y=lower,
                mode="lines",
                name="95% ensemble-mean consistency bounds",
                legendgroup="95% ensemble-mean consistency bounds",
                line={"color": "black", "dash": "dash", "width": 1.5},
                hoverinfo="skip",
                showlegend=False,
            ),
            row=row,
            col=1,
        )
        return

    for probability, dash in zip(
        REFERENCE_PROBABILITIES,
        ("dot", "dash", "dashdot"),
        strict=True,
    ):
        threshold = (
            float(chi2.ppf(probability, series.dof))
            if mode == HEATMAP_EMPIRICAL_CDF
            else probability
        )
        reference_name = (
            f"expected {100.0 * probability:.2f}% quantile"
            if mode == HEATMAP_EMPIRICAL_CDF
            else f"expected CDF probability {100.0 * probability:.2f}%"
        )
        figure.add_trace(
            go.Scatter(
                x=(series.time_s[0], series.time_s[-1]),
                y=(threshold, threshold),
                mode="lines",
                name=reference_name,
                legendgroup=f"expected_{probability}",
                line={"color": "black", "dash": dash, "width": 1.3},
                hoverinfo="skip",
                showlegend=row == 1,
            ),
            row=row,
            col=1,
        )


def write_consistency_dashboard(
    title: str,
    series_items: Sequence[ConsistencySeries],
    output_path: Path,
    *,
    heatmap_mode: str = HEATMAP_DENSITY,
) -> Path:
    """Write one heatmap-linked consistency dashboard."""
    if not series_items:
        raise ValueError("consistency dashboard requires at least one series")
    if heatmap_mode not in HEATMAP_MODES:
        raise ValueError(f"unsupported consistency heatmap mode '{heatmap_mode}'")
    display_title = _dashboard_title(title, heatmap_mode)
    rows = len(series_items)
    figure = make_subplots(
        rows=rows,
        cols=2,
        column_widths=(0.68, 0.32),
        horizontal_spacing=0.07,
        vertical_spacing=0.08,
        subplot_titles=[
            title
            for row, series in enumerate(series_items)
            for title in (
                series.title,
                "Frozen epoch distribution" if row == 0 else "",
            )
        ],
    )
    distribution_trace_indices: list[dict[str, list[int]]] = []
    row_axis_indices: list[int] = []
    right_axis_indices: list[int] = []
    for row, series in enumerate(series_items, start=1):
        edges, _, _ = _histogram_density_grid(series)
        heatmap_y, heatmap_z, colorscale, colorbar_title, hover_detail, zmin, zmax, zmid = (
            _heatmap_data(series, heatmap_mode)
        )
        figure.add_trace(
            go.Heatmap(
                x=series.time_s,
                y=heatmap_y,
                z=heatmap_z,
                colorscale=colorscale,
                zmin=zmin,
                zmax=zmax,
                zmid=zmid,
                colorbar={
                    "title": colorbar_title,
                    "x": 0.665,
                    "xanchor": "center",
                    "y": 0.5,
                    "len": 0.90,
                    "thickness": 14,
                }
                if row == 1
                else None,
                hovertemplate=f"Time: %{{x:.3f}} s<br>{hover_detail}<extra></extra>",
                name=f"{series.name} {heatmap_mode}",
                showscale=row == 1,
            ),
            row=row,
            col=1,
        )
        _add_left_reference_traces(figure, series, row, heatmap_mode)
        initial_index = len(series.time_s) - 1
        initial_values = series.values[:, initial_index]
        initial_values = initial_values[np.isfinite(initial_values)]
        if initial_values.size == 0:
            initial_values = series.values[np.isfinite(series.values)]
        figure.add_trace(
            go.Histogram(
                x=initial_values,
                name="Monte Carlo samples",
                histnorm="probability density",
                xbins={"start": float(edges[0]), "end": float(edges[-1]), "size": float(edges[1] - edges[0])},
                marker={"color": "rgba(30,110,190,0.55)"},
                showlegend=False,
            ),
            row=row,
            col=2,
        )
        histogram_trace_index = len(figure.data) - 1
        x_pdf = np.linspace(edges[0], edges[-1], 256)
        figure.add_trace(
            go.Scatter(
                x=x_pdf,
                y=chi2.pdf(x_pdf, series.dof),
                mode="lines",
                name="chi-square PDF",
                line={"color": "black", "width": 1.5},
                showlegend=False,
            ),
            row=row,
            col=2,
        )
        pdf_trace_index = len(figure.data) - 1
        probabilities = (np.arange(1, len(initial_values) + 1) - 0.5) / len(initial_values)
        figure.add_trace(
            go.Scatter(
                x=np.sort(initial_values),
                y=probabilities,
                mode="lines",
                name="empirical CDF",
                line={"color": "rgb(30,110,190)", "width": 2.0},
                showlegend=False,
                visible=False,
            ),
            row=row,
            col=2,
        )
        empirical_cdf_trace_index = len(figure.data) - 1
        figure.add_trace(
            go.Scatter(
                x=x_pdf,
                y=chi2.cdf(x_pdf, series.dof),
                mode="lines",
                name="chi-square CDF",
                line={"color": "black", "dash": "dash", "width": 1.5},
                showlegend=False,
                visible=False,
            ),
            row=row,
            col=2,
        )
        chi_square_cdf_trace_index = len(figure.data) - 1
        expected_quantiles = chi2.ppf(probabilities, series.dof)
        figure.add_trace(
            go.Scatter(
                x=expected_quantiles,
                y=np.sort(initial_values),
                mode="markers",
                name="QQ samples",
                marker={"color": "rgb(30,110,190)", "size": 4},
                showlegend=False,
                visible=False,
            ),
            row=row,
            col=2,
        )
        qq_trace_index = len(figure.data) - 1
        diagonal_upper = max(float(expected_quantiles[-1]), float(initial_values.max()))
        figure.add_trace(
            go.Scatter(
                x=(0.0, diagonal_upper),
                y=(0.0, diagonal_upper),
                mode="lines",
                name="QQ identity",
                line={"color": "black", "dash": "dash", "width": 1.5},
                showlegend=False,
                visible=False,
            ),
            row=row,
            col=2,
        )
        qq_identity_trace_index = len(figure.data) - 1
        distribution_trace_indices.append(
            {
                "pdf": [histogram_trace_index, pdf_trace_index],
                "cdf": [empirical_cdf_trace_index, chi_square_cdf_trace_index],
                "qq": [qq_trace_index, qq_identity_trace_index],
            }
        )
        left_axis_index = 2 * row - 1
        row_axis_indices.append(left_axis_index)
        right_axis_indices.append(2 * row)
        kind_label = _kind_label(series.kind)
        left_y_title = (
            "CDF probability"
            if heatmap_mode == HEATMAP_CDF_PROBABILITY_RESIDUAL
            else f"{kind_label} [{series.dof} DOF]"
        )
        figure.update_yaxes(title_text=left_y_title, row=row, col=1)
        figure.update_xaxes(title_text="Time [s]" if row == rows else None, row=row, col=1)
        figure.update_xaxes(title_text=kind_label if row == rows else None, row=row, col=2)
        figure.update_yaxes(title_text="Density", row=row, col=2)

    for row in range(2, rows + 1):
        figure.update_xaxes(matches="x", row=row, col=1)

    figure.update_layout(
        title={
            "text": display_title,
            "x": 0.5,
            "xanchor": "center",
            "y": 0.99,
            "yanchor": "top",
        },
        template="plotly_white",
        height=max(380 * rows, 520),
        barmode="overlay",
        hovermode="closest",
        legend={
            "orientation": "h",
            "x": 0.5,
            "xanchor": "center",
            "y": 1.0,
            "yanchor": "bottom",
            "traceorder": "normal",
        },
        margin={"r": 30, "t": 145, "b": 60},
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure_id = "navkit_consistency_dashboard"
    payload_id = "navkit_consistency_payload"
    freeze_id = "navkit_consistency_frozen_epoch"
    epoch_input_id = "navkit_consistency_epoch_input"
    freeze_button_id = "navkit_consistency_epoch_mode"
    distribution_button_ids = {
        "pdf": "navkit_consistency_distribution_pdf",
        "cdf": "navkit_consistency_distribution_cdf",
        "qq": "navkit_consistency_distribution_qq",
    }
    export_id = "navkit_consistency_export"
    figure_html = figure.to_html(
        include_plotlyjs="cdn",
        full_html=False,
        div_id=figure_id,
        config={"doubleClick": "reset+autosize", "responsive": True, "scrollZoom": True},
    )
    payload = _dashboard_payload(series_items)
    safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", output_path.stem)
    document = f"""<!doctype html>
<html lang="en">
<head><meta charset="utf-8"><title>{display_title}</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 0; padding: 12px; }}
.navkit-controls {{ display: flex; flex-wrap: wrap; justify-content: flex-end; align-items: center; gap: 12px; margin: 0 0 8px 0; }}
.navkit-controls label {{ display: inline-flex; align-items: center; gap: 6px; }}
.navkit-controls input {{ width: 108px; padding: 5px; }}
.navkit-controls button:disabled {{ cursor: default; font-weight: bold; opacity: 0.70; }}
button {{ padding: 7px 11px; cursor: pointer; }}
</style></head>
<body>
<div class="navkit-controls"><strong id="{freeze_id}">Following heatmap</strong><label>Epoch [s]<input id="{epoch_input_id}" type="number"></label><button id="{freeze_button_id}">Freeze epoch</button><span>Distribution:</span><button id="{distribution_button_ids['pdf']}">PDF</button><button id="{distribution_button_ids['cdf']}">CDF</button><button id="{distribution_button_ids['qq']}">QQ</button><button id="{export_id}">Download snapshot</button></div>
{figure_html}
<script id="{payload_id}" type="application/json">{payload}</script>
{_dashboard_script(figure_id, payload_id, freeze_id, epoch_input_id, freeze_button_id, distribution_button_ids, export_id, distribution_trace_indices, row_axis_indices, right_axis_indices, safe_name)}
</body></html>
"""
    output_path.write_text(document, encoding="utf-8")
    print(f"Wrote {output_path}")
    return output_path


def write_consistency_dashboards(
    nees_series: Sequence[ConsistencySeries],
    nis_series: Sequence[ConsistencySeries],
    marginal_series: Sequence[ConsistencySeries],
    figures_dir: Path,
) -> dict[str, Path]:
    """Write the complete joint and marginal consistency dashboard set."""
    figures_dir.mkdir(parents=True, exist_ok=True)
    for stale_path in figures_dir.glob("consistency_*.html"):
        stale_path.unlink()
    by_name = {series.name: series for series in (*nees_series, *nis_series)}
    paths: dict[str, Path] = {}
    marginal_by_name = {series.name: series for series in marginal_series}
    for heatmap_mode in HEATMAP_MODES:
        mode_dir = figures_dir / heatmap_mode
        for name, title, group_names in DASHBOARD_DEFINITIONS:
            selected = [by_name[group_name] for group_name in group_names if group_name in by_name]
            if not selected:
                continue
            path = mode_dir / f"consistency_{name}.html"
            paths[f"{heatmap_mode}/{name}"] = write_consistency_dashboard(
                title,
                selected,
                path,
                heatmap_mode=heatmap_mode,
            )
        for name, title, group_names in MARGINAL_DASHBOARD_DEFINITIONS:
            selected = [
                marginal_by_name[group_name]
                for group_name in group_names
                if group_name in marginal_by_name
            ]
            if not selected:
                continue
            path = mode_dir / f"consistency_{name}.html"
            paths[f"{heatmap_mode}/{name}"] = write_consistency_dashboard(
                title,
                selected,
                path,
                heatmap_mode=heatmap_mode,
            )
    index_path = figures_dir / "index.html"
    sections: list[str] = []
    for heatmap_mode in HEATMAP_MODES:
        links = [
            f'<li><a href="{path.relative_to(figures_dir).as_posix()}">{name}</a></li>'
            for key, path in paths.items()
            if key.startswith(f"{heatmap_mode}/")
            for name in (key.split("/", maxsplit=1)[1].replace("_", " ").title(),)
        ]
        sections.append(
            f"<h2>{heatmap_mode.replace('_', ' ').title()}</h2><ul>{''.join(links)}</ul>"
        )
    index_path.write_text(
        "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
        "<title>NavKit Consistency Dashboards</title>"
        "<style>body{font-family:Arial,sans-serif;max-width:960px;margin:32px auto;"
        "padding:0 20px}li{margin:7px 0}</style></head><body>"
        "<h1>NavKit Consistency Dashboards</h1>"
        "<p>Choose an occurrence-density, empirical-CDF, or normalized "
        "CDF-residual diagnostic.</p>"
        f"{''.join(sections)}</body></html>",
        encoding="utf-8",
    )
    paths["index"] = index_path
    print(f"Wrote {index_path}")
    return paths
