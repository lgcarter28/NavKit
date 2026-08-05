# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pandas as pd

from navkit_analysis.figures.trajectory import (
    autopilot_response_plot_spec,
    body_trajectory_plot_spec,
    ecef_trajectory_plot_spec,
    eci_trajectory_plot_spec,
    guidance_control_plot_spec,
    guidance_plot_spec,
    lla_trajectory_3d_plot_spec,
    ned_trajectory_plot_spec,
    relative_ned_trajectory_3d_plot_spec,
    tracking_error_plot_spec,
    trajectory_plot_specs,
)
from navkit_analysis.renderers import render_plotly, render_plotly_3d
from navkit_analysis.trajectory_data import TrajectoryRunData


def _run_data(**frames: pd.DataFrame) -> TrajectoryRunData:
    return TrajectoryRunData(
        run_dir=Path("."),
        data_dir=Path("."),
        figures_dir=Path("."),
        **frames,
    )


class TrajectoryPlotTests(unittest.TestCase):
    def test_body_dashboard_uses_six_frame_explicit_panels(self) -> None:
        time_s = np.array([0.0, 1.0])
        frame = pd.DataFrame({"time_s": time_s})
        for prefix in (
            "v_ib_b",
            "a_ib_b",
            "v_eb_b",
            "a_eb_b",
            "specific_force_ib_b",
            "w_ib_b",
        ):
            units = (
                "mps"
                if prefix.startswith("v_")
                else "radps"
                if prefix.startswith("w_")
                else "mps2"
            )
            for axis_index, axis in enumerate(("x", "y", "z"), start=1):
                frame[f"{prefix}_{axis}_{units}"] = axis_index * np.ones_like(time_s)

        spec = body_trajectory_plot_spec(_run_data(body=frame))

        self.assertIsNotNone(spec)
        assert spec is not None
        self.assertEqual(
            [axis.title for axis in spec.axes],
            [
                "ECI-Relative Velocity Resolved in Body",
                "ECI-Relative Acceleration Resolved in Body",
                "ECEF-Relative Velocity Resolved in Body",
                "ECEF-Relative Acceleration Resolved in Body",
                "ECI Specific Force Resolved in Body",
                "Body Angular Rate With Respect to ECI",
            ],
        )
        self.assertEqual(spec.metadata["legend_scope"], "per_axis")
        self.assertEqual(
            [spec.axes[index].y_label for index in (1, 3, 4)],
            ["Acceleration [g]", "Acceleration [g]", "Specific force [g]"],
        )
        for axis_index in (1, 3, 4):
            np.testing.assert_allclose(
                spec.axes[axis_index].traces[0].y,
                np.full_like(time_s, 1.0 / 9.80665),
            )

    def test_relative_local_position_uses_north_east_up_from_initial_sample(
        self,
    ) -> None:
        earth_radius_m = 6_378_137.0
        ecef = pd.DataFrame(
            {
                "time_s": [0.0, 1.0],
                "p_e_x_m": [earth_radius_m, earth_radius_m + 30.0],
                "p_e_y_m": [0.0, 20.0],
                "p_e_z_m": [0.0, 10.0],
            }
        )
        ned = pd.DataFrame(
            {
                "time_s": [0.0, 1.0],
                "p_lla_lat_deg": [0.0, 0.0],
                "p_lla_lon_deg": [0.0, 0.0],
                "p_lla_h_m": [0.0, 30.0],
            }
        )

        spec = relative_ned_trajectory_3d_plot_spec(
            _run_data(ecef=ecef, ned=ned)
        )

        self.assertIsNotNone(spec)
        assert spec is not None
        self.assertEqual(
            (spec.x_label, spec.y_label, spec.z_label),
            ("North [m]", "East [m]", "Up [m]"),
        )
        np.testing.assert_allclose(spec.traces[0].x, [0.0, 10.0])
        np.testing.assert_allclose(spec.traces[0].y, [0.0, 20.0])
        np.testing.assert_allclose(spec.traces[0].z, [0.0, 30.0])
        self.assertEqual(spec.metadata["aspectmode"], "data")

    def test_plotly_legends_are_split_by_trajectory_subplot(self) -> None:
        frame = pd.DataFrame(
            {
                "time_s": [0.0],
                "p_lla_lat_deg": [0.0],
                "p_lla_lon_deg": [0.0],
                "p_lla_h_m": [0.0],
                "v_n_n_mps": [0.0],
                "v_n_e_mps": [0.0],
                "v_n_d_mps": [0.0],
                "a_n_n_mps2": [0.0],
                "a_n_e_mps2": [0.0],
                "a_n_d_mps2": [0.0],
                "q_b2n_w": [1.0],
                "q_b2n_x": [0.0],
                "q_b2n_y": [0.0],
                "q_b2n_z": [0.0],
                "w_nb_b_x_radps": [0.0],
                "w_nb_b_y_radps": [0.0],
                "w_nb_b_z_radps": [0.0],
            }
        )
        spec = ned_trajectory_plot_spec(_run_data(ned=frame))
        assert spec is not None

        figure = render_plotly(spec)
        visible_by_legend: dict[str, list[str]] = {}
        for trace in figure.data:
            if bool(trace.showlegend):
                visible_by_legend.setdefault(str(trace.legend), []).append(trace.name)

        self.assertEqual(
            visible_by_legend,
            {
                "legend": ["Latitude"],
                "legend2": ["Longitude"],
                "legend3": ["Height"],
                "legend4": ["North", "East", "Down"],
                "legend5": ["North", "East", "Down"],
                "legend6": ["Roll", "Pitch", "Yaw"],
                "legend7": ["P", "Q", "R"],
            },
        )
        self.assertAlmostEqual(figure.layout.legend.y, figure.layout.yaxis.domain[1])
        self.assertAlmostEqual(figure.layout.legend7.y, figure.layout.yaxis7.domain[1])

    def test_all_trajectory_acceleration_axes_use_standard_g_units(self) -> None:
        time_s = [0.0]
        acceleration_mps2 = 9.80665

        def frame_with_vectors(
            prefixes: tuple[tuple[str, str], ...],
            axes: tuple[str, str, str] = ("x", "y", "z"),
        ) -> pd.DataFrame:
            values: dict[str, list[float]] = {"time_s": time_s}
            for prefix, units in prefixes:
                for axis in axes:
                    values[f"{prefix}_{axis}_{units}"] = [
                        acceleration_mps2 if units == "mps2" else 0.0
                    ]
            return pd.DataFrame(values)

        ecef = frame_with_vectors(
            (
                ("p_e", "m"),
                ("v_e", "mps"),
                ("a_e", "mps2"),
                ("w_eb_b", "radps"),
            )
        )
        eci = frame_with_vectors(
            (
                ("p_i", "m"),
                ("v_i", "mps"),
                ("a_i", "mps2"),
                ("w_ib_b", "radps"),
            )
        )
        ned = frame_with_vectors(
            (("v_n", "mps"), ("a_n", "mps2")), axes=("n", "e", "d")
        )
        for axis in ("x", "y", "z"):
            ned[f"w_nb_b_{axis}_radps"] = [0.0]
        ned["p_lla_lat_deg"] = [0.0]
        ned["p_lla_lon_deg"] = [0.0]
        ned["p_lla_h_m"] = [0.0]
        body = frame_with_vectors(
            (
                ("v_ib_b", "mps"),
                ("a_ib_b", "mps2"),
                ("v_eb_b", "mps"),
                ("a_eb_b", "mps2"),
                ("specific_force_ib_b", "mps2"),
                ("w_ib_b", "radps"),
            )
        )
        guidance = frame_with_vectors(
            (
                ("guidance_acceleration_command_n", "mps2"),
                ("guidance_acceleration_response_n", "mps2"),
            ),
            axes=("n", "e", "d"),
        )
        guidance_body = frame_with_vectors(
            (
                ("guidance_acceleration_command_b", "mps2"),
                ("guidance_acceleration_response_b", "mps2"),
                ("guidance_specific_force_command_b", "mps2"),
                ("guidance_specific_force_filtered_b", "mps2"),
            )
        )
        for column in guidance_body.columns:
            if column != "time_s":
                guidance[column] = guidance_body[column]
        guidance["guidance_bank_command_n_rad"] = [0.0]
        guidance["guidance_bank_response_n_rad"] = [0.0]
        guidance["guidance_state_index"] = [0]
        autopilot = frame_with_vectors(
            (
                ("autopilot_angular_rate_command_b", "radps"),
                ("autopilot_angular_rate_controller_response_b", "radps"),
                ("velocity_tracking_error_b", "mps"),
                ("acceleration_tracking_error_b", "mps2"),
                ("attitude_tracking_error_b", "rad"),
                ("angular_rate_tracking_error_b", "radps"),
                ("specific_force_tracking_error_b", "mps2"),
            )
        )
        for frame, prefix in (
            (ecef, "q_b2e"),
            (eci, "q_b2i"),
            (ned, "q_b2n"),
        ):
            for component in ("w", "x", "y", "z"):
                frame[f"{prefix}_{component}"] = [1.0 if component == "w" else 0.0]
        for kind in ("command", "response"):
            for component in ("w", "x", "y", "z"):
                autopilot[f"autopilot_q_{kind}_b2n_{component}"] = [
                    1.0 if component == "w" else 0.0
                ]

        run = _run_data(
            ecef=ecef,
            eci=eci,
            ned=ned,
            body=body,
            guidance=guidance,
            autopilot_vehicle=autopilot,
        )
        specs = (
            ecef_trajectory_plot_spec(run),
            eci_trajectory_plot_spec(run),
            ned_trajectory_plot_spec(run),
            body_trajectory_plot_spec(run),
            guidance_plot_spec(run),
            tracking_error_plot_spec(run),
            guidance_control_plot_spec(run),
        )
        acceleration_axis_count = 0
        for spec in specs:
            assert spec is not None
            for axis in spec.axes:
                is_acceleration = "Acceleration" in axis.title
                is_specific_force = (
                    "Specific-Force" in axis.title or "Specific Force" in axis.title
                )
                if not is_acceleration and not is_specific_force:
                    continue
                acceleration_axis_count += 1
                self.assertIn("[g]", axis.y_label)
                for trace in axis.traces:
                    np.testing.assert_allclose(trace.y, [1.0])
        self.assertEqual(acceleration_axis_count, 13)

    def test_interactive_html_enables_scroll_zoom_and_double_click_reset(self) -> None:
        frame = pd.DataFrame(
            {
                "time_s": [0.0],
                **{
                    f"{prefix}_{axis}_{units}": [0.0]
                    for prefix, units in (
                        ("v_ib_b", "mps"),
                        ("a_ib_b", "mps2"),
                        ("v_eb_b", "mps"),
                        ("a_eb_b", "mps2"),
                        ("specific_force_ib_b", "mps2"),
                        ("w_ib_b", "radps"),
                    )
                    for axis in ("x", "y", "z")
                },
            }
        )
        line_spec = body_trajectory_plot_spec(_run_data(body=frame))
        assert line_spec is not None
        trajectory_3d_spec = lla_trajectory_3d_plot_spec(
            _run_data(
                ned=pd.DataFrame(
                    {
                        "time_s": [0.0],
                        "p_lla_lat_deg": [0.0],
                        "p_lla_lon_deg": [0.0],
                        "p_lla_h_m": [0.0],
                    }
                )
            )
        )
        assert trajectory_3d_spec is not None

        with TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            paths = (directory / "line.html", directory / "trajectory.html")
            render_plotly(line_spec, paths[0])
            render_plotly_3d(trajectory_3d_spec, paths[1])
            for path in paths:
                html = path.read_text(encoding="utf-8")
                self.assertIn('"scrollZoom": true', html)
                self.assertIn('"doubleClick": "reset+autosize"', html)

    def test_autopilot_plot_omits_moving_average_and_registry_uses_new_names(
        self,
    ) -> None:
        frame = pd.DataFrame(
            {
                "time_s": [0.0],
                **{
                    f"autopilot_q_{kind}_b2n_{component}": [1.0 if component == "w" else 0.0]
                    for kind in ("command", "response")
                    for component in ("w", "x", "y", "z")
                },
                **{
                    f"autopilot_angular_rate_{kind}_b_{axis}_radps": [0.0]
                    for kind in ("command", "controller_response")
                    for axis in ("x", "y", "z")
                },
                **{
                    f"autopilot_gyro_observation_b_{axis}_radps": [0.0]
                    for axis in ("x", "y", "z")
                },
                **{
                    f"{prefix}_b_{axis}_{units}": [0.0]
                    for prefix, units in (
                        ("velocity_tracking_error", "mps"),
                        ("acceleration_tracking_error", "mps2"),
                        ("attitude_tracking_error", "rad"),
                        ("angular_rate_tracking_error", "radps"),
                        ("specific_force_tracking_error", "mps2"),
                    )
                    for axis in ("x", "y", "z")
                },
            }
        )
        run = _run_data(autopilot_vehicle=frame)

        spec = autopilot_response_plot_spec(run)
        assert spec is not None
        self.assertEqual(len(spec.axes), 2)
        self.assertFalse(
            any("Observation" in axis.title for axis in spec.axes)
        )

        plot_names = trajectory_plot_specs(run)
        self.assertIn("autopilot_response", plot_names)
        self.assertNotIn("vehicle_response_body", plot_names)
        self.assertNotIn("nested_loop_response_body", plot_names)

    def test_guidance_markers_follow_logged_state_indices(self) -> None:
        ned = pd.DataFrame(
            {
                "time_s": [0.0, 1.0, 2.0],
                "p_lla_lat_deg": [35.0, 35.001, 35.002],
                "p_lla_lon_deg": [-106.0, -106.0, -106.0],
                "p_lla_h_m": [1500.0, 1500.0, 1500.0],
            }
        )
        guidance = pd.DataFrame(
            {
                "time_s": [0.0, 1.0, 2.0],
                "guidance_state_index": [5, 5, 1],
            }
        )

        spec = lla_trajectory_3d_plot_spec(
            _run_data(ned=ned, guidance=guidance)
        )

        self.assertIsNotNone(spec)
        assert spec is not None
        self.assertEqual(
            spec.traces[1].label,
            "Guidance state 5 entered",
        )
        self.assertEqual(spec.traces[2].label, "Guidance state 1 entered")


if __name__ == "__main__":
    unittest.main()
