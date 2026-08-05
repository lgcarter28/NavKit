# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Renderer-neutral trajectory truth and command/response plot preparation."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pandas as pd

from navkit_analysis.plot_spec import Plot3DSpec, Plot3DTrace, PlotAxis, PlotSpec, PlotTrace
from navkit_analysis.trajectory_data import TrajectoryRunData

AXIS_COLORS = ("tab:red", "tab:blue", "tab:green")
STANDARD_GRAVITY_MPS2 = 9.80665
MPS2_TO_G = 1.0 / STANDARD_GRAVITY_MPS2


def _traces(
    frame: pd.DataFrame,
    columns: Sequence[str],
    labels: Sequence[str],
    colors: Sequence[str],
    *,
    scale: float = 1.0,
    line_style: str = "solid",
) -> tuple[PlotTrace, ...]:
    time_s = frame["time_s"].to_numpy()
    return tuple(
        PlotTrace(
            x=time_s,
            y=frame[column].to_numpy() * scale,
            label=label,
            color=color,
            line_style=line_style,
        )
        for column, label, color in zip(columns, labels, colors)
    )


def _vector_axis(
    frame: pd.DataFrame,
    columns: Sequence[str],
    title: str,
    y_label: str,
    labels: Sequence[str] = ("X", "Y", "Z"),
    *,
    scale: float = 1.0,
) -> PlotAxis:
    return PlotAxis(
        title=title,
        y_label=y_label,
        traces=_traces(frame, columns, labels, AXIS_COLORS, scale=scale),
        zero_line=True,
    )


def _rpy_deg_from_quaternion_wxyz(frame: pd.DataFrame, prefix: str) -> np.ndarray:
    quaternion = frame[
        [f"{prefix}_w", f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"]
    ].to_numpy()
    norms = np.linalg.norm(quaternion, axis=1)
    norms[norms == 0.0] = 1.0
    quaternion = quaternion / norms[:, None]
    w = quaternion[:, 0]
    x = quaternion[:, 1]
    y = quaternion[:, 2]
    z = quaternion[:, 3]
    roll_rad = np.arctan2(2.0 * ((w * x) + (y * z)), 1.0 - (2.0 * ((x * x) + (y * y))))
    pitch_argument = np.clip(2.0 * ((w * y) - (z * x)), -1.0, 1.0)
    pitch_rad = np.arcsin(pitch_argument)
    yaw_rad = np.arctan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))
    return np.rad2deg(np.column_stack((roll_rad, pitch_rad, yaw_rad)))


def _rpy_axis(frame: pd.DataFrame, prefix: str, title: str) -> PlotAxis:
    rpy_deg = _rpy_deg_from_quaternion_wxyz(frame, prefix)
    time_s = frame["time_s"].to_numpy()
    traces = tuple(
        PlotTrace(
            x=time_s,
            y=rpy_deg[:, axis_index],
            label=label,
            color=color,
        )
        for axis_index, (label, color) in enumerate(
            zip(("Roll", "Pitch", "Yaw"), AXIS_COLORS)
        )
    )
    return PlotAxis(title=title, y_label="Angle [deg]", traces=traces, zero_line=True)


def ecef_trajectory_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build the ECEF kinematics truth dashboard."""
    frame = run.ecef
    if frame is None:
        return None
    return PlotSpec(
        title="Trajectory Kinematics - ECEF",
        x_label="Time [s]",
        output_name="trajectory_kinematics_ecef.html",
        axes=(
            _vector_axis(
                frame,
                ("p_e_x_m", "p_e_y_m", "p_e_z_m"),
                "ECEF Position",
                "Position [m]",
            ),
            _vector_axis(
                frame,
                ("v_e_x_mps", "v_e_y_mps", "v_e_z_mps"),
                "ECEF Velocity",
                "Velocity [m/s]",
            ),
            _vector_axis(
                frame,
                ("a_e_x_mps2", "a_e_y_mps2", "a_e_z_mps2"),
                "ECEF Acceleration",
                "Acceleration [g]",
                scale=MPS2_TO_G,
            ),
            _rpy_axis(frame, "q_b2e", "Body-to-ECEF Euler Attitude"),
            _vector_axis(
                frame,
                ("w_eb_b_x_radps", "w_eb_b_y_radps", "w_eb_b_z_radps"),
                "Body Rate With Respect to ECEF",
                "Rate [deg/s]",
                labels=("P", "Q", "R"),
                scale=180.0 / np.pi,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def eci_trajectory_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build the ECI kinematics truth dashboard."""
    frame = run.eci
    if frame is None:
        return None
    return PlotSpec(
        title="Trajectory Kinematics - ECI",
        x_label="Time [s]",
        output_name="trajectory_kinematics_eci.html",
        axes=(
            _vector_axis(
                frame,
                ("p_i_x_m", "p_i_y_m", "p_i_z_m"),
                "ECI Position",
                "Position [m]",
            ),
            _vector_axis(
                frame,
                ("v_i_x_mps", "v_i_y_mps", "v_i_z_mps"),
                "ECI Velocity",
                "Velocity [m/s]",
            ),
            _vector_axis(
                frame,
                ("a_i_x_mps2", "a_i_y_mps2", "a_i_z_mps2"),
                "ECI Acceleration",
                "Acceleration [g]",
                scale=MPS2_TO_G,
            ),
            _rpy_axis(frame, "q_b2i", "Body-to-ECI Euler Attitude"),
            _vector_axis(
                frame,
                ("w_ib_b_x_radps", "w_ib_b_y_radps", "w_ib_b_z_radps"),
                "Body Rate With Respect to ECI",
                "Rate [deg/s]",
                labels=("P", "Q", "R"),
                scale=180.0 / np.pi,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def ned_trajectory_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build the geodetic/local-level trajectory truth dashboard."""
    frame = run.ned
    if frame is None:
        return None
    time_s = frame["time_s"].to_numpy()
    return PlotSpec(
        title="Trajectory Kinematics - Local NED",
        x_label="Time [s]",
        output_name="trajectory_kinematics_ned.html",
        axes=(
            PlotAxis(
                title="Geodetic Latitude",
                y_label="Latitude [deg]",
                traces=(
                    PlotTrace(
                        x=time_s,
                        y=frame["p_lla_lat_deg"].to_numpy(),
                        label="Latitude",
                        color="tab:red",
                    ),
                ),
                zero_line=False,
            ),
            PlotAxis(
                title="Geodetic Longitude",
                y_label="Longitude [deg]",
                traces=(
                    PlotTrace(
                        x=time_s,
                        y=frame["p_lla_lon_deg"].to_numpy(),
                        label="Longitude",
                        color="tab:blue",
                    ),
                ),
                zero_line=False,
            ),
            PlotAxis(
                title="Geodetic Height",
                y_label="Height [m]",
                traces=(
                    PlotTrace(
                        x=time_s,
                        y=frame["p_lla_h_m"].to_numpy(),
                        label="Height",
                        color="tab:green",
                    ),
                ),
                zero_line=False,
            ),
            _vector_axis(
                frame,
                ("v_n_n_mps", "v_n_e_mps", "v_n_d_mps"),
                "NED Velocity",
                "Velocity [m/s]",
                labels=("North", "East", "Down"),
            ),
            _vector_axis(
                frame,
                ("a_n_n_mps2", "a_n_e_mps2", "a_n_d_mps2"),
                "NED Acceleration",
                "Acceleration [g]",
                labels=("North", "East", "Down"),
                scale=MPS2_TO_G,
            ),
            _rpy_axis(frame, "q_b2n", "Body-to-NED Attitude"),
            _vector_axis(
                frame,
                ("w_nb_b_x_radps", "w_nb_b_y_radps", "w_nb_b_z_radps"),
                "Body Rate With Respect to NED",
                "Rate [deg/s]",
                labels=("P", "Q", "R"),
                scale=180.0 / np.pi,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def body_trajectory_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build the body-resolved trajectory truth dashboard."""
    frame = run.body
    if frame is None:
        return None
    return PlotSpec(
        title="Trajectory Kinematics - Body Resolved",
        x_label="Time [s]",
        output_name="trajectory_kinematics_body.html",
        axes=(
            _vector_axis(
                frame,
                ("v_ib_b_x_mps", "v_ib_b_y_mps", "v_ib_b_z_mps"),
                "ECI-Relative Velocity Resolved in Body",
                "Velocity [m/s]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
            ),
            _vector_axis(
                frame,
                ("a_ib_b_x_mps2", "a_ib_b_y_mps2", "a_ib_b_z_mps2"),
                "ECI-Relative Acceleration Resolved in Body",
                "Acceleration [g]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
                scale=MPS2_TO_G,
            ),
            _vector_axis(
                frame,
                ("v_eb_b_x_mps", "v_eb_b_y_mps", "v_eb_b_z_mps"),
                "ECEF-Relative Velocity Resolved in Body",
                "Velocity [m/s]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
            ),
            _vector_axis(
                frame,
                ("a_eb_b_x_mps2", "a_eb_b_y_mps2", "a_eb_b_z_mps2"),
                "ECEF-Relative Acceleration Resolved in Body",
                "Acceleration [g]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
                scale=MPS2_TO_G,
            ),
            _vector_axis(
                frame,
                (
                    "specific_force_ib_b_x_mps2",
                    "specific_force_ib_b_y_mps2",
                    "specific_force_ib_b_z_mps2",
                ),
                "ECI Specific Force Resolved in Body",
                "Specific force [g]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
                scale=MPS2_TO_G,
            ),
            _vector_axis(
                frame,
                ("w_ib_b_x_radps", "w_ib_b_y_radps", "w_ib_b_z_radps"),
                "Body Angular Rate With Respect to ECI",
                "Rate [deg/s]",
                labels=("P", "Q", "R"),
                scale=180.0 / np.pi,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def _guidance_event_marker_traces(
    run: TrajectoryRunData,
    trajectory_time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
) -> tuple[Plot3DTrace, ...]:
    """Create markers for logged Guidance-state and waypoint transitions."""
    guidance = run.guidance
    if guidance is None or "guidance_state_index" not in guidance:
        return ()
    state_index = guidance["guidance_state_index"].to_numpy(dtype=int)
    if len(state_index) == 0:
        return ()
    transition_indices = np.flatnonzero(
        np.concatenate(([True], state_index[1:] != state_index[:-1]))
    )
    traces: list[Plot3DTrace] = []
    for index in transition_indices:
        transition_time_s = float(guidance["time_s"].iloc[index])
        entered_state_index = int(state_index[index])
        traces.append(
            Plot3DTrace(
                x=np.array([np.interp(transition_time_s, trajectory_time_s, x)]),
                y=np.array([np.interp(transition_time_s, trajectory_time_s, y)]),
                z=np.array([np.interp(transition_time_s, trajectory_time_s, z)]),
                label=f"Guidance state {entered_state_index} entered",
                color="black",
                mode="markers",
                marker_size=6.0,
            )
        )

    waypoint_columns = {
        "guidance_reference_index",
        "guidance_reference_position_valid",
    }
    if waypoint_columns.issubset(guidance.columns):
        reference_index = guidance["guidance_reference_index"].to_numpy(dtype=int)
        reference_valid = guidance["guidance_reference_position_valid"].to_numpy(dtype=bool)
        previous_valid = np.concatenate(([False], reference_valid[:-1]))
        previous_index = np.concatenate((reference_index[:1], reference_index[:-1]))
        transition_indices = np.flatnonzero(
            reference_valid & (~previous_valid | (reference_index != previous_index))
        )
        for index in transition_indices:
            transition_time_s = float(guidance["time_s"].iloc[index])
            traces.append(
                Plot3DTrace(
                    x=np.array([np.interp(transition_time_s, trajectory_time_s, x)]),
                    y=np.array([np.interp(transition_time_s, trajectory_time_s, y)]),
                    z=np.array([np.interp(transition_time_s, trajectory_time_s, z)]),
                    label=f"Waypoint {reference_index[index]} activated",
                    color="tab:purple",
                    mode="markers",
                    marker_size=7.0,
                )
            )
    return tuple(traces)


def lla_trajectory_3d_plot_spec(run: TrajectoryRunData) -> Plot3DSpec | None:
    """Build an interactive longitude/latitude/height trajectory."""
    frame = run.ned
    if frame is None:
        return None
    traces = (
        Plot3DTrace(
            x=frame["p_lla_lon_deg"].to_numpy(),
            y=frame["p_lla_lat_deg"].to_numpy(),
            z=frame["p_lla_h_m"].to_numpy(),
            label="Truth trajectory",
            color="tab:blue",
        ),
        *_guidance_event_marker_traces(
            run,
            frame["time_s"].to_numpy(),
            frame["p_lla_lon_deg"].to_numpy(),
            frame["p_lla_lat_deg"].to_numpy(),
            frame["p_lla_h_m"].to_numpy(),
        ),
    )
    return Plot3DSpec(
        title="Trajectory - Geodetic Position",
        x_label="Longitude [deg]",
        y_label="Latitude [deg]",
        z_label="Ellipsoidal height [m]",
        output_name="trajectory_position_lla_3d.html",
        traces=traces,
        metadata={"aspectmode": "cube"},
    )


def relative_ned_trajectory_3d_plot_spec(run: TrajectoryRunData) -> Plot3DSpec | None:
    """Build a local tangent-plane trajectory relative to the first sample."""
    ecef = run.ecef
    ned = run.ned
    if ecef is None or ned is None:
        return None

    time_s = ned["time_s"].to_numpy()
    ecef_time_s = ecef["time_s"].to_numpy()
    p_e_m = np.column_stack(
        tuple(
            np.interp(time_s, ecef_time_s, ecef[column].to_numpy())
            for column in ("p_e_x_m", "p_e_y_m", "p_e_z_m")
        )
    )
    latitude_rad = np.deg2rad(ned["p_lla_lat_deg"].iloc[0])
    longitude_rad = np.deg2rad(ned["p_lla_lon_deg"].iloc[0])
    sin_latitude = np.sin(latitude_rad)
    cos_latitude = np.cos(latitude_rad)
    sin_longitude = np.sin(longitude_rad)
    cos_longitude = np.cos(longitude_rad)
    C_e2n = np.array(
        [
            [
                -sin_latitude * cos_longitude,
                -sin_latitude * sin_longitude,
                cos_latitude,
            ],
            [-sin_longitude, cos_longitude, 0.0],
            [
                -cos_latitude * cos_longitude,
                -cos_latitude * sin_longitude,
                -sin_latitude,
            ],
        ]
    )
    relative_ned_m = (p_e_m - p_e_m[0]) @ C_e2n.T
    relative_local_m = relative_ned_m.copy()
    relative_local_m[:, 2] *= -1.0
    traces = (
        Plot3DTrace(
            x=relative_local_m[:, 0],
            y=relative_local_m[:, 1],
            z=relative_local_m[:, 2],
            label="Truth trajectory",
            color="tab:blue",
        ),
        *_guidance_event_marker_traces(
            run,
            time_s,
            relative_local_m[:, 0],
            relative_local_m[:, 1],
            relative_local_m[:, 2],
        ),
    )
    return Plot3DSpec(
        title="Trajectory - Position Relative to Initial Local Frame",
        x_label="North [m]",
        y_label="East [m]",
        z_label="Up [m]",
        output_name="trajectory_position_relative_ned_3d.html",
        traces=traces,
        metadata={
            "aspectmode": "data",
            "reference_time_s": float(time_s[0]),
            "reference_latitude_deg": float(ned["p_lla_lat_deg"].iloc[0]),
            "reference_longitude_deg": float(ned["p_lla_lon_deg"].iloc[0]),
        },
    )


def _response_traces(
    frame: pd.DataFrame,
    signal_columns: Sequence[Sequence[str]],
    signal_labels: Sequence[str],
    signal_styles: Sequence[str],
    *,
    axis_labels: Sequence[str] = ("X", "Y", "Z"),
    scale: float = 1.0,
) -> tuple[PlotTrace, ...]:
    traces: list[PlotTrace] = []
    time_s = frame["time_s"].to_numpy()
    for columns, signal_label, line_style in zip(
        signal_columns, signal_labels, signal_styles
    ):
        for column, axis_label, color in zip(columns, axis_labels, AXIS_COLORS):
            traces.append(
                PlotTrace(
                    x=time_s,
                    y=frame[column].to_numpy() * scale,
                    label=f"{signal_label} {axis_label}",
                    color=color,
                    line_style=line_style,
                )
            )
    return tuple(traces)


def _rpy_response_axis(
    frame: pd.DataFrame,
    command_prefix: str,
    response_prefix: str,
    title: str,
) -> PlotAxis:
    time_s = frame["time_s"].to_numpy()
    command_rpy_deg = _rpy_deg_from_quaternion_wxyz(frame, command_prefix)
    response_rpy_deg = _rpy_deg_from_quaternion_wxyz(frame, response_prefix)
    traces: list[PlotTrace] = []
    for axis_index, (axis_label, color) in enumerate(
        zip(("Roll", "Pitch", "Yaw"), AXIS_COLORS)
    ):
        traces.extend(
            (
                PlotTrace(
                    x=time_s,
                    y=command_rpy_deg[:, axis_index],
                    label=f"Command {axis_label}",
                    color=color,
                    line_style="dash",
                ),
                PlotTrace(
                    x=time_s,
                    y=response_rpy_deg[:, axis_index],
                    label=f"Response {axis_label}",
                    color=color,
                ),
            )
        )
    return PlotAxis(title=title, y_label="Angle [deg]", traces=tuple(traces), zero_line=True)


def guidance_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build focused Guidance acceleration and bank command/response inspection."""
    frame = run.guidance
    if frame is None:
        return None
    time_s = frame["time_s"].to_numpy()
    return PlotSpec(
        title="Guidance Command and Response",
        x_label="Time [s]",
        output_name="trajectory_guidance.html",
        axes=(
            PlotAxis(
                title="Guidance Inertial Acceleration Resolved in NED",
                y_label="Acceleration [g]",
                traces=_response_traces(
                    frame,
                    (
                        (
                            "guidance_acceleration_command_n_n_mps2",
                            "guidance_acceleration_command_n_e_mps2",
                            "guidance_acceleration_command_n_d_mps2",
                        ),
                        (
                            "guidance_acceleration_response_n_n_mps2",
                            "guidance_acceleration_response_n_e_mps2",
                            "guidance_acceleration_response_n_d_mps2",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("North", "East", "Down"),
                    scale=MPS2_TO_G,
                ),
            ),
            PlotAxis(
                title="Guidance Inertial Acceleration Resolved in Body",
                y_label="Acceleration [g]",
                traces=_response_traces(
                    frame,
                    (
                        (
                            "guidance_acceleration_command_b_x_mps2",
                            "guidance_acceleration_command_b_y_mps2",
                            "guidance_acceleration_command_b_z_mps2",
                        ),
                        (
                            "guidance_acceleration_response_b_x_mps2",
                            "guidance_acceleration_response_b_y_mps2",
                            "guidance_acceleration_response_b_z_mps2",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("X (forward)", "Y (right)", "Z (down)"),
                    scale=MPS2_TO_G,
                ),
            ),
            PlotAxis(
                title="Guidance Bank Angle With Respect to NED",
                y_label="Bank [deg]",
                traces=(
                    PlotTrace(
                        x=time_s,
                        y=np.rad2deg(frame["guidance_bank_command_n_rad"].to_numpy()),
                        label="Command roll",
                        color="tab:red",
                        line_style="dash",
                    ),
                    PlotTrace(
                        x=time_s,
                        y=np.rad2deg(frame["guidance_bank_response_n_rad"].to_numpy()),
                        label="Response roll",
                        color="tab:red",
                    ),
                ),
                zero_line=True,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def tracking_error_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build explicit command-minus-realized trajectory tracking errors."""
    frame = run.autopilot_vehicle
    if frame is None:
        return None
    return PlotSpec(
        title="Trajectory Command-Minus-Realized Tracking Error",
        x_label="Time [s]",
        output_name="trajectory_tracking_error.html",
        axes=(
            _vector_axis(
                frame,
                (
                    "velocity_tracking_error_b_x_mps",
                    "velocity_tracking_error_b_y_mps",
                    "velocity_tracking_error_b_z_mps",
                ),
                "Inertial Velocity Tracking Error Resolved in Body",
                "Command - realized [m/s]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
            ),
            _vector_axis(
                frame,
                (
                    "acceleration_tracking_error_b_x_mps2",
                    "acceleration_tracking_error_b_y_mps2",
                    "acceleration_tracking_error_b_z_mps2",
                ),
                "Inertial Acceleration Tracking Error Resolved in Body",
                "Command - realized [g]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
                scale=MPS2_TO_G,
            ),
            _vector_axis(
                frame,
                (
                    "attitude_tracking_error_b_x_rad",
                    "attitude_tracking_error_b_y_rad",
                    "attitude_tracking_error_b_z_rad",
                ),
                "Body-Resolved Attitude Tracking Error",
                "Command - realized [deg]",
                labels=("X rotation", "Y rotation", "Z rotation"),
                scale=180.0 / np.pi,
            ),
            _vector_axis(
                frame,
                (
                    "angular_rate_tracking_error_b_x_radps",
                    "angular_rate_tracking_error_b_y_radps",
                    "angular_rate_tracking_error_b_z_radps",
                ),
                "Body Angular-Rate Tracking Error",
                "Command - realized [deg/s]",
                labels=("P", "Q", "R"),
                scale=180.0 / np.pi,
            ),
            _vector_axis(
                frame,
                (
                    "specific_force_tracking_error_b_x_mps2",
                    "specific_force_tracking_error_b_y_mps2",
                    "specific_force_tracking_error_b_z_mps2",
                ),
                "Body Specific-Force Tracking Error",
                "Command - realized [g]",
                labels=("X (forward)", "Y (right)", "Z (down)"),
                scale=MPS2_TO_G,
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def autopilot_response_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build Autopilot attitude/rate command and response inspection."""
    frame = run.autopilot_vehicle
    if frame is None:
        return None
    return PlotSpec(
        title="Autopilot Command and Response",
        x_label="Time [s]",
        output_name="trajectory_autopilot_response.html",
        axes=(
            _rpy_response_axis(
                frame,
                "autopilot_q_command_b2n",
                "autopilot_q_response_b2n",
                "Body-to-NED Euler Attitude",
            ),
            PlotAxis(
                title="Body Angular Rate With Respect to ECI",
                y_label="Rate [deg/s]",
                traces=_response_traces(
                    frame,
                    (
                        (
                            "autopilot_angular_rate_command_b_x_radps",
                            "autopilot_angular_rate_command_b_y_radps",
                            "autopilot_angular_rate_command_b_z_radps",
                        ),
                        (
                            "autopilot_angular_rate_controller_response_b_x_radps",
                            "autopilot_angular_rate_controller_response_b_y_radps",
                            "autopilot_angular_rate_controller_response_b_z_radps",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("P", "Q", "R"),
                    scale=180.0 / np.pi,
                ),
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def guidance_control_plot_spec(run: TrajectoryRunData) -> PlotSpec | None:
    """Build the consolidated Guidance and Autopilot command/response dashboard."""
    guidance = run.guidance
    autopilot = run.autopilot_vehicle
    if guidance is None or autopilot is None:
        return None
    return PlotSpec(
        title="Trajectory Guidance and Autopilot Command/Response",
        x_label="Time [s]",
        output_name="trajectory_guidance_control.html",
        axes=(
            PlotAxis(
                title="Guidance Inertial Acceleration Resolved in NED",
                y_label="Acceleration [g]",
                traces=_response_traces(
                    guidance,
                    (
                        (
                            "guidance_acceleration_command_n_n_mps2",
                            "guidance_acceleration_command_n_e_mps2",
                            "guidance_acceleration_command_n_d_mps2",
                        ),
                        (
                            "guidance_acceleration_response_n_n_mps2",
                            "guidance_acceleration_response_n_e_mps2",
                            "guidance_acceleration_response_n_d_mps2",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("North", "East", "Down"),
                    scale=MPS2_TO_G,
                ),
            ),
            PlotAxis(
                title="Guidance Inertial Acceleration Resolved in Body",
                y_label="Acceleration [g]",
                traces=_response_traces(
                    guidance,
                    (
                        (
                            "guidance_acceleration_command_b_x_mps2",
                            "guidance_acceleration_command_b_y_mps2",
                            "guidance_acceleration_command_b_z_mps2",
                        ),
                        (
                            "guidance_acceleration_response_b_x_mps2",
                            "guidance_acceleration_response_b_y_mps2",
                            "guidance_acceleration_response_b_z_mps2",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("X (forward)", "Y (right)", "Z (down)"),
                    scale=MPS2_TO_G,
                ),
            ),
            PlotAxis(
                title="Guidance Body Specific-Force Command Filter",
                y_label="Specific force [g]",
                traces=_response_traces(
                    guidance,
                    (
                        (
                            "guidance_specific_force_command_b_x_mps2",
                            "guidance_specific_force_command_b_y_mps2",
                            "guidance_specific_force_command_b_z_mps2",
                        ),
                        (
                            "guidance_specific_force_filtered_b_x_mps2",
                            "guidance_specific_force_filtered_b_y_mps2",
                            "guidance_specific_force_filtered_b_z_mps2",
                        ),
                    ),
                    ("Command", "Filtered"),
                    ("dash", "solid"),
                    axis_labels=("X (forward)", "Y (right)", "Z (down)"),
                    scale=MPS2_TO_G,
                ),
            ),
            _rpy_response_axis(
                autopilot,
                "autopilot_q_command_b2n",
                "autopilot_q_response_b2n",
                "Autopilot Body-to-NED Euler Attitude",
            ),
            PlotAxis(
                title="Autopilot Body-Rate Command and Response",
                y_label="Rate [deg/s]",
                traces=_response_traces(
                    autopilot,
                    (
                        (
                            "autopilot_angular_rate_command_b_x_radps",
                            "autopilot_angular_rate_command_b_y_radps",
                            "autopilot_angular_rate_command_b_z_radps",
                        ),
                        (
                            "autopilot_angular_rate_controller_response_b_x_radps",
                            "autopilot_angular_rate_controller_response_b_y_radps",
                            "autopilot_angular_rate_controller_response_b_z_radps",
                        ),
                    ),
                    ("Command", "Response"),
                    ("dash", "solid"),
                    axis_labels=("P", "Q", "R"),
                    scale=180.0 / np.pi,
                ),
            ),
        ),
        metadata={"legend_scope": "per_axis"},
    )


def trajectory_plot_specs(run: TrajectoryRunData) -> dict[str, PlotSpec | Plot3DSpec]:
    """Return every available trajectory plot specification by stable name."""
    candidates = {
        "kinematics_ecef": ecef_trajectory_plot_spec(run),
        "kinematics_eci": eci_trajectory_plot_spec(run),
        "kinematics_ned": ned_trajectory_plot_spec(run),
        "kinematics_body": body_trajectory_plot_spec(run),
        "position_lla_3d": lla_trajectory_3d_plot_spec(run),
        "position_relative_ned_3d": relative_ned_trajectory_3d_plot_spec(run),
        "guidance": guidance_plot_spec(run),
        "autopilot_response": autopilot_response_plot_spec(run),
        "guidance_control": guidance_control_plot_spec(run),
        "tracking_error": tracking_error_plot_spec(run),
    }
    return {name: spec for name, spec in candidates.items() if spec is not None}
