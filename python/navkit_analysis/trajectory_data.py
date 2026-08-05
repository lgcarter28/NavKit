# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Optional CSV-backed trajectory inspection data for one simulation run."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pandas as pd

from navkit_analysis.data import read_navkit_csv, resolve_run_dirs


@dataclass(frozen=True)
class TrajectoryRunData:
    """Frame-explicit trajectory logs kept separate from estimator run data."""

    run_dir: Path
    data_dir: Path
    figures_dir: Path
    ecef: pd.DataFrame | None = None
    eci: pd.DataFrame | None = None
    ned: pd.DataFrame | None = None
    body: pd.DataFrame | None = None
    guidance: pd.DataFrame | None = None
    autopilot_vehicle: pd.DataFrame | None = None

    @property
    def available(self) -> bool:
        """Return whether the run contains any trajectory inspection product."""
        return any(
            frame is not None
            for frame in (
                self.ecef,
                self.eci,
                self.ned,
                self.body,
                self.guidance,
                self.autopilot_vehicle,
            )
        )


def _optional_frame(data_dir: Path, name: str) -> pd.DataFrame | None:
    path = data_dir / name
    if not path.exists():
        return None
    frame = read_navkit_csv(path)
    return None if frame.empty else frame


def _trim_time(
    frame: pd.DataFrame | None,
    start_time_s: float | None,
    end_time_s: float | None,
) -> pd.DataFrame | None:
    if frame is None:
        return None
    selected = frame
    if start_time_s is not None:
        selected = selected.loc[selected["time_s"] >= start_time_s]
    if end_time_s is not None:
        selected = selected.loc[selected["time_s"] <= end_time_s]
    return selected.reset_index(drop=True)


def load_trajectory_run(
    source: Path,
    *,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
) -> TrajectoryRunData:
    """Load any available frame-specific trajectory inspection CSVs."""
    run_dir, data_dir, figures_dir = resolve_run_dirs(source)
    return TrajectoryRunData(
        run_dir=run_dir,
        data_dir=data_dir,
        figures_dir=figures_dir,
        ecef=_trim_time(
            _optional_frame(data_dir, "trajectory_kinematics_ecef.csv"),
            start_time_s,
            end_time_s,
        ),
        eci=_trim_time(
            _optional_frame(data_dir, "trajectory_kinematics_eci.csv"),
            start_time_s,
            end_time_s,
        ),
        ned=_trim_time(
            _optional_frame(data_dir, "trajectory_kinematics_ned.csv"),
            start_time_s,
            end_time_s,
        ),
        body=_trim_time(
            _optional_frame(data_dir, "trajectory_kinematics_body.csv"),
            start_time_s,
            end_time_s,
        ),
        guidance=_trim_time(
            _optional_frame(data_dir, "trajectory_guidance.csv"),
            start_time_s,
            end_time_s,
        ),
        autopilot_vehicle=_trim_time(
            _optional_frame(data_dir, "trajectory_autopilot_vehicle.csv"),
            start_time_s,
            end_time_s,
        ),
    )
