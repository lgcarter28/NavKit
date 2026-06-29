# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pandas as pd


@dataclass(frozen=True)
class RunData:
    """Container for NavKit run CSV data.

    Optional logs are represented as None so plotting functions can decide
    whether to skip a figure cleanly.
    """

    run_dir: Path
    nav: pd.DataFrame
    gnss_pos_update: pd.DataFrame | None = None


def _read_optional_csv(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        return None
    return pd.read_csv(path)


def _require_columns(frame: pd.DataFrame, columns: list[str], source: Path) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise KeyError(f"{source} is missing required columns: {missing}")


def load_run(run_dir: Path) -> RunData:
    """Load standard NavKit run logs from a run directory."""
    run_dir = run_dir.resolve()

    nav_path = run_dir / "nav.csv"
    nav = pd.read_csv(nav_path)

    _require_columns(
        nav,
        [
            "time_s",
            "err_p_e_x_m",
            "err_p_e_y_m",
            "err_p_e_z_m",
            "sigma_p_e_x_m",
            "sigma_p_e_y_m",
            "sigma_p_e_z_m",
        ],
        nav_path,
    )

    gnss_pos_update_path = run_dir / "gnss_pos_update.csv"
    gnss_pos_update = _read_optional_csv(gnss_pos_update_path)

    if gnss_pos_update is not None:
        _require_columns(
            gnss_pos_update,
            [
                "time_s",
                "nu_p_e_x_m",
                "nu_p_e_y_m",
                "nu_p_e_z_m",
                "sigma_nu_p_e_x_m",
                "sigma_nu_p_e_y_m",
                "sigma_nu_p_e_z_m",
                "nis",
            ],
            gnss_pos_update_path,
        )

    return RunData(run_dir=run_dir, nav=nav, gnss_pos_update=gnss_pos_update)
