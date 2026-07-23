# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Shared CSV/HDF5 source selection for NavKit plotting entry points."""

from __future__ import annotations

from pathlib import Path

from navkit_analysis.bundle import is_analysis_bundle, load_run_from_bundle
from navkit_analysis.data import RunData, load_run, trim_run_data


def load_analysis_run(
    source: Path,
    *,
    run_id: str | None = None,
    start_time_s: float | None = None,
    end_time_s: float | None = None,
) -> RunData:
    """Load CSV-folder or HDF5-bundle data through one public analysis seam."""
    if is_analysis_bundle(source):
        run = load_run_from_bundle(source, run_id)
        return trim_run_data(run, start_time_s, end_time_s)
    return load_run(source, start_time_s=start_time_s, end_time_s=end_time_s)
