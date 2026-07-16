# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Public figure-generation API for NavKit analysis."""

from __future__ import annotations

from navkit_analysis.figures.gnss_position_histograms import (
    plot_gnss_position_histograms,
)
from navkit_analysis.figures.gnss_position_innovation import (
    plot_gnss_position_innovation,
)
from navkit_analysis.figures.gnss_position_nis import plot_gnss_position_nis
from navkit_analysis.figures.imu_increments import (
    plot_imu_debug_terms,
    plot_imu_increment_cumsums,
    plot_imu_increment_time_histories,
)
from navkit_analysis.figures.position_error import plot_position_error_covariance
from navkit_analysis.figures.state_errors import (
    plot_filter_corrections,
    plot_imu_bias_truth_errors,
    plot_truth_errors,
    plot_truth_errors_ecef,
    plot_truth_errors_ned,
    plot_truth_errors_ned_dashboard,
)

__all__ = [
    "plot_filter_corrections",
    "plot_gnss_position_histograms",
    "plot_gnss_position_innovation",
    "plot_gnss_position_nis",
    "plot_imu_debug_terms",
    "plot_imu_bias_truth_errors",
    "plot_imu_increment_cumsums",
    "plot_imu_increment_time_histories",
    "plot_position_error_covariance",
    "plot_truth_errors",
    "plot_truth_errors_ecef",
    "plot_truth_errors_ned",
    "plot_truth_errors_ned_dashboard",
]
