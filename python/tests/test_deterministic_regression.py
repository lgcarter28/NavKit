# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from navkit_analysis.regression import (
    TRUTH_RECONSTRUCTION_METRIC_UNITS,
    DeterministicRegressionCase,
    UpdateCountContract,
    evaluate_truth_reconstruction,
    load_deterministic_regression_suite,
    load_truth_reconstruction_metrics,
    truth_reconstruction_metrics,
)
from navkit_analysis.schema import DETERMINISTIC_REGRESSION_SUITE_SCHEMA
from navkit_analysis.data import RunData


def _quaternion_wxyz(rotation: Rotation) -> np.ndarray:
    quaternion_xyzw = rotation.as_quat()
    return quaternion_xyzw[..., [3, 0, 1, 2]]


def _state_frame(
    time_s: np.ndarray,
    position_e_m: np.ndarray,
    velocity_e_mps: np.ndarray,
    quaternion_b2e_wxyz: np.ndarray,
) -> pd.DataFrame:
    return pd.DataFrame(
        {
            "time_s": time_s,
            "p_e_x_m": position_e_m[:, 0],
            "p_e_y_m": position_e_m[:, 1],
            "p_e_z_m": position_e_m[:, 2],
            "v_e_x_mps": velocity_e_mps[:, 0],
            "v_e_y_mps": velocity_e_mps[:, 1],
            "v_e_z_mps": velocity_e_mps[:, 2],
            "q_b2e_w": quaternion_b2e_wxyz[:, 0],
            "q_b2e_x": quaternion_b2e_wxyz[:, 1],
            "q_b2e_y": quaternion_b2e_wxyz[:, 2],
            "q_b2e_z": quaternion_b2e_wxyz[:, 3],
        }
    )


def _run_data(nav: pd.DataFrame, truth: pd.DataFrame) -> RunData:
    placeholder = Path("synthetic")
    return RunData(
        run_dir=placeholder,
        data_dir=placeholder,
        figures_dir=placeholder,
        nav=nav,
        truth=truth,
    )


class DeterministicRegressionMetricTests(unittest.TestCase):
    def test_interpolates_position_and_velocity_at_mismatched_cadence(self) -> None:
        truth_time_s = np.array([0.0, 1.0, 2.0])
        nav_time_s = np.array([0.25, 0.75, 1.25, 1.75])
        truth_position = np.column_stack(
            (2.0 * truth_time_s, -3.0 * truth_time_s, 4.0 * truth_time_s)
        )
        truth_velocity = np.column_stack(
            (
                1.0 + truth_time_s,
                2.0 - (0.5 * truth_time_s),
                -2.0 * truth_time_s,
            )
        )
        nav_position = np.column_stack(
            (2.0 * nav_time_s, -3.0 * nav_time_s, 4.0 * nav_time_s)
        )
        nav_velocity = np.column_stack(
            (
                1.0 + nav_time_s,
                2.0 - (0.5 * nav_time_s),
                -2.0 * nav_time_s,
            )
        )
        truth_quaternion = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
        nav_quaternion = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (4, 1))

        metrics = truth_reconstruction_metrics(
            _run_data(
                _state_frame(nav_time_s, nav_position, nav_velocity, nav_quaternion),
                _state_frame(
                    truth_time_s,
                    truth_position,
                    truth_velocity,
                    truth_quaternion,
                ),
            )
        )

        self.assertEqual(metrics["sample_count"], 4)
        self.assertAlmostEqual(float(metrics["duration_s"]), 1.5)
        for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS:
            self.assertAlmostEqual(float(metrics[metric_name]), 0.0, places=14)

    def test_attitude_metric_is_sign_invariant_and_reports_relative_rotation(self) -> None:
        truth_time_s = np.array([0.0, 1.0])
        nav_time_s = np.array([0.5])
        truth_quaternion = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
            ]
        )
        relative_angle_rad = 0.125
        nav_attitude = Rotation.from_rotvec(np.array([-relative_angle_rad, 0.0, 0.0]))
        nav_quaternion = _quaternion_wxyz(nav_attitude).reshape(1, 4)
        zero_truth = np.zeros((2, 3))
        zero_nav = np.zeros((1, 3))

        metrics = truth_reconstruction_metrics(
            _run_data(
                _state_frame(nav_time_s, zero_nav, zero_nav, nav_quaternion),
                _state_frame(
                    truth_time_s,
                    zero_truth,
                    zero_truth,
                    truth_quaternion,
                ),
            )
        )

        self.assertAlmostEqual(
            float(metrics["attitude_max_norm_rad"]), relative_angle_rad, places=14
        )
        self.assertAlmostEqual(
            float(metrics["attitude_rms_norm_rad"]), relative_angle_rad, places=14
        )

    def test_rejects_navigation_epochs_outside_truth_coverage(self) -> None:
        truth_time_s = np.array([0.0, 1.0])
        zero_truth = np.zeros((2, 3))
        truth_quaternion = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1))
        truth = _state_frame(
            truth_time_s, zero_truth, zero_truth, truth_quaternion
        )

        for nav_time_s in (np.array([-0.1, 0.5]), np.array([0.5, 1.1])):
            with self.subTest(nav_time_s=nav_time_s):
                zero_nav = np.zeros((2, 3))
                nav_quaternion = np.tile(
                    np.array([1.0, 0.0, 0.0, 0.0]), (2, 1)
                )
                nav = _state_frame(
                    nav_time_s, zero_nav, zero_nav, nav_quaternion
                )
                with self.assertRaisesRegex(ValueError, "truth does not bracket"):
                    truth_reconstruction_metrics(_run_data(nav, truth))

    def test_rejects_nonfinite_timestamps_and_errors(self) -> None:
        valid_time_s = np.array([0.0, 1.0])
        zeros = np.zeros((2, 3))
        identity = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1))
        valid_truth = _state_frame(valid_time_s, zeros, zeros, identity)

        bad_time_nav = _state_frame(
            np.array([0.0, np.nan]), zeros, zeros, identity
        )
        with self.assertRaisesRegex(ValueError, "timestamps must be finite"):
            truth_reconstruction_metrics(_run_data(bad_time_nav, valid_truth))

        nonfinite_position = zeros.copy()
        nonfinite_position[1, 0] = np.inf
        bad_state_nav = _state_frame(
            valid_time_s, nonfinite_position, zeros, identity
        )
        with self.assertRaisesRegex(ValueError, "errors must be finite"):
            truth_reconstruction_metrics(_run_data(bad_state_nav, valid_truth))


class DeterministicRegressionContractTests(unittest.TestCase):
    def test_counts_distinct_accepted_sensor_update_timestamps(self) -> None:
        time_s = np.array([0.0, 1.0])
        zeros = np.zeros((2, 3))
        identity = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1))
        state = _state_frame(time_s, zeros, zeros, identity)

        with tempfile.TemporaryDirectory(prefix="navkit_regression_run_") as temp_dir:
            run_dir = Path(temp_dir)
            data_dir = run_dir / "data"
            data_dir.mkdir()
            state.to_csv(data_dir / "nav_estimate_ecef.csv", index=False)
            state.to_csv(data_dir / "truth_trajectory_ecef.csv", index=False)
            pd.DataFrame(
                {
                    "time_s": [0.0, 0.0, 1.0, 2.0],
                    "accepted": [1.0, 1.0, 0.0, 1.0],
                }
            ).to_csv(
                data_dir / "gnss_pos_update.csv", index=False
            )
            pd.DataFrame(columns=["time_s", "accepted"]).to_csv(
                data_dir / "gnss_vel_update.csv", index=False
            )

            metrics = load_truth_reconstruction_metrics(run_dir)

        self.assertEqual(metrics["gnss_position_update_count"], 2)
        self.assertEqual(metrics["gnss_velocity_update_count"], 0)

    def test_requires_sensor_update_evidence_logs(self) -> None:
        time_s = np.array([0.0, 1.0])
        zeros = np.zeros((2, 3))
        identity = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1))
        state = _state_frame(time_s, zeros, zeros, identity)

        with tempfile.TemporaryDirectory(prefix="navkit_regression_run_") as temp_dir:
            run_dir = Path(temp_dir)
            data_dir = run_dir / "data"
            data_dir.mkdir()
            state.to_csv(data_dir / "nav_estimate_ecef.csv", index=False)
            state.to_csv(data_dir / "truth_trajectory_ecef.csv", index=False)
            with self.assertRaisesRegex(FileNotFoundError, "sensor-update log"):
                load_truth_reconstruction_metrics(run_dir)

    def test_evaluates_metric_duration_and_sample_thresholds(self) -> None:
        thresholds = {
            metric_name: 1.0 for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS
        }
        case = DeterministicRegressionCase(
            name="synthetic",
            scenario=Path("synthetic.json"),
            minimum_duration_s=10.0,
            minimum_sample_count=100,
            thresholds=thresholds,
            sensor_update_counts={
                "gnss_position": UpdateCountContract(1, None),
                "gnss_velocity": UpdateCountContract(1, 2),
            },
        )
        passing_metrics: dict[str, float | int] = {
            metric_name: 0.5 for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS
        }
        passing_metrics.update({"duration_s": 10.0, "sample_count": 100})
        passing_metrics.update(
            {"gnss_position_update_count": 1, "gnss_velocity_update_count": 2}
        )

        passed, checks = evaluate_truth_reconstruction(passing_metrics, case)

        self.assertTrue(passed)
        self.assertTrue(all(bool(check["passed"]) for check in checks.values()))

        failing_metrics = dict(passing_metrics)
        failing_metrics["velocity_max_norm_mps"] = 1.5
        failing_metrics["duration_s"] = 9.0
        failing_metrics["sample_count"] = 99
        failing_metrics["gnss_position_update_count"] = 0
        failing_metrics["gnss_velocity_update_count"] = 3

        passed, checks = evaluate_truth_reconstruction(failing_metrics, case)

        self.assertFalse(passed)
        self.assertFalse(bool(checks["velocity_max_norm_mps"]["passed"]))
        self.assertFalse(bool(checks["minimum_duration_s"]["passed"]))
        self.assertFalse(bool(checks["minimum_sample_count"]["passed"]))
        self.assertFalse(bool(checks["gnss_position_update_count"]["passed"]))
        self.assertFalse(bool(checks["gnss_velocity_update_count"]["passed"]))

    def test_loads_valid_suite_and_rejects_incompatible_schema(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_regression_test_") as temp_dir:
            root = Path(temp_dir)
            scenario = root / "scenario.json"
            scenario.write_text("{}\n", encoding="utf-8")
            suite_path = root / "suite.json"
            document = {
                "schema": DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
                "suite_name": "synthetic_suite",
                "execution": {
                    "build_type": "Release",
                    "navkit_config": "apps/navkit_sim/Synthetic.hpp",
                },
                "output": {"root": "output/regressions/synthetic"},
                "cases": [
                    {
                        "name": "synthetic_case",
                        "scenario": "scenario.json",
                        "minimum_duration_s": 1.0,
                        "minimum_sample_count": 2,
                        "thresholds": {
                            metric_name: 1.0
                            for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS
                        },
                        "sensor_update_counts": {
                            "gnss_position": {"minimum": 0, "maximum": 0},
                            "gnss_velocity": {"minimum": 1},
                        },
                    }
                ],
            }
            suite_path.write_text(json.dumps(document), encoding="utf-8")

            suite = load_deterministic_regression_suite(suite_path)

            self.assertEqual(suite.name, "synthetic_suite")
            self.assertEqual(suite.cases[0].scenario, scenario.resolve())

            document["schema"] = "navkit.deterministic_regression_suite.v999"
            suite_path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "incompatible schema"):
                load_deterministic_regression_suite(suite_path)


if __name__ == "__main__":
    unittest.main()
