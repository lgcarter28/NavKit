# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import unittest

import numpy as np
import pandas as pd

from navkit_analysis.consistency import nees_from_frame
from navkit_analysis.data import derive_truth_error_frame


class TruthErrorConventionTests(unittest.TestCase):
    def test_all_state_families_follow_truth_minus_estimate(self) -> None:
        nav = pd.DataFrame(
            {
                "time_s": [0.0],
                "p_e_x_m": [1.0],
                "p_e_y_m": [2.0],
                "p_e_z_m": [3.0],
                "v_e_x_mps": [4.0],
                "v_e_y_mps": [5.0],
                "v_e_z_mps": [6.0],
                "q_b2e_w": [1.0],
                "q_b2e_x": [0.0],
                "q_b2e_y": [0.0],
                "q_b2e_z": [0.0],
                "gyro_bias_b_x_radps": [0.1],
                "gyro_bias_b_y_radps": [0.2],
                "gyro_bias_b_z_radps": [0.3],
                "accel_bias_b_x_mps2": [0.4],
                "accel_bias_b_y_mps2": [0.5],
                "accel_bias_b_z_mps2": [0.6],
            }
        )
        truth = pd.DataFrame(
            {
                "time_s": [0.0],
                "p_e_x_m": [2.0],
                "p_e_y_m": [4.0],
                "p_e_z_m": [6.0],
                "v_e_x_mps": [8.0],
                "v_e_y_mps": [10.0],
                "v_e_z_mps": [12.0],
                "q_b2e_w": [np.cos(0.05)],
                "q_b2e_x": [np.sin(0.05)],
                "q_b2e_y": [0.0],
                "q_b2e_z": [0.0],
            }
        )
        imu = pd.DataFrame(
            {
                "time_s": [0.0],
                "truth_gyro_bias_b_x_radps": [0.2],
                "truth_gyro_bias_b_y_radps": [0.4],
                "truth_gyro_bias_b_z_radps": [0.6],
                "truth_accel_bias_b_x_mps2": [0.8],
                "truth_accel_bias_b_y_mps2": [1.0],
                "truth_accel_bias_b_z_mps2": [1.2],
            }
        )

        errors = derive_truth_error_frame(nav, truth, imu)

        self.assertIsNotNone(errors)
        assert errors is not None
        np.testing.assert_allclose(
            errors[["error_p_e_x_m", "error_p_e_y_m", "error_p_e_z_m"]].to_numpy(),
            [[1.0, 2.0, 3.0]],
        )
        np.testing.assert_allclose(
            errors[
                ["error_v_e_x_mps", "error_v_e_y_mps", "error_v_e_z_mps"]
            ].to_numpy(),
            [[4.0, 5.0, 6.0]],
        )
        np.testing.assert_allclose(
            errors[
                [
                    "error_theta_b2e_x_rad",
                    "error_theta_b2e_y_rad",
                    "error_theta_b2e_z_rad",
                ]
            ].to_numpy(),
            [[0.1, 0.0, 0.0]],
            atol=1.0e-14,
        )
        np.testing.assert_allclose(
            errors[
                [
                    "error_gyro_bias_b_x_radps",
                    "error_gyro_bias_b_y_radps",
                    "error_gyro_bias_b_z_radps",
                ]
            ].to_numpy(),
            [[0.1, 0.2, 0.3]],
        )
        np.testing.assert_allclose(
            errors[
                [
                    "error_accel_bias_b_x_mps2",
                    "error_accel_bias_b_y_mps2",
                    "error_accel_bias_b_z_mps2",
                ]
            ].to_numpy(),
            [[0.4, 0.5, 0.6]],
        )

    def test_joint_nees_retains_truth_minus_estimate_sign_across_cross_covariance(
        self,
    ) -> None:
        labels = ("p_e_x_m", "theta_b2e_x_rad")
        covariance = np.array([[4.0, 1.5], [1.5, 1.0]])
        errors = np.array([2.0, 0.5])
        frame = pd.DataFrame(
            {
                "error_p_e_x_m": [errors[0]],
                "error_theta_b2e_x_rad": [errors[1]],
                "P_p_e_x_m__p_e_x_m": [covariance[0, 0]],
                "P_p_e_x_m__theta_b2e_x_rad": [covariance[0, 1]],
                "P_theta_b2e_x_rad__theta_b2e_x_rad": [covariance[1, 1]],
            }
        )

        nees = nees_from_frame(frame, labels)

        self.assertIsNotNone(nees)
        assert nees is not None
        expected = errors @ np.linalg.solve(covariance, errors)
        wrong_attitude_sign = np.array([errors[0], -errors[1]])
        wrong = wrong_attitude_sign @ np.linalg.solve(covariance, wrong_attitude_sign)
        np.testing.assert_allclose(nees, [expected])
        self.assertNotAlmostEqual(float(nees[0]), float(wrong))


if __name__ == "__main__":
    unittest.main()
