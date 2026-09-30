# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import unittest

import numpy as np

from navkit_analysis.covariance_diagnostics import (
    full_nees,
    marginal_block_nees,
    two_block_schur_nees,
    validate_error_covariance,
)


class CovarianceDiagnosticTests(unittest.TestCase):
    def test_batched_schur_terms_sum_to_full_nees(self) -> None:
        errors = np.array(
            [
                [0.5, -0.25, 1.0, -0.5],
                [-1.0, 0.75, 0.25, 0.5],
            ]
        )
        covariances = np.array(
            [
                [
                    [2.0, 0.2, 0.4, -0.1],
                    [0.2, 1.5, 0.1, 0.3],
                    [0.4, 0.1, 1.2, 0.15],
                    [-0.1, 0.3, 0.15, 1.8],
                ],
                [
                    [1.4, -0.1, 0.2, 0.05],
                    [-0.1, 1.1, -0.15, 0.25],
                    [0.2, -0.15, 1.6, 0.2],
                    [0.05, 0.25, 0.2, 1.3],
                ],
            ]
        )

        decomposition = two_block_schur_nees(
            errors, covariances, (0, 1), (2, 3)
        )

        np.testing.assert_allclose(
            decomposition.decomposed_nees,
            decomposition.full_nees,
            rtol=1.0e-13,
            atol=1.0e-13,
        )
        np.testing.assert_allclose(
            decomposition.full_nees,
            full_nees(errors, covariances),
            rtol=0.0,
            atol=0.0,
        )
        np.testing.assert_allclose(
            decomposition.first_marginal_nees,
            marginal_block_nees(errors, covariances, (0, 1)),
            rtol=1.0e-13,
            atol=1.0e-13,
        )

    def test_cross_covariance_exposes_joint_failure_with_healthy_marginals(self) -> None:
        errors = np.array([1.0, 1.0])
        covariance = np.array([[1.0, -0.99], [-0.99, 1.0]])

        first_marginal = marginal_block_nees(errors, covariance, (0,))
        second_marginal = marginal_block_nees(errors, covariance, (1,))
        decomposition = two_block_schur_nees(errors, covariance, (0,), (1,))

        self.assertAlmostEqual(float(first_marginal), 1.0)
        self.assertAlmostEqual(float(second_marginal), 1.0)
        self.assertGreater(float(decomposition.second_conditional_nees), 100.0)
        self.assertGreater(float(decomposition.full_nees), 100.0)
        self.assertAlmostEqual(
            float(decomposition.decomposed_nees),
            float(decomposition.full_nees),
            places=12,
        )

    def test_broadcasts_covariance_across_error_batches(self) -> None:
        errors = np.array([[1.0, 0.0], [0.0, 2.0]])
        covariance = np.diag([2.0, 4.0])

        values = full_nees(errors, covariance)

        np.testing.assert_allclose(values, np.array([0.5, 1.0]))

    def test_qualification_partition_normalizes_by_each_block_dof(self) -> None:
        errors = np.ones((2, 3, 15))
        covariance = np.eye(15)

        decomposition = two_block_schur_nees(
            errors,
            covariance,
            tuple(range(9)),
            tuple(range(9, 15)),
        )

        self.assertEqual(decomposition.full_nees.shape, (2, 3))
        np.testing.assert_allclose(decomposition.full_nees / 15.0, 1.0)
        np.testing.assert_allclose(
            decomposition.first_marginal_nees / 9.0, 1.0
        )
        np.testing.assert_allclose(
            decomposition.second_marginal_nees / 6.0, 1.0
        )
        np.testing.assert_allclose(
            decomposition.second_conditional_nees / 6.0, 1.0
        )

    def test_rejects_invalid_covariances_and_partitions(self) -> None:
        errors = np.array([1.0, 2.0])
        with self.assertRaisesRegex(ValueError, "symmetric"):
            validate_error_covariance(errors, np.array([[1.0, 1.0], [0.0, 1.0]]))
        with self.assertRaisesRegex(ValueError, "positive definite"):
            validate_error_covariance(errors, np.array([[1.0, 2.0], [2.0, 1.0]]))
        with self.assertRaisesRegex(ValueError, "cover the complete state"):
            two_block_schur_nees(
                np.array([1.0, 2.0, 3.0]),
                np.eye(3),
                (0,),
                (1,),
            )
        with self.assertRaisesRegex(ValueError, "must be integers"):
            marginal_block_nees(errors, np.eye(2), (0.5,))
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            full_nees(np.empty((0, 2)), np.eye(2))


if __name__ == "__main__":
    unittest.main()
