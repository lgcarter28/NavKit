# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Cross-covariance-aware diagnostics for partitioned estimator errors."""

from __future__ import annotations

import operator
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class TwoBlockNeesDecomposition:
    """Exact NEES decomposition induced by an ordered two-block partition.

    ``first_marginal_nees`` is the ordinary marginal statistic for the first
    block. ``second_conditional_nees`` is evaluated with the conditional error
    and Schur-complement covariance of the second block given the first. Their
    sum is the full-state NEES, including every cross-covariance term.
    """

    first_marginal_nees: np.ndarray
    second_marginal_nees: np.ndarray
    second_conditional_nees: np.ndarray
    full_nees: np.ndarray
    conditional_error: np.ndarray
    conditional_covariance: np.ndarray

    @property
    def decomposed_nees(self) -> np.ndarray:
        """Return the sum of the first marginal and second conditional terms."""
        return self.first_marginal_nees + self.second_conditional_nees


def validate_error_covariance(
    errors: np.ndarray,
    covariances: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Validate and broadcast error vectors with symmetric positive-definite covariance.

    Errors have shape ``(..., N)`` and covariances have shape
    ``(..., N, N)``. Their leading dimensions may differ when NumPy can
    broadcast them to a common batch shape. Scalar use is represented by
    shapes ``(N,)`` and ``(N, N)``.
    """
    error_values = np.asarray(errors, dtype=float)
    covariance_values = np.asarray(covariances, dtype=float)
    if error_values.ndim < 1:
        raise ValueError("errors must have shape (..., N)")
    if covariance_values.ndim < 2:
        raise ValueError("covariances must have shape (..., N, N)")
    state_dimension = error_values.shape[-1]
    if state_dimension == 0:
        raise ValueError("errors and covariances must have a nonzero state dimension")
    if covariance_values.shape[-2:] != (state_dimension, state_dimension):
        raise ValueError(
            "covariance trailing dimensions must match the error state dimension"
        )
    try:
        batch_shape = np.broadcast_shapes(
            error_values.shape[:-1], covariance_values.shape[:-2]
        )
    except ValueError as error:
        raise ValueError(
            "error and covariance batch dimensions are not broadcast-compatible"
        ) from error
    if any(dimension == 0 for dimension in batch_shape):
        raise ValueError("error and covariance batches must not be empty")
    broadcast_errors = np.broadcast_to(
        error_values, batch_shape + (state_dimension,)
    )
    broadcast_covariances = np.broadcast_to(
        covariance_values, batch_shape + (state_dimension, state_dimension)
    )
    if not np.isfinite(broadcast_errors).all():
        raise ValueError("errors must contain only finite values")
    if not np.isfinite(broadcast_covariances).all():
        raise ValueError("covariances must contain only finite values")
    if not np.allclose(
        broadcast_covariances,
        np.swapaxes(broadcast_covariances, -1, -2),
        rtol=1.0e-10,
        atol=1.0e-12,
    ):
        raise ValueError("covariances must be symmetric")
    try:
        np.linalg.cholesky(broadcast_covariances)
    except np.linalg.LinAlgError as error:
        raise ValueError("covariances must be positive definite") from error
    return broadcast_errors, broadcast_covariances


def full_nees(errors: np.ndarray, covariances: np.ndarray) -> np.ndarray:
    """Return full-state NEES for scalar or batched error/covariance arrays."""
    error_values, covariance_values = validate_error_covariance(errors, covariances)
    return _quadratic_form(error_values, covariance_values)


def marginal_block_nees(
    errors: np.ndarray,
    covariances: np.ndarray,
    indices: Sequence[int],
) -> np.ndarray:
    """Return marginal NEES for one selected covariance principal block."""
    error_values, covariance_values = validate_error_covariance(errors, covariances)
    block_indices = _validate_indices(indices, error_values.shape[-1], "block")
    block_errors = np.take(error_values, block_indices, axis=-1)
    block_covariances = _principal_block(covariance_values, block_indices)
    return _quadratic_form(block_errors, block_covariances)


def two_block_schur_nees(
    errors: np.ndarray,
    covariances: np.ndarray,
    first_indices: Sequence[int],
    second_indices: Sequence[int],
) -> TwoBlockNeesDecomposition:
    """Decompose full NEES using an ordered two-block Schur complement.

    For covariance blocks ``A``, ``B``, and ``C`` and errors ``a`` and ``b``,
    the returned terms implement

    ``NEES = a.T A^-1 a + r.T S^-1 r``,

    where ``r = b - C.T A^-1 a`` and ``S = B - C.T A^-1 C``. The two index
    sequences must be nonempty, disjoint, and together cover the complete
    state. Swap their order to diagnose the opposite conditional direction.
    """
    error_values, covariance_values = validate_error_covariance(errors, covariances)
    state_dimension = error_values.shape[-1]
    first = _validate_indices(first_indices, state_dimension, "first block")
    second = _validate_indices(second_indices, state_dimension, "second block")
    if set(first).intersection(second):
        raise ValueError("two-block partition indices must be disjoint")
    if set(first).union(second) != set(range(state_dimension)):
        raise ValueError("two-block partition indices must cover the complete state")

    first_errors = np.take(error_values, first, axis=-1)
    second_errors = np.take(error_values, second, axis=-1)
    first_covariance = _principal_block(covariance_values, first)
    second_covariance = _principal_block(covariance_values, second)
    first_rows = np.take(covariance_values, first, axis=-2)
    cross_covariance = np.take(first_rows, second, axis=-1)

    first_error_solution = np.linalg.solve(
        first_covariance, first_errors[..., np.newaxis]
    )[..., 0]
    first_cross_solution = np.linalg.solve(first_covariance, cross_covariance)
    cross_transpose = np.swapaxes(cross_covariance, -1, -2)
    conditional_error = second_errors - np.matmul(
        cross_transpose, first_error_solution[..., np.newaxis]
    )[..., 0]
    conditional_covariance = second_covariance - np.matmul(
        cross_transpose, first_cross_solution
    )
    conditional_covariance = 0.5 * (
        conditional_covariance + np.swapaxes(conditional_covariance, -1, -2)
    )

    first_marginal = np.einsum(
        "...i,...i->...", first_errors, first_error_solution
    )
    second_marginal = _quadratic_form(second_errors, second_covariance)
    second_conditional = _quadratic_form(
        conditional_error, conditional_covariance
    )
    complete_nees = _quadratic_form(error_values, covariance_values)
    return TwoBlockNeesDecomposition(
        first_marginal_nees=first_marginal,
        second_marginal_nees=second_marginal,
        second_conditional_nees=second_conditional,
        full_nees=complete_nees,
        conditional_error=conditional_error,
        conditional_covariance=conditional_covariance,
    )


def _quadratic_form(errors: np.ndarray, covariances: np.ndarray) -> np.ndarray:
    solutions = np.linalg.solve(covariances, errors[..., np.newaxis])[..., 0]
    return np.einsum("...i,...i->...", errors, solutions)


def _principal_block(covariances: np.ndarray, indices: tuple[int, ...]) -> np.ndarray:
    rows = np.take(covariances, indices, axis=-2)
    return np.take(rows, indices, axis=-1)


def _validate_indices(
    indices: Sequence[int],
    state_dimension: int,
    label: str,
) -> tuple[int, ...]:
    try:
        values = tuple(operator.index(index) for index in indices)
    except TypeError as error:
        raise ValueError(f"{label} indices must be integers") from error
    if not values:
        raise ValueError(f"{label} indices must not be empty")
    if len(set(values)) != len(values):
        raise ValueError(f"{label} indices must be unique")
    if any(index < 0 or index >= state_dimension for index in values):
        raise ValueError(f"{label} index is outside the error state")
    return values
