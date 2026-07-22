# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.stats import chi2


def measurement_dof_from_innovations(updates: pd.DataFrame) -> int:
    """Infer measurement degrees of freedom from innovation columns."""

    return len([column for column in updates.columns if column.startswith("nu_")])


def chi_square_threshold(confidence: float, dof: int) -> float:
    """Return the chi-square threshold for the requested confidence and DOF."""

    return float(chi2.ppf(confidence, df=dof))


def chi_square_upper_tail_probability(values: np.ndarray | pd.Series, dof: int) -> np.ndarray:
    """Return upper-tail chi-square probabilities for observed values."""

    return chi2.sf(values, df=dof)
