# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import numpy as np


def nis(innovation: np.ndarray, S: np.ndarray) -> float:
    return float(innovation.T @ np.linalg.solve(S, innovation))
