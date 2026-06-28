from __future__ import annotations

import numpy as np


def rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x))))
