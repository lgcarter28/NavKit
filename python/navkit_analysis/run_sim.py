# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("apps/navkit_sim/configs/stationary_gnss.json"))
    args = parser.parse_args()
    return subprocess.call([str(args.exe), str(args.config)])


if __name__ == "__main__":
    raise SystemExit(main())
