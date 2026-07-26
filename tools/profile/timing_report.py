# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import sys
from pathlib import Path

TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from internal.perf_artifacts import DEFAULT_TIMING_PATH, load_timing_artifact, print_timing_report


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize a NavKit timing artifact.")
    parser.add_argument(
        "timing_json",
        type=Path,
        nargs="?",
        default=DEFAULT_TIMING_PATH,
        help="Path to a navkit.timing.v1 JSON artifact.",
    )
    args = parser.parse_args()

    print_timing_report(load_timing_artifact(args.timing_json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
