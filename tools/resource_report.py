# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
from pathlib import Path

from navkit_build_dirs import resolve_build_dir
from perf_artifacts import (
    DEFAULT_NAVKIT_CONFIG,
    default_resource_report_path,
    load_resource_report,
    print_resource_report,
    write_resource_report,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Write a coarse NavKit binary-size report.")
    parser.add_argument("--build-type", choices=["Release", "Debug"], default="Debug")
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--navkit-config", default=DEFAULT_NAVKIT_CONFIG)
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    build_dir = resolve_build_dir(root, args.build_type, args.navkit_config, args.build_dir)
    output = args.output or default_resource_report_path(args.build_type)

    write_resource_report(
        output,
        build_dir=build_dir,
        build_type=args.build_type,
        navkit_config=args.navkit_config,
    )
    print(f"Wrote resource report to {output}")
    if not args.no_display:
        print()
        print_resource_report(load_resource_report(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
